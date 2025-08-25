/**
 * LoRaWan Mapper build for LilyGo TTGO T-Beam v1.2 boards.
 * Copyright (C) 2021-2022 by Max-Plastix
 *
 * This is a development fork by Max-Plastix hosted here:
 * https://github.com/Max-Plastix/tbeam-helium-mapper/
 *
 * This code comes from a number of developers and earlier efforts, visible in
 * the full lineage on Github, including:
 *
 *     Fizzy, longfi-arduino, Kyle T. Gabriel, and Xose Pérez
 *
 * GPL makes this all possible -- continue to modify, extend, and share!
 */

/**
 * Main module
 *
 * Modified by Kyle T. Gabriel to fix issue with incorrect GPS data for TTNMapper
 *
 * Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <esp_bt.h>

#include "configuration.h"
#include "credentials.h"
#include "gps.h"
#include "screen.h"
#include "sleep.h"

#define FPORT_MAPPER 2  // FPort for Uplink messages -- must match Helium Console Decoder script!

#define STATUS_BOOT 1
#define STATUS_USB_ON 2
#define STATUS_USB_OFF 3

bool justSendNow = false;               // Send one at boot, regardless of deadzone?
unsigned long int last_send_ms = 0;     // Time of last uplink
unsigned long int last_moved_ms = 0;    // Time of last movement
unsigned long int last_gpslost_ms = 0;  // Time of last gps-lost packet
unsigned long int last_display_ms = 0;  // Time of last display update
double last_send_lat = 0;               // Last known location
double last_send_lon = 0;               //
double dist_moved = 0;                  // Distance in m from last uplink

// Deadzone (no uplink) location and radius
double deadzone_lat = DEADZONE_LAT;
double deadzone_lon = DEADZONE_LON;
double deadzone_radius_m = DEADZONE_RADIUS_M;
boolean in_deadzone = false;

/* Defaults that can be overwritten by downlink messages */
/* (32-bit int seconds allows for 50 days) */
unsigned int stationary_tx_interval_s;  // prefs STATIONARY_TX_INTERVAL
unsigned int rest_wait_s;               // prefs REST_WAIT
unsigned int rest_tx_interval_s;        // prefs REST_TX_INTERVAL

unsigned int tx_interval_s;  // Currently-active time interval

enum activity_state {
  ACTIVITY_MOVING,
  ACTIVITY_REST,
  ACTIVITY_SLEEP,
  ACTIVITY_GPS_LOST,
  ACTIVITY_WOKE,
  ACTIVITY_INVALID
};
enum activity_state active_state = ACTIVITY_INVALID;
boolean never_rest = NEVER_REST;

// Return status from mapper uplink, since we care about the flavor of the failure
enum mapper_uplink_result { MAPPER_UPLINK_SUCCESS, MAPPER_UPLINK_BADFIX, MAPPER_UPLINK_NOLORA, MAPPER_UPLINK_NOTYET };

/* Maybe these moves to prefs eventually? */
unsigned int sleep_wait_s = SLEEP_WAIT;
unsigned int sleep_tx_interval_s = SLEEP_TX_INTERVAL;

unsigned int gps_lost_wait_s = GPS_LOST_WAIT;
unsigned int gps_lost_ping_s = GPS_LOST_PING;
uint32_t last_fix_time = 0;

float battery_low_voltage = BATTERY_LOW_VOLTAGE;
float min_dist_moved = MIN_DIST;

XPowersLibInterface *PMU = NULL;
bool pmu_irq = false;  // true when PMU IRQ pending

bool oled_found = false;
bool pmu_found = false;
uint8_t oled_addr = 0;  // i2c address of OLED controller

bool packetQueued;
bool isJoined = false;

bool screen_stay_on = false;
bool screen_stay_off = false;
bool is_screen_on = true;
int screen_idle_off_s = SCREEN_IDLE_OFF_S;
int screen_menu_timeout_s = MENU_TIMEOUT_S;
uint32_t screen_last_active_ms = 0;
boolean in_menu = false;
boolean have_usb_power = true;
uint8_t usb_power_count = 0;

// Buffer for Payload frame
static uint8_t txBuffer[11];

// deep sleep support
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_source_t wakeCause;  // the reason we booted this time

char buffer[40];  // Screen buffer

String lorawanServer;
uint8_t lorawanAck = false;
uint8_t lorawan_sf;  // prefs LORAWAN_SF
char sf_name[40];

unsigned long int ack_req = 0;
unsigned long int ack_rx = 0;

// Store Lat & Long in six bytes of payload
void pack_lat_lon(double lat, double lon) {
  uint32_t LatitudeBinary;
  uint32_t LongitudeBinary;
  LatitudeBinary = ((lat + 90) / 180.0) * 16777215;
  LongitudeBinary = ((lon + 180) / 360.0) * 16777215;

  txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
  txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;
  txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
  txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;
}

uint8_t battery_byte(void) {
  uint16_t batteryVoltage = ((float_t)((float_t)(PMU->getBattVoltage()) / 10.0) + .5);
  return (uint8_t)((batteryVoltage - 200) & 0xFF);
}

// Prepare a packet for the Mapper
void build_mapper_packet() {
  double lat;
  double lon;
  uint16_t altitudeGps;
  uint8_t sats;

  lat = tGPS.location.lat();
  lon = tGPS.location.lng();
  pack_lat_lon(lat, lon);
  altitudeGps = (uint16_t)tGPS.altitude.meters();
  sats = tGPS.satellites.value();

  sprintf(buffer, "Lat: %f, ", lat);
  Serial.print(buffer);
  sprintf(buffer, "Long: %f, ", lon);
  Serial.print(buffer);
  sprintf(buffer, "Alt: %f, ", tGPS.altitude.meters());
  Serial.print(buffer);
  sprintf(buffer, "Sats: %d", sats);
  Serial.println(buffer);

  txBuffer[6] = (altitudeGps >> 8) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  txBuffer[8] = sats & 0xFF;
}

/// Blow away our prefs (i.e. to rejoin from scratch)
void ttn_erase_prefs() {
  node.clearSession();
  Preferences p;
  if (p.begin("lora", false)) {
    p.clear();
    p.end();
  }
}

// Helium requires a FCount reset sometime before hitting 0xFFFF
// 50,000 makes it obvious it was intentional
#define MAX_FCOUNT 50000

boolean send_uplink(uint8_t *txBuffer, uint8_t length, uint8_t fport, boolean confirmed) {
  if (confirmed) {
    Serial.println("ACK requested");
    screen_print("? ");
    digitalWrite(RED_LED, LOW);  // Light LED
    ack_req++;
  }

  // send it!
  // set battery fill level - the LoRaWAN network server
  // may periodically request this information
  // 0 = external power source
  // 1 = lowest (empty battery)
  // 254 = highest (full battery)
  // 255 = unable to measure
  uint8_t battLevel = 146;
  if (PMU->isBatteryConnect()) {
    battLevel = int(PMU->getBatteryPercent() * 2.53);
  } else {
    battLevel = 0;
  }
  node.setDeviceStatus(battLevel);
  packetQueued = true;
  int16_t state = RADIOLIB_ERR_NONE;
  uint8_t downlinkPayload[10];  // Make sure this fits your plans!
  size_t downlinkSize;          // To hold the actual payload size received
  LoRaWANEvent_t uplinkDetails;
  LoRaWANEvent_t downlinkDetails;
  if (confirmed) {
    node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
    node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);
    state = node.sendReceive(txBuffer, length, fport, downlinkPayload, &downlinkSize, confirmed, &uplinkDetails,
                             &downlinkDetails);
  } else {
    state = node.sendReceive(txBuffer, length, fport, downlinkPayload, &downlinkSize, confirmed, &uplinkDetails,
                             &downlinkDetails);
  }
  Serial.print("Send result: ");
  Serial.println(state);

  // Check for error:
  if( state == RADIOLIB_ERR_NETWORK_NOT_JOINED){
    screen_print("\nNot Joined!\n"); 
  }

  // If things got returned:
  // Check if downlink was received
  // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
  if (state > 0) {
    // Did we get a downlink with data for us
    if (downlinkSize > 0) {
      Serial.println(F("Downlink data"));
    } else {
      Serial.println(F("<MAC commands only>"));
    }

    // print RSSI (Received Signal Strength Indicator)
    Serial.print(F("[LoRaWAN] RSSI:\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print SNR (Signal-to-Noise Ratio)
    Serial.print(F("[LoRaWAN] SNR:\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

    // print frequency error
    Serial.print(F("[LoRaWAN] Frequency error:\t"));
    Serial.print(radio.getFrequencyError());
    Serial.println(F(" Hz"));

    // print extra information about the event
    Serial.println(F("[LoRaWAN] Event information:"));
    Serial.print(F("[LoRaWAN] Confirmed:\t"));
    Serial.println(downlinkDetails.confirmed);
    Serial.print(F("[LoRaWAN] Confirming:\t"));
    Serial.println(downlinkDetails.confirming);
    Serial.print(F("[LoRaWAN] Datarate:\t"));
    Serial.println(downlinkDetails.datarate);
    Serial.print(F("[LoRaWAN] Frequency:\t"));
    Serial.print(downlinkDetails.freq, 3);
    Serial.println(F(" MHz"));
    Serial.print(F("[LoRaWAN] Output power:\t"));
    Serial.print(downlinkDetails.power);
    Serial.println(F(" dBm"));
    Serial.print(F("[LoRaWAN] Frame count:\t"));
    Serial.println(downlinkDetails.fCnt);
    Serial.print(F("[LoRaWAN] Port:\t\t"));
    Serial.println(downlinkDetails.fPort);

    uint8_t margin = 0;
    uint8_t gwCnt = 0;
    if (node.getMacLinkCheckAns(&margin, &gwCnt) == RADIOLIB_ERR_NONE) {
      Serial.print(F("[LoRaWAN] LinkCheck margin:\t"));
      Serial.println(margin);
      Serial.print(F("[LoRaWAN] LinkCheck count:\t"));
      Serial.println(gwCnt);
    }

    uint32_t networkTime = 0;
    uint8_t fracSecond = 0;
    if (node.getMacDeviceTimeAns(&networkTime, &fracSecond, true) == RADIOLIB_ERR_NONE) {
      Serial.print(F("[LoRaWAN] DeviceTime Unix:\t"));
      Serial.println(networkTime);
      Serial.print(F("[LoRaWAN] DeviceTime second:\t1/"));
      Serial.println(fracSecond);
    }
  }

  // Helium requires a re-join / reset of count to avoid 16bit count rollover
  // Hopefully a device reboot every 50k uplinks is no problem.
  if (node.getFCntUp() > MAX_FCOUNT) {
    Serial.println("FCount Rollover!");

    // I don't understand why this doesn't show at all
    screen_print("\n\nRollover Reset!\n");
    screen_update();
    delay(1000);  // Give some time to read the screen

    ttn_erase_prefs();
    ESP.restart();
  }

  return true;
}

// LoRa message event callback
void lora_msg_callback(const _ev_t message) {
  static boolean seen_joined = false, seen_joining = false;

  // This is confusing because JOINED is sometimes spoofed and comes early
  if (EV_JOINED == message)
    seen_joined = true;
  if (EV_JOINING == message)
    seen_joining = true;
  if (!isJoined && seen_joined && seen_joining) {
    isJoined = true;
    screen_print("Joined Network!\n");
    // ttn_set_sf(lorawan_sf);  // Joining seems to leave it at SF10?
    // ttn_get_sf_name(sf_name, sizeof(sf_name));
  }

  if (EV_TXSTART == message) {
    screen_print("+\v");
    screen_update();
  }
  // We only want to say 'packetSent' for our packets (not packets needed for
  // joining)
  if (EV_TXCOMPLETE == message && packetQueued) {
    //    screen_print("sent.\n");
    packetQueued = false;
    if (pmu_found)
      PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);
  }

  if (EV_ACK == message) {
    digitalWrite(RED_LED, HIGH);
    ack_rx++;
    Serial.printf("ACK! %lu / %lu\n", ack_rx, ack_req);
    screen_print("! ");
  }

  if (EV_JOIN_FAILED == message) {
    digitalWrite(RED_LED, HIGH);
  }

}

// Send a packet, if one is warranted
enum mapper_uplink_result mapper_uplink() {
  double now_lat = tGPS.location.lat();
  double now_lon = tGPS.location.lng();
  unsigned long int now = millis();

  if (!justSendNow) {
    // Here we try to filter out bogus GPS readings.
    if (!(tGPS.location.isValid() && tGPS.time.isValid() && tGPS.satellites.isValid() && tGPS.hdop.isValid() &&
          tGPS.altitude.isValid() && tGPS.speed.isValid()))
      return MAPPER_UPLINK_BADFIX;

    // Filter out any reports while we have low satellite count.  The receiver can old a fix on 3, but it's poor.
    if (tGPS.satellites.value() < 4)
      return MAPPER_UPLINK_BADFIX;

    // HDOP is only a hint as to accuracy, but we can assume very bad HDOP is not worth mapping.
    // https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation) suggests 5 is a good cutoff.
    if (tGPS.hdop.hdop() > 5.0)
      return MAPPER_UPLINK_BADFIX;

    // With the exception of a few places, a perfectly zero lat or long probably means we got a bad reading
    if (now_lat == 0.0 || now_lon == 0.0)
      return MAPPER_UPLINK_BADFIX;

    // Don't attempt to send or update until we join Helium
    if (!isJoined)
      return MAPPER_UPLINK_NOLORA;

    // LoRa is not ready for a new packet, maybe still sending the last one.
    if (!node.isActivated())
      return MAPPER_UPLINK_NOLORA;

    // Check if there is not a current TX/RX job running
    if (node.timeUntilUplink() > 1)
      return MAPPER_UPLINK_NOLORA;
  }
  // distance from last transmitted location
  double dist_moved = tGPS.distanceBetween(last_send_lat, last_send_lon, now_lat, now_lon);
  double deadzone_dist = tGPS.distanceBetween(deadzone_lat, deadzone_lon, now_lat, now_lon);
  in_deadzone = (deadzone_dist <= deadzone_radius_m);

  /*
  Serial.printf("[Time %lu / %us, Moved %dm in %lus %c]\n", (now - last_send_ms) / 1000, tx_interval_s,
  (int32_t)dist_moved, (now - last_moved_ms) / 1000, in_deadzone ? 'D' : '-');
  */

  // Deadzone means we don't send unless asked
  if (in_deadzone && !justSendNow)
    return MAPPER_UPLINK_NOTYET;

  // Want an ACK on this one?
  bool confirmed;
  if (justSendNow) {
    confirmed = true;
  } else {
    confirmed = (lorawanAck > 0) && (node.getFCntUp() % lorawanAck == 0);
  }

  char because = '?';
  if (justSendNow) {
    justSendNow = false;
    Serial.println("** JUST_SEND_NOW");
    because = '>';
  } else if (dist_moved > min_dist_moved) {
    Serial.println("** DIST");
    last_moved_ms = now;
    because = 'D';
  } else if (now - last_send_ms > tx_interval_s * 1000) {
    Serial.println("** TIME");
    because = 'T';
  } else {
    return MAPPER_UPLINK_NOTYET;  // Nothing to do, go home early
  }

  // The first distance-moved is crazy, since has no origin.. don't put it on
  // screen.
  if (dist_moved > 1000000)
    dist_moved = 0;

  snprintf(buffer, sizeof(buffer), "\n%d %c %4lus %4.0fm ", node.getFCntUp(), because, (now - last_send_ms) / 1000,
           dist_moved);
  screen_print(buffer);

  // prepare the LoRa frame
  build_mapper_packet();

  // Send it!
  lora_msg_callback(EV_TXSTART);
  if (!send_uplink(txBuffer, 9, FPORT_MAPPER, confirmed))
    return MAPPER_UPLINK_NOLORA;

  last_send_ms = now;
  last_send_lat = now_lat;
  last_send_lon = now_lon;

  screen_last_active_ms = now;
  lora_msg_callback(EV_TXCOMPLETE);
  return MAPPER_UPLINK_SUCCESS;  // We did it!
}

void mapper_restore_prefs(void) {
  Preferences p;
  if (p.begin("mapper", true)) {  // Read-only
    min_dist_moved = p.getFloat("min_dist", MIN_DIST);
    stationary_tx_interval_s = p.getUInt("tx_interval", STATIONARY_TX_INTERVAL);
    never_rest = p.getBool("never_rest", NEVER_REST);
    rest_wait_s = p.getUInt("rest_wait", REST_WAIT);
    rest_tx_interval_s = p.getUInt("rest_tx", REST_TX_INTERVAL);
    sleep_wait_s = p.getUInt("sleep_wait", SLEEP_WAIT);
    sleep_tx_interval_s = p.getUInt("sleep_tx", SLEEP_TX_INTERVAL);
    gps_lost_wait_s = p.getUInt("gps_lost_wait", GPS_LOST_WAIT);
    gps_lost_ping_s = p.getUInt("gps_lost_ping", GPS_LOST_PING);
    // Close the Preferences
    p.end();
  } else {
    Serial.println("No Mapper prefs -- using defaults.");
    min_dist_moved = MIN_DIST;
    stationary_tx_interval_s = STATIONARY_TX_INTERVAL;
    never_rest = NEVER_REST;
    rest_wait_s = REST_WAIT;
    rest_tx_interval_s = REST_TX_INTERVAL;
    sleep_wait_s = SLEEP_WAIT;
    sleep_tx_interval_s = SLEEP_TX_INTERVAL;
    gps_lost_wait_s = GPS_LOST_WAIT;
    gps_lost_ping_s = GPS_LOST_PING;
  }

  tx_interval_s = stationary_tx_interval_s;
}

void mapper_save_prefs(void) {
  Preferences p;

  Serial.println("Saving mapper prefs.");
  if (p.begin("mapper", false)) {
    p.putFloat("min_dist", min_dist_moved);
    p.putUInt("tx_interval", stationary_tx_interval_s);
    p.putBool("never_rest", never_rest);
    p.putUInt("rest_wait", rest_wait_s);
    p.putUInt("rest_tx", rest_tx_interval_s);
    p.putUInt("sleep_wait", sleep_wait_s);
    p.putUInt("sleep_tx", sleep_tx_interval_s);
    p.putUInt("gps_lost_wait", gps_lost_wait_s);
    p.putUInt("gps_lost_ping", gps_lost_ping_s);
    p.end();
  }
}

void mapper_erase_prefs(void) {
#if 0
    nvs_flash_erase(); // erase the NVS partition and...
    nvs_flash_init(); // initialize the NVS partition.
#endif
  Preferences p;
  if (p.begin("mapper", false)) {
    p.clear();
    p.end();
  }
}

void lorawan_restore_prefs(void) {
  Preferences p;
  if (p.begin("lora", true)) {  // Read-only
    lorawanAck = p.getUChar("ack", LORAWAN_CONFIRMED_EVERY);
    lorawan_sf = p.getUChar("sf", LORAWAN_SF);
    lorawanServer = p.getString("server", "helium");
    // a buffer that holds all LW base parameters that should persist at all times!
    uint8_t BbufferNonces[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
    p.getBytes("nonces", BbufferNonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    // a buffer that holds all LW session parameters that preferably persist, but can be afforded to get lost
    uint8_t BbufferSession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
    p.getBytes("session", BbufferSession, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

    uint32_t state;
    state = node.setBufferNonces(BbufferNonces);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("set nonces success!"));
    } else {
      Serial.print(F("set nonces failed, code "));
      Serial.println(state);
    }

    state = node.setBufferSession(BbufferSession);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("set session success!"));
    } else {
      Serial.print(F("set session failed, code "));
      Serial.println(state);
    }

    /** Close the Preferences */
    p.end();
  } else {
    Serial.println("No lorawan prefs -- using defaults.");
    lorawanAck = LORAWAN_CONFIRMED_EVERY;
    lorawan_sf = LORAWAN_SF;
  }
}

void lorawan_save_prefs(void) {
  Preferences p;
  Serial.println("Saving lorawan prefs.");
  if (p.begin("lora", false)) {
    p.putString("server", lorawanServer);
    p.putUChar("sf", lorawan_sf);
    p.putUChar("ack", lorawanAck);
    // ##### save the join counters (nonces) to permanent store
    Serial.println(F("Saving nonces to flash"));
    uint8_t* noncesPtr = node.getBufferNonces();
    p.putBytes("nonces", noncesPtr, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    uint8_t* sessionPtr  = node.getBufferSession();
    p.putBytes("session", sessionPtr, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    p.end();
  }
}

// Clear all saves surrounding the Lora setting
void lorawan_erase_prefs(void) {
  Preferences p;
  node.clearSession();
  if (p.begin("lora", false)) {
    p.clear();
    p.end();
  }
}

void deadzone_restore_prefs(void) {
  Preferences p;
  if (p.begin("deadzone", true)) {  // Read-only
    deadzone_lat = p.getDouble("lat", DEADZONE_LAT);
    deadzone_lon = p.getDouble("lon", DEADZONE_LON);
    deadzone_radius_m = p.getUInt("radius", DEADZONE_RADIUS_M);
    /** Close the Preferences */
    p.end();
  } else {
    Serial.println("No deadzone prefs -- using defaults.");
    deadzone_lat = DEADZONE_LAT;
    deadzone_lon = DEADZONE_LON;
    deadzone_radius_m = DEADZONE_RADIUS_M;
  }
}

void deadzone_save_prefs(void) {
  Preferences p;
  Serial.println("Saving deadzone prefs.");
  if (p.begin("deadzone", false)) {
    p.putDouble("lat", deadzone_lat);
    p.putDouble("lon", deadzone_lon);
    p.putUInt("radius", deadzone_radius_m);
    p.end();
  }
}

void deadzone_erase_prefs(void) {
  Preferences p;
  if (p.begin("deadzone", false)) {
    p.clear();
    p.end();
  }
}

void screen_restore_prefs(void) {
  Preferences p;
  if (p.begin("screen", true)) {  // Read-only
    screen_idle_off_s = p.getInt("off_time", SCREEN_IDLE_OFF_S);
    screen_menu_timeout_s = p.getInt("menu_timeout", MENU_TIMEOUT_S);
    /** Close the Preferences */
    p.end();
  } else {
    Serial.println("No screen prefs -- using defaults.");
    screen_idle_off_s = SCREEN_IDLE_OFF_S;
    screen_menu_timeout_s = MENU_TIMEOUT_S;
  }
}

void screen_save_prefs(void) {
  Preferences p;
  Serial.println("Saving screen prefs.");
  if (p.begin("screen", false)) {
    p.putDouble("off_time", screen_idle_off_s);
    p.putDouble("menu_timeout", screen_menu_timeout_s);
    p.end();
  }
}

void screen_erase_prefs(void) {
  Preferences p;
  if (p.begin("screen", false)) {
    p.clear();
    p.end();
  }
}

void scanI2CDevice(void) {
  byte err, addr;
  int nDevices = 0;
  for (addr = 1; addr < 0x7F; addr++) {
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0) {
      // Serial.printf("I2C device found at address 0x%X !\r\n", addr);
      nDevices++;
      if (addr == 0x3C || addr == 0x78 || addr == 0x7E) {
        oled_addr = addr;
        oled_found = true;
        Serial.printf("OLED at 0x%02X\r\n", oled_addr);
      }
      if (addr == AXP2101_SLAVE_ADDRESS) {
        pmu_found = true;
        Serial.printf("AXP192/AXP2101 PMU at 0x%02X\r\n", addr);
      }
    } else if (err == 4) {
      Serial.printf("Unknown i2c device at 0x%02X\r\n", addr);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found!\r\n");
  }
}

/**
 * The AXP library computes this incorrectly for AXP192.
 * It's just a fixed mapping table from the datasheet
 */
int axp_charge_to_ma(int set) {
  int32_t ma_list[16] = {
      100,  190,  280,  360,   // 0, 1, 2, 3
      450,  550,  630,  700,   // 4, 5, 6, 7
      780,  880,  960,  1000,  // 8, 9, 10, 11
      1080, 1160, 1240, 1320,  // 12, 13, 14, 15
  };
  if (set >= 0 && set < sizeof(ma_list)) {
    return ma_list[set];
  } else {
    return -1;
  }
}

/**
 * Initialize the AXP192 power manager chip.
 *
 * DCDC1 0.7-3.5V @ 1200mA max -> OLED
 * If you turn the OLED off, it will drag down the I2C lines and block the bus
 * from the AXP192 which shares it.
 * Use SSD1306 sleep mode instead
 *
 * DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
 * LDO1 30mA -> "VCC_RTC" charges GPS tiny J13 backup battery
 * LDO2 200mA -> "LORA_VCC"
 * LDO3 200mA -> "GPS_VCC"
 */
void axpInit() {
  if (!pmu_found) {
    Serial.println("AXP192/AXP2101 PMU not found!");
    return;
  }

  if (!PMU) {
    PMU = new XPowersAXP2101(Wire);
    if (!PMU->init()) {
      Serial.println("Warning: Failed to find AXP2101 power management");
      delete PMU;
      PMU = NULL;
    } else {
      Serial.println("AXP2101 PMU init succeeded, using AXP2101 PMU");
    }
  }

  if (!PMU) {
    PMU = new XPowersAXP192(Wire);
    if (!PMU->init()) {
      Serial.println("Warning: Failed to find AXP192 power management");
      delete PMU;
      PMU = NULL;
    } else {
      Serial.println("AXP192 PMU init succeeded, using AXP192 PMU");
    }
  }

  if (!PMU) {
    pmu_found = false;
    return;
  }

  if (PMU->getChipModel() == XPOWERS_AXP192) {
    PMU->setProtectedChannel(XPOWERS_DCDC3);

    // lora
    PMU->setPowerChannelVoltage(XPOWERS_LDO2, 3300);
    // gps
    PMU->setPowerChannelVoltage(XPOWERS_LDO3, 3300);
    // oled
    PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);

    PMU->enablePowerOutput(XPOWERS_LDO2);
    PMU->enablePowerOutput(XPOWERS_LDO3);

    // protected oled power source
    PMU->setProtectedChannel(XPOWERS_DCDC1);
    // protected esp32 power source
    PMU->setProtectedChannel(XPOWERS_DCDC3);
    // enable oled power
    PMU->enablePowerOutput(XPOWERS_DCDC1);

    // disable not use channel
    PMU->disablePowerOutput(XPOWERS_DCDC2);

  } else if (PMU->getChipModel() == XPOWERS_AXP2101) {
    // Unuse power channel
    PMU->disablePowerOutput(XPOWERS_DCDC2);
    PMU->disablePowerOutput(XPOWERS_DCDC3);
    PMU->disablePowerOutput(XPOWERS_DCDC4);
    PMU->disablePowerOutput(XPOWERS_DCDC5);
    PMU->disablePowerOutput(XPOWERS_ALDO1);
    PMU->disablePowerOutput(XPOWERS_ALDO4);
    PMU->disablePowerOutput(XPOWERS_BLDO1);
    PMU->disablePowerOutput(XPOWERS_BLDO2);
    PMU->disablePowerOutput(XPOWERS_DLDO1);
    PMU->disablePowerOutput(XPOWERS_DLDO2);

    // GNSS RTC PowerVDD 3300mV
    PMU->setPowerChannelVoltage(XPOWERS_VBACKUP, 3300);
    PMU->enablePowerOutput(XPOWERS_VBACKUP);

    // ESP32 VDD 3300mV
    //  ! No need to set, automatically open , Don't close it
    //  PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);
    //  PMU->setProtectedChannel(XPOWERS_DCDC1);
    PMU->setProtectedChannel(XPOWERS_DCDC1);

    // LoRa VDD 3300mV
    PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO2);

    // GNSS VDD 3300mV
    PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO3);

    // Charge with 300mA
    PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_300MA);
  }

  // Flash the Blue LED until our first packet is transmitted
  PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
  // PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);

  // Fire an interrupt on falling edge.  Note that some IRQs repeat/persist.
  pinMode(PMU_IRQ, INPUT);
  gpio_pullup_en((gpio_num_t)PMU_IRQ);
  attachInterrupt(PMU_IRQ, [] { pmu_irq = true; }, FALLING);

  // Configure REG 36H: PEK press key parameter set.  Index values for
  // argument!
  PMU->setPowerKeyPressOnTime(XPOWERS_POWERON_2S);
  PMU->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);

  have_usb_power = PMU->isVbusIn();
  Serial.printf("Battery Charge Level: %d%%\n", PMU->getBatteryPercent());

  Serial.printf("=========================================\n");
  if (PMU->isChannelAvailable(XPOWERS_DCDC1)) {
    Serial.printf("DC1  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC1) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_DCDC1));
  }
  if (PMU->isChannelAvailable(XPOWERS_DCDC2)) {
    Serial.printf("DC2  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC2) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_DCDC2));
  }
  if (PMU->isChannelAvailable(XPOWERS_DCDC3)) {
    Serial.printf("DC3  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC3) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_DCDC3));
  }
  if (PMU->isChannelAvailable(XPOWERS_DCDC4)) {
    Serial.printf("DC4  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC4) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_DCDC4));
  }
  if (PMU->isChannelAvailable(XPOWERS_DCDC5)) {
    Serial.printf("DC5  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC5) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_DCDC5));
  }
  if (PMU->isChannelAvailable(XPOWERS_LDO2)) {
    Serial.printf("LDO2 : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_LDO2) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_LDO2));
  }
  if (PMU->isChannelAvailable(XPOWERS_LDO3)) {
    Serial.printf("LDO3 : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_LDO3) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_LDO3));
  }
  if (PMU->isChannelAvailable(XPOWERS_ALDO1)) {
    Serial.printf("ALDO1: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO1) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_ALDO1));
  }
  if (PMU->isChannelAvailable(XPOWERS_ALDO2)) {
    Serial.printf("ALDO2: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO2) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_ALDO2));
  }
  if (PMU->isChannelAvailable(XPOWERS_ALDO3)) {
    Serial.printf("ALDO3: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO3) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_ALDO3));
  }
  if (PMU->isChannelAvailable(XPOWERS_ALDO4)) {
    Serial.printf("ALDO4: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO4) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_ALDO4));
  }
  if (PMU->isChannelAvailable(XPOWERS_BLDO1)) {
    Serial.printf("BLDO1: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_BLDO1) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_BLDO1));
  }
  if (PMU->isChannelAvailable(XPOWERS_BLDO2)) {
    Serial.printf("BLDO2: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_BLDO2) ? "+" : "-",
                  PMU->getPowerChannelVoltage(XPOWERS_BLDO2));
  }
  Serial.printf("=========================================\n");

  // It is necessary to disable the detection function of the TS pin on the board
  // without the battery temperature detection function, otherwise it will cause abnormal charging
  PMU->disableTSPinMeasure();

    // Enable internal ADC detection
  PMU->enableBattDetection();
  PMU->enableVbusVoltageMeasure();
  PMU->enableBattVoltageMeasure();
  PMU->enableSystemVoltageMeasure();

  // Call the interrupt request through the interface class
  PMU->disableInterrupt(XPOWERS_ALL_INT);

  PMU->enableInterrupt(XPOWERS_USB_INSERT_INT | XPOWERS_USB_REMOVE_INT | XPOWERS_BATTERY_INSERT_INT |
                       XPOWERS_BATTERY_REMOVE_INT | XPOWERS_PWR_BTN_CLICK_INT | XPOWERS_CHARGE_START_INT |
                       XPOWERS_CHARGE_DONE_INT | XPOWERS_PWR_BTN_LONGPRESSED_INT);

  // Clear all interrupt flags
  PMU->clearIrqStatus();
}

/**
 * Perform power on init that we do on each wake from deep sleep
 */
void wakeup() {
  bootCount++;
  wakeCause = esp_sleep_get_wakeup_cause();

  Serial.printf("BOOT #%d!  cause:%d ext1:%08llx\n", bootCount, wakeCause, esp_sleep_get_ext1_wakeup_status());
}

void setup() {
  // Debug
#ifdef DEBUG_PORT
  DEBUG_PORT.begin(SERIAL_BAUD);
#endif
  wakeup();

  // Buttons & LED
  pinMode(MIDDLE_BUTTON_PIN, INPUT);
  gpio_pullup_en((gpio_num_t)MIDDLE_BUTTON_PIN);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);  // Off

  mapper_restore_prefs();
  // lorawan_restore_prefs();
  deadzone_restore_prefs();
  screen_restore_prefs();

  /** Make sure WiFi and BT are off */
  // WiFi.disconnect(true);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();

  Wire.begin(I2C_SDA, I2C_SCL);
  SPI.begin(SCK_GPIO, MISO_GPIO, MOSI_GPIO, NSS_GPIO);

  scanI2CDevice();

  axpInit();

  // GPS sometimes gets wedged with no satellites in view and only a power-cycle
  // saves it. Here we turn off power and the delay in screen setup is enough
  // time to bonk the GPS
  if (PMU) {
    if (PMU->getChipModel() == XPOWERS_AXP192) {
      PMU->disablePowerOutput(XPOWERS_LDO3);
    } else if (PMU->getChipModel() == XPOWERS_AXP2101) {
      PMU->disablePowerOutput(XPOWERS_ALDO3);
    }
  }

  // Hello
  DEBUG_MSG("\n" APP_NAME " " APP_VERSION "\n");

  // mapper_restore_prefs();  // Fetch saved settings

  /**
   * This creates the display object, so if we don't call it..
   * all screen ops are do-nothing.
   */
  if (oled_found) {
    screen_setup(oled_addr);
  }
  is_screen_on = true;

  /** GPS power on, so it has time to settle. */
  if (PMU) {
    if (PMU->getChipModel() == XPOWERS_AXP192) {
      PMU->enablePowerOutput(XPOWERS_LDO3);
    } else if (PMU->getChipModel() == XPOWERS_AXP2101) {
      PMU->enablePowerOutput(XPOWERS_ALDO3);
    }
  }

  /** Show logo on first boot (as opposed to wake) */
  if (bootCount <= 1) {
    screen_print(APP_NAME " " APP_VERSION, 0, 0);  // Above the Logo
    // screen_print(APP_NAME " " APP_VERSION "\n");   // Add it to the log too

    screen_update();
    delay(LOGO_DELAY);
  }

  // LoRaWan setup
  int16_t state = 0;  // return value for calls to RadioLib
  state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);

  Serial.print(F("[LoRaWAN] Resuming previous session ... "));

  node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);

  lorawan_restore_prefs();
  lora_msg_callback(EV_JOINING);

  state = node.activateOTAA();

  if ((state == RADIOLIB_ERR_NONE)||(state == RADIOLIB_LORAWAN_NEW_SESSION)||(state == RADIOLIB_LORAWAN_SESSION_RESTORED)) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("Restore failed, code "));
    Serial.println(state);
    // while(true);
    // node.wipe();

    Serial.print(F("[LoRaWAN] Attempting over-the-air activation ... "));
    node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    state = node.activateOTAA();

    node.setADR(false);

    if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("Activation failed, code "));
      lora_msg_callback(EV_JOIN_FAILED);
      Serial.println(state);

      // Check for the specific -1116 error (No JoinAccept received)
      if (state == RADIOLIB_ERR_NO_JOIN_ACCEPT) { // This constant equals -1116
          // Show a more descriptive error for this specific case
          screen_print("No JoinAccept\nCheck Keys/Range\n\nPress middle button\nto retry.", 0, 20);
      } else {
          // Show the generic failure message for all other errors
          screen_print("Join failed!\nPress middle button\nto retry.", 0, 20);
      }
      screen_update();

      // Loop forever until the middle button is pressed to retry
      while (true) {
          if (!digitalRead(MIDDLE_BUTTON_PIN)) {
              Serial.println("Middle button pressed. Retrying activation by restarting...");
              screen_clear();
              screen_update();
              ESP.restart();
          }
          delay(50);
      }

        //while (true);
    }
  }

  // Print the DevAddr
  Serial.print("[LoRaWAN] DevAddr: ");
  Serial.println((unsigned long)node.getDevAddr(), HEX);
  lora_msg_callback(EV_JOINED);

  // DataRate 5 = SF7BW125KHZ
  node.setDatarate(5);
  // TX Power to 16dBm (max for EU868)
  node.setTxPower(16);

  // on EEPROM enabled boards, you can save the current session
  // by calling "saveSession" which allows retrieving the session after reboot or deepsleep
  // node.saveSession();
  lorawan_save_prefs();

  /**
   * Might have to add a longer delay here for GPS boot-up.
   * Takes longer to sync if we talk to it too early.
   */
  delay(100);
  gps_setup(true);  // Init GPS baudrate and messages

  /** This is bad.. we can't find the AXP192 PMIC, so no menu key detect: */
  if (!pmu_found) {
    screen_print("** Missing AXP192! **\n");
  }

  Serial.printf("Deadzone: %f.0m @ %f, %f\n", deadzone_radius_m, deadzone_lat, deadzone_lon);
}

// Should be around 0.5mA ESP32 consumption, plus OLED controller and PMIC overhead.
void low_power_sleep(uint32_t seconds) {
  boolean was_screen_on = is_screen_on;

  Serial.printf("Sleep %d..\n", seconds);
  Serial.flush();

  screen_off();

  digitalWrite(RED_LED, HIGH);  // LED Off

  if (pmu_found) {
    if (PMU) {
      if (PMU->getChipModel() == XPOWERS_AXP192) {
        PMU->disablePowerOutput(XPOWERS_LDO3);
      } else if (PMU->getChipModel() == XPOWERS_AXP2101) {
        PMU->disablePowerOutput(XPOWERS_ALDO3);
      }
    }
    // axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS power
    PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);  // Blue LED off

    // Turning off DCDC1 consumes MORE power, for reasons unknown
    // axp.setPowerOutPut(AXP192_DCDC1, AXP202_OFF);  // OLED power off
    // pinMode(I2C_SCL, OUTPUT);
    // digitalWrite(I2C_SCL, HIGH);                    // Must enable pull-up on SCL to keep AXP accessible
  }

  // Wake on either button press
  gpio_wakeup_enable((gpio_num_t)MIDDLE_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)PMU_IRQ, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  esp_sleep_enable_timer_wakeup(seconds * 1000ULL * 1000ULL);  // call expects usecs
  // Some GPIOs need this to stay on?
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  esp_light_sleep_start();
  // If we woke by keypress (7) then turn on the screen
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    // Try not to puke, but we pretend we moved if they hit a key, to exit SLEEP and restart timers
    last_moved_ms = screen_last_active_ms = millis();
    was_screen_on = true;  // Lies
    Serial.println("(GPIO)");
  }
  Serial.println("..woke");

  if (was_screen_on) {
    screen_on();
  }

  if (pmu_found) {
    if (PMU) {
      if (PMU->getChipModel() == XPOWERS_AXP192) {
        PMU->enablePowerOutput(XPOWERS_LDO3);
      } else if (PMU->getChipModel() == XPOWERS_AXP2101) {
        PMU->enablePowerOutput(XPOWERS_ALDO3);
      }
    }
    // axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS power
    // axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // OLED power
    // if (oled_found)
    //  screen_setup();
  }

  delay(100);        // GPS doesn't respond right away.. not ready for baud-rate test.
  gps_setup(false);  // Resync with GPS
}

/** Power OFF -- does not return */
void clean_shutdown(void) {
  /** cleanly shutdown the radio */
  Serial.println("Shutdown.");
  // LMIC_shutdown();
  mapper_save_prefs();
  lorawan_save_prefs();
  deadzone_save_prefs();
  screen_save_prefs();
  // ttn_write_prefs();
  if (pmu_found) {
    /** Surprisingly sticky if you don't set it */
    PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);
    /** PMIC power off */
    // axp.shutdown();
    PMU->shutdown();
  } else {
    while (1);  // ?? What to do here.  burn power?
  }
}

uint32_t woke_time_ms = 0;
uint32_t woke_fix_count = 0;

/** Determine the current activity state */
void update_activity() {
  static enum activity_state last_active_state = ACTIVITY_INVALID;

  if (active_state != last_active_state) {
    switch (active_state) {
      case ACTIVITY_MOVING:
        Serial.println("//MOVING//");
        screen_print("\nMoving");
        break;
      case ACTIVITY_REST:
        Serial.println("//REST//");
        break;
      case ACTIVITY_SLEEP:
        Serial.println("//SLEEP//");
        break;
      case ACTIVITY_GPS_LOST:
        Serial.println("//GPS_LOST//");
        screen_print("\nGPS Lost");
        break;
      case ACTIVITY_WOKE:
        Serial.println("//WOKE//");
        break;
      default:
        Serial.println("//WTF?//");
        break;
    }
    last_active_state = active_state;
  }

  uint32_t now = millis();
  float bat_volts = PMU->getBattVoltage() / 1000.0;
  // float charge_ma = axp.getBattChargeCurrent();

  if (pmu_found && PMU->isBatteryConnect() && bat_volts < battery_low_voltage
      /*&& \
      charge_ma < 99.0*/
  ) {
    Serial.print("Low Battery OFF ");
    Serial.println(bat_volts);
    screen_print("\nLow Battery OFF\n");
    delay(4999);  // Give some time to read the screen
    clean_shutdown();
  }

  // Here we just woke from a GPS-off long sleep.
  // When we have a fresh GPS fix, and the fix qualifies for mapper report, we can resume
  // either mapping or going back to sleep.  Until then, we loop in Wake looking for a good GPS signal.
  // Note that we have to be sensitive to "good fix, but not interesting" and go right back to sleep.
  // We're only staying awake until we got a good GPS fix or gave up, NOT until we send a mapper report.
  if (active_state == ACTIVITY_WOKE) {
    if (tGPS.sentencesWithFix() != woke_fix_count && mapper_uplink() != MAPPER_UPLINK_BADFIX) {
      active_state = ACTIVITY_REST;
    } else if (now - woke_time_ms > gps_lost_wait_s * 1000) {
      active_state = ACTIVITY_GPS_LOST;
    }
    return;  // else stay in WOKE until we make a good report
  }

  if (active_state == ACTIVITY_SLEEP && !in_menu) {
    low_power_sleep(tx_interval_s);
    active_state = ACTIVITY_WOKE;
    woke_time_ms = millis();
    woke_fix_count = tGPS.sentencesWithFix();
    return;
  }

  // In order of precedence:
  if (never_rest) {
    active_state = ACTIVITY_MOVING;
  } else if (now - last_moved_ms > sleep_wait_s * 1000) {
    active_state = ACTIVITY_SLEEP;
  } else if (now - last_fix_time > gps_lost_wait_s * 1000) {
    active_state = ACTIVITY_GPS_LOST;
  } else if (now - last_moved_ms > rest_wait_s * 1000) {
    active_state = ACTIVITY_REST;
  } else {
    active_state = ACTIVITY_MOVING;
  }

  // If we have USB power, keep GPS on all the time; don't sleep
  if (have_usb_power) {
    if (active_state == ACTIVITY_SLEEP) {
      active_state = ACTIVITY_REST;
    }
  }

  switch (active_state) {
    case ACTIVITY_MOVING:
      tx_interval_s = stationary_tx_interval_s;
      break;
    case ACTIVITY_REST:
      tx_interval_s = rest_tx_interval_s;
      break;
    case ACTIVITY_GPS_LOST:
      tx_interval_s = gps_lost_ping_s;
      break;
    case ACTIVITY_SLEEP:
      tx_interval_s = sleep_tx_interval_s;
      break;
    default:
      // ???
      tx_interval_s = stationary_tx_interval_s;
      break;
  }

  // Has the screen been on for longer than idle time?
  if (now - screen_last_active_ms > screen_idle_off_s * 1000) {
    if (is_screen_on && !screen_stay_on) {
      is_screen_on = false;
      screen_off();
    }
  } else {  // Else we had some recent activity.  Turn on?
    if (!is_screen_on && !screen_stay_off) {
      is_screen_on = true;
      screen_on();
    }
  }
}

/** I must know what that interrupt was for! */
const char *find_irq_name(void) {
  const char *irq_name = "MysteryIRQ";
  return irq_name;
}

struct menu_entry {
  const char *name;
  void (*func)(void);
};

void menu_send_now(void) {
  justSendNow = true;
}

void menu_power_off(void) {
  screen_print("\nPOWER OFF...\n");
  delay(5);  // Give some time to read the screen
  clean_shutdown();
}

void menu_flush_prefs(void) {
  screen_print("\nFlushing Prefs!\n");
  mapper_erase_prefs();
  lorawan_erase_prefs();
  delay(1000);  // Give some time to read the screen
  ESP.restart();
}

void menu_distance_plus(void) {
  min_dist_moved += 5;
}

void menu_distance_minus(void) {
  min_dist_moved -= 5;
  if (min_dist_moved < 10)
    min_dist_moved = 10;
}

void menu_time_plus(void) {
  stationary_tx_interval_s += 10;
}

void menu_time_minus(void) {
  stationary_tx_interval_s -= 10;
  if (stationary_tx_interval_s < 10)
    stationary_tx_interval_s = 10;
}

void menu_gps_passthrough(void) {
  if (PMU) {
    PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
    if (PMU->getChipModel() == XPOWERS_AXP192) {
      PMU->disablePowerOutput(XPOWERS_LDO2);
    } else if (PMU->getChipModel() == XPOWERS_AXP2101) {
      PMU->disablePowerOutput(XPOWERS_ALDO2);
    }
  }
  // axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  // Kill LORA radio
  gps_passthrough();
  // Does not return.
}

void menu_deadzone_here(void) {
  if (tGPS.location.isValid()) {
    deadzone_lat = tGPS.location.lat();
    deadzone_lon = tGPS.location.lng();
    deadzone_radius_m = DEADZONE_RADIUS_M;
  }
}

void menu_no_deadzone(void) {
  deadzone_radius_m = 0.0;
}

void menu_stay_on(void) {
  screen_stay_on = !screen_stay_on;
}

void menu_gps_reset(void) {
  gps_full_reset();
}

/*dr_t sf_list[] = {DR_SF7, DR_SF8, DR_SF9, DR_SF10};
#define SF_ENTRIES (sizeof(sf_list) / sizeof(sf_list[0]))
uint8_t sf_index = 0;

void menu_change_sf(void)
{
    sf_index++;
    if (sf_index >= SF_ENTRIES)
        sf_index = 0;

    //lorawan_sf = sf_list[sf_index];
    //ttn_set_sf(lorawan_sf);
    //ttn_get_sf_name(sf_name, sizeof(sf_name));
    Serial.printf("New SF: %s\n", sf_name);
}
*/
struct menu_entry menu[] = {
    {"Send Now", menu_send_now},
    {"Power Off", menu_power_off},
    {"Distance +", menu_distance_plus},
    {"Distance -", menu_distance_minus},
    {"Time +", menu_time_plus},
    {"Time -", menu_time_minus},
    //    {    "Change SF",       menu_change_sf},
    {"Full Reset", menu_flush_prefs},
    {"USB GPS", menu_gps_passthrough},
    {"Deadzone Here", menu_deadzone_here},
    {"No Deadzone", menu_no_deadzone},
    {"Stay On", menu_stay_on},
    {"GPS Reset", menu_gps_reset},
    //    {   "Experiment",      menu_experiment},
};
#define MENU_ENTRIES (sizeof(menu) / sizeof(menu[0]))

const char *menu_prev;
const char *menu_cur;
const char *menu_next;
boolean is_highlighted = false;
int menu_entry = 0;
static uint32_t menu_idle_start = 0;  // what tick should we call this press long enough

void menu_press(void) {
  if (in_menu) {
    menu_entry = (menu_entry + 1) % MENU_ENTRIES;
  } else {
    in_menu = true;
  }

  menu_prev = menu[(menu_entry - 1) % MENU_ENTRIES].name;
  menu_cur = menu[menu_entry].name;
  menu_next = menu[(menu_entry + 1) % MENU_ENTRIES].name;

  menu_idle_start = millis();
}

void menu_selected(void) {
  menu_idle_start = millis();
  menu[menu_entry].func();
}

void update_screen(void) {
  screen_header(tx_interval_s, min_dist_moved, sf_name, in_deadzone, screen_stay_on, never_rest);
  screen_body(in_menu, menu_prev, menu_cur, menu_next, is_highlighted);
}

void loop() {
  static uint32_t last_fix_count = 0;
  static boolean booted = true;
  uint32_t now_fix_count;
  uint32_t now = millis();

  gps_loop(0 /* active_state == ACTIVITY_WOKE */);  // Update GPS
  now_fix_count = tGPS.sentencesWithFix();          // Did we get a new fix?
  if (now_fix_count != last_fix_count) {
    last_fix_count = now_fix_count;
    last_fix_time = now;  // Note the time of most recent fix
  }

  // ttn_loop();

  // menu timeout
  if (in_menu && now - menu_idle_start > (screen_menu_timeout_s) * 1000)
    in_menu = false;

  // update only every 250ms, or all the time when the in_menu is set
  if (in_menu || (now - last_display_ms) > 250) {
    update_screen();
    last_display_ms = now;
  }

  // If any interrupts on PMIC, report the name
  // PEK button handler
  if (pmu_found && pmu_irq) {
    const char *irq_name;
    pmu_irq = false;
    // uint32_t status = PMU->getIrqStatus();
    PMU->getIrqStatus();

    // Check for USB power events first
    if (PMU->isVbusInsertIrq()) {
        have_usb_power = true;
        Serial.println("USB power connected.");
        screen_print("\nUSB ON");
    } else if (PMU->isVbusRemoveIrq()) {
        have_usb_power = false;
        Serial.println("USB power disconnected.");
        screen_print("\nUSB OFF");
    } else if (PMU->isBatChargeStartIrq()) {
        Serial.println("Battery charge start.");
        screen_print("\nCharge ON");
    } else if (PMU->isBatChargeDoneIrq()) {
        Serial.println("Battery charge done.");
        screen_print("\nCharge DONE");
    } else if (PMU->isPekeyShortPressIrq()) {
      menu_press();
    } else if (PMU->isPekeyLongPressIrq()) {  // want to turn OFF
      menu_power_off();
    } else {
      irq_name = find_irq_name();
      snprintf(buffer, sizeof(buffer), "\n* %s  ", irq_name);
      screen_print(buffer);
    }

    // Clear PMU Interrupt Status Register
    PMU->clearIrqStatus();
    screen_last_active_ms = now;
  }

  // Middle Button handler
  static uint32_t pressTime = 0;
  if (!digitalRead(MIDDLE_BUTTON_PIN)) {
    // Pressure is on
    if (!pressTime) {  // just started a new press
      pressTime = now;
      screen_last_active_ms = now;
      is_highlighted = true;
    }
  } else if (pressTime) {
    // we just did a release
    if (in_menu)
      menu_selected();
    else {
      screen_print("\nSend now!");
      justSendNow = true;
    }
    is_highlighted = false;

    if (now - pressTime > 1000) {
      // Was a long press
    } else {
      // Was a short press
    }
    pressTime = 0;  // Released
    screen_last_active_ms = now;
  }

  update_activity();

  if (booted) {
    // status_uplink(STATUS_BOOT, 0);
    booted = 0;
  }

  if (mapper_uplink() == MAPPER_UPLINK_SUCCESS) {
    // Good send, light Blue LED
    if (pmu_found)
      PMU->setChargingLedMode(XPOWERS_CHG_LED_ON);
  } else {
    // Nothing sent.
    // Do NOT delay() here.. the LoRa receiver and join housekeeping also needs to run!
  }
}
