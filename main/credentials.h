/* Credentials definition */

#ifndef _CONFIG_H
#define _CONFIG_H

#include <RadioLib.h>

/*
This is where you define the three key values that map your Device to the LoRaWAN Console.
All three values must match between the code and the Console.

If you want to take the random Console values for a new device, and use them here, be sure to select:
   Device EUI: msb
   App Key:    msb
   NwK Key:    msb
in the Console, then click the arrows to expand the values with comma separators, then paste them below.
*/

/*
NwkKey option for LoRaWAN 1.1.x
- For LoRaWAN 1.0.x, comment out the #define line.
- For LoRaWAN 1.1.x, uncomment it and provide your NwkKey.
*/
//#define USE_NWK_KEY

// joinEUI - previous versions of LoRaWAN called this AppEUI
// for development purposes you can use all zeros - see wiki for details
#define RADIOLIB_LORAWAN_JOIN_EUI 0xa09284515663b1a5

// the Device EUI & two keys can be generated on the TTN console
#ifndef RADIOLIB_LORAWAN_DEV_EUI  // Replace with your Device EUI
#define RADIOLIB_LORAWAN_DEV_EUI 0x044e31696f7f04de
#endif
#ifndef RADIOLIB_LORAWAN_APP_KEY  // Replace with your App Key
#define RADIOLIB_LORAWAN_APP_KEY \
  0x4B, 0x8F, 0xA9, 0x31, 0xAB, 0x2C, 0x68, 0x5B, 0x14, 0x3C, 0x49, 0xB0, 0x7B, 0xFD, 0x35, 0xE3
#endif
#ifndef RADIOLIB_LORAWAN_NWK_KEY  // Put your Nwk Key here
#define RADIOLIB_LORAWAN_NWK_KEY \
  0x77, 0xFC, 0x5F, 0x55, 0x5C, 0x5F, 0x8F, 0x74, 0x3A, 0x04, 0x59, 0x07, 0xA8, 0x09, 0xFB, 0x84
#endif

// for the curious, the #ifndef blocks allow for automated testing &/or you can
// put your EUI & keys in to your platformio.ini - see wiki for more tips

// regional choices: EU868, US915, AU915, AS923, IN865, KR920, CN780, CN500
const LoRaWANBand_t Region = EU868;
const uint8_t subBand = 0;  // For US915, change this to 2, otherwise leave on 0

// copy over the EUI's & keys in to the something that will not compile if incorrectly formatted
uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = {RADIOLIB_LORAWAN_APP_KEY};

// Conditionally define nwkKey. If the macro doesn't exist, it becomes a null pointer.
#ifdef USE_NWK_KEY
  uint8_t nwkKey[] = {RADIOLIB_LORAWAN_NWK_KEY};
#else
  uint8_t* nwkKey = NULL;
#endif

// do not modify below easily, switch between radios in the platformio.ini file build_flags section.

#ifdef ARDUINO_TBEAM_USE_RADIO_SX1262
  // SX1262 on Tbeam1.2
  #define SX1262_CS 18
  #define SX1262_DIO1 33  // SX1262 IRQ
  #define SX1262_BUSY 32  // SX1262 BUSY
  #define SX1262_RESET 23

  SX1262 radio = new Module(SX1262_CS, SX1262_DIO1, SX1262_RESET, SX1262_BUSY);
#endif

#ifdef ARDUINO_TBEAM_USE_RADIO_SX1276
  // SX1276 on Tbeam1.2
  #define SX1276_CS 18
  #define SX1276_DIO1 26  // SX1262 IRQ
  #define SX1262_RESET 23

  SX1276 radio = new Module(SX1276_CS, SX1276_DIO1, SX1262_RESET);
#endif

// create the LoRaWAN node
LoRaWANNode node(&radio, &Region, subBand);

// helper function to display any issues
void debug(bool isFail, const __FlashStringHelper* message, int state, bool Freeze) {
  if (isFail) {
    Serial.print(message);
    Serial.print("(");
    Serial.print(state);
    Serial.println(")");
    while (Freeze);
  }
}

#endif