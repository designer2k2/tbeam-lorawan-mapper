/* Credentials definition */

#ifndef _CONFIG_H
#define _CONFIG_H

#include <RadioLib.h>

/*
This is where you define the three key values that map your Device to the Helium Console.
All three values must match between the code and the Console.

There are two general ways to go about this:
1) Let the Console pick random values for one or all of them, and copy them here in the code.
-or-
2) Define them here in the code, and then copy them to the Console to match these values.

When the Mapper boots, it will show all three values in the Monitor console, like this:

DevEUI (msb): AABBCCDDEEFEFF
APPEUI (msb): 6081F9BF908E2EA0
APPKEY (msb): CF4B3E8F8FCB779C8E1CAEE311712AE5

This format is suitable for copying from Terminal/Monitor and pasting directly into the console as-is.

If you want to take the random Console values for a new device, and use them here, be sure to select:
   Device EUI: msb
   App Key:    msb
   NwK Key:    msb
in the Console, then click the arrows to expand the values with comma separators, then paste them below.
*/
// joinEUI - previous versions of LoRaWAN called this AppEUI
// for development purposes you can use all zeros - see wiki for details
#define RADIOLIB_LORAWAN_JOIN_EUI 0x0000000000000000

// the Device EUI & two keys can be generated on the TTN console
#ifndef RADIOLIB_LORAWAN_DEV_EUI  // Replace with your Device EUI
#define RADIOLIB_LORAWAN_DEV_EUI 0x70B3D57ED0066B6E
#endif
#ifndef RADIOLIB_LORAWAN_APP_KEY  // Replace with your App Key
#define RADIOLIB_LORAWAN_APP_KEY \
  0x74, 0x5D, 0x28, 0x7B, 0xEF, 0xFB, 0x51, 0xFF, 0x4A, 0x89, 0xDC, 0xF7, 0x95, 0x3B, 0x16, 0x4D
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
uint8_t nwkKey[] = {RADIOLIB_LORAWAN_NWK_KEY};


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