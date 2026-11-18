#pragma once

#include <stdint.h>

struct LoraCreds {
  uint64_t joinEUI;
  uint64_t devEUI;
  uint8_t appKey[16];
  uint8_t nwkKey[16];
  bool useNwkKey;  // false for LoRaWAN 1.0.x
};

// Overrides fields in c with any values stored in NVS.
void creds_load(LoraCreds &c);
void creds_print(const LoraCreds &c, bool include_keys);
// Non-blocking; reads USB serial and handles configuration commands.
void serial_config_poll(const LoraCreds &active);
