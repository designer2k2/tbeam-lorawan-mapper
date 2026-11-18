#include "serial_config.h"

#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

#define CREDS_NS "creds"
#define LINE_LEN 128

static LoraCreds pending;
static bool have_pending = false;

static int hex_val(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

// Accepts "A1B2..", "A1:B2:..", "0xA1, 0xB2, .." and "{ 0xA1, .. }" (TTN console msb format).
static bool parse_hex(const char *s, uint8_t *out, size_t len) {
  uint8_t tmp[16];
  size_t digits = 0;
  bool prev_hex = false;
  if (!s || len > sizeof(tmp)) return false;
  for (; *s; s++) {
    if (!prev_hex && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
      s++;
      continue;
    }
    int v = hex_val(*s);
    if (v < 0) {
      if (!strchr(" \t,:-{}", *s)) return false;
      prev_hex = false;
      continue;
    }
    prev_hex = true;
    if (digits >= len * 2) return false;
    if (digits % 2 == 0)
      tmp[digits / 2] = v << 4;
    else
      tmp[digits / 2] |= v;
    digits++;
  }
  if (digits != len * 2) return false;
  memcpy(out, tmp, len);
  return true;
}

static bool parse_eui(const char *s, uint64_t &out) {
  uint8_t b[8];
  if (!parse_hex(s, b, sizeof(b))) return false;
  out = 0;
  for (int i = 0; i < 8; i++) out = (out << 8) | b[i];
  return true;
}

static void print_key(const char *label, const uint8_t *key) {
  Serial.print(label);
  for (int i = 0; i < 16; i++) Serial.printf("%02X", key[i]);
  Serial.println();
}

void creds_load(LoraCreds &c) {
  Preferences p;
  if (!p.begin(CREDS_NS, true)) return;  // Namespace absent: keep build defaults
  if (p.isKey("joineui")) c.joinEUI = p.getULong64("joineui");
  if (p.isKey("deveui")) c.devEUI = p.getULong64("deveui");
  if (p.isKey("appkey")) p.getBytes("appkey", c.appKey, sizeof(c.appKey));
  if (p.isKey("use_nwk")) {
    c.useNwkKey = p.getBool("use_nwk");
    if (c.useNwkKey) p.getBytes("nwkkey", c.nwkKey, sizeof(c.nwkKey));
  }
  p.end();
}

void creds_print(const LoraCreds &c, bool include_keys) {
  Preferences p;
  bool stored = p.begin(CREDS_NS, true) && p.isKey("deveui");
  p.end();
  Serial.printf("[Creds] Source:  %s\n", stored ? "NVS" : "build defaults (credentials.h)");
  Serial.printf("[Creds] DevEUI:  %016llX\n", (unsigned long long)c.devEUI);
  Serial.printf("[Creds] JoinEUI: %016llX\n", (unsigned long long)c.joinEUI);
  if (include_keys) {
    print_key("[Creds] AppKey:  ", c.appKey);
    if (c.useNwkKey)
      print_key("[Creds] NwkKey:  ", c.nwkKey);
    else
      Serial.println("[Creds] NwkKey:  (not used, LoRaWAN 1.0.x)");
  }
}

static void print_help() {
  Serial.println(F("Commands:\n"
                   "  show                    print credentials\n"
                   "  set deveui <16 hex>     stage DevEUI (msb)\n"
                   "  set joineui <16 hex>    stage JoinEUI/AppEUI (msb)\n"
                   "  set appkey <32 hex>     stage AppKey (msb)\n"
                   "  set nwkkey <32 hex>     stage NwkKey (LoRaWAN 1.1)\n"
                   "  clear nwkkey            stage: do not use NwkKey (LoRaWAN 1.0.x)\n"
                   "  save                    store staged values, forget session, reboot\n"
                   "  discard                 drop staged values\n"
                   "  clear keys              erase stored keys (back to build defaults), reboot\n"
                   "  reboot"));
}

// The stored session/nonces belong to the old keys and must not be reused.
static void forget_session() {
  Preferences p;
  if (p.begin("lora", false)) {
    p.remove("nonces");
    p.remove("session");
    p.end();
  }
}

static void save_and_reboot() {
  Preferences p;
  if (!p.begin(CREDS_NS, false)) {
    Serial.println("Error: could not open NVS");
    return;
  }
  p.putULong64("joineui", pending.joinEUI);
  p.putULong64("deveui", pending.devEUI);
  p.putBytes("appkey", pending.appKey, sizeof(pending.appKey));
  p.putBytes("nwkkey", pending.nwkKey, sizeof(pending.nwkKey));
  p.putBool("use_nwk", pending.useNwkKey);
  p.end();
  forget_session();
  Serial.println("Saved. Rebooting to join with new credentials...");
  Serial.flush();
  ESP.restart();
}

static void handle_set(const char *field, const char *value) {
  bool ok;
  if (!field || !value) {
    Serial.println("Usage: set <deveui|joineui|appkey|nwkkey> <hex>");
    return;
  }
  if (!strcasecmp(field, "deveui")) {
    ok = parse_eui(value, pending.devEUI);
  } else if (!strcasecmp(field, "joineui") || !strcasecmp(field, "appeui")) {
    ok = parse_eui(value, pending.joinEUI);
  } else if (!strcasecmp(field, "appkey")) {
    ok = parse_hex(value, pending.appKey, sizeof(pending.appKey));
  } else if (!strcasecmp(field, "nwkkey")) {
    ok = parse_hex(value, pending.nwkKey, sizeof(pending.nwkKey));
    if (ok) pending.useNwkKey = true;
  } else {
    Serial.printf("Unknown field '%s'\n", field);
    return;
  }
  if (!ok) {
    Serial.println("Invalid hex value (EUIs need 16 hex digits, keys 32)");
    return;
  }
  have_pending = true;
  Serial.printf("Staged %s. Type 'save' to apply.\n", field);
}

static void handle_line(char *line, const LoraCreds &active) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!have_pending) pending = active;

  if (!strcasecmp(cmd, "help") || !strcmp(cmd, "?")) {
    print_help();
  } else if (!strcasecmp(cmd, "show")) {
    creds_print(active, true);
    if (have_pending) {
      Serial.println("Staged (unsaved):");
      creds_print(pending, true);
    }
  } else if (!strcasecmp(cmd, "set")) {
    char *field = strtok(NULL, " \t");
    handle_set(field, strtok(NULL, ""));
  } else if (!strcasecmp(cmd, "clear")) {
    char *what = strtok(NULL, " \t");
    if (what && !strcasecmp(what, "nwkkey")) {
      pending.useNwkKey = false;
      have_pending = true;
      Serial.println("Staged: NwkKey off. Type 'save' to apply.");
    } else if (what && !strcasecmp(what, "keys")) {
      Preferences p;
      if (p.begin(CREDS_NS, false)) {
        p.clear();
        p.end();
      }
      forget_session();
      Serial.println("Stored keys erased. Rebooting...");
      Serial.flush();
      ESP.restart();
    } else {
      Serial.println("Usage: clear <nwkkey|keys>");
    }
  } else if (!strcasecmp(cmd, "save")) {
    if (have_pending)
      save_and_reboot();
    else
      Serial.println("Nothing staged.");
  } else if (!strcasecmp(cmd, "discard")) {
    have_pending = false;
    Serial.println("Staged values dropped.");
  } else if (!strcasecmp(cmd, "reboot")) {
    ESP.restart();
  } else {
    Serial.printf("Unknown command '%s' (try 'help')\n", cmd);
  }
}

void serial_config_poll(const LoraCreds &active) {
  static char line[LINE_LEN];
  static size_t len = 0;
  static bool overflow = false;

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (overflow) {
        Serial.println("Line too long, ignored.");
      } else if (len > 0) {
        line[len] = '\0';
        Serial.printf("> %s\n", line);
        handle_line(line, active);
      }
      len = 0;
      overflow = false;
    } else if (len < LINE_LEN - 1) {
      line[len++] = ch;
    } else {
      overflow = true;
    }
  }
}
