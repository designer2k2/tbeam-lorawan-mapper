/*

SSD1306 - Screen module

Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "screen.h"

#include <Wire.h>

#include "OLEDDisplay.h"
#include "SSD1306Wire.h"
#include "configuration.h"
#include "credentials.h"
#include "fonts.h"
#include "gps.h"
#include "images.h"

#define SCREEN_HEADER_HEIGHT 24

SSD1306Wire *display;
uint8_t _screen_line = SCREEN_HEADER_HEIGHT - 1;

void screen_show_logo() {
  if (!display) return;

  uint8_t x = (display->getWidth() - TTN_IMAGE_WIDTH) / 2;
  uint8_t y = SCREEN_HEADER_HEIGHT + (display->getHeight() - SCREEN_HEADER_HEIGHT - TTN_IMAGE_HEIGHT) / 2 + 1;
  display->drawXbm(x, y, TTN_IMAGE_WIDTH, TTN_IMAGE_HEIGHT, TTN_IMAGE);
}

void screen_off() {
  if (!display) return;

  display->displayOff();
}

void screen_on() {
  if (!display) return;

  display->displayOn();
}

void screen_clear() {
  if (!display) return;

  display->clear();
}

void screen_print(const char *text, uint8_t x, uint8_t y, uint8_t alignment) {
  DEBUG_MSG(text);

  if (!display) return;

  display->setTextAlignment((OLEDDISPLAY_TEXT_ALIGNMENT)alignment);
  display->drawString(x, y, text);
}

void screen_print(const char *text, uint8_t x, uint8_t y) {
  screen_print(text, x, y, TEXT_ALIGN_LEFT);
}

void screen_print(const char *text) {
  Serial.printf("Screen: %s\n", text);
  if (!display) return;

  display->print(text);
  if (_screen_line + 8 > display->getHeight()) {
    // scroll
  }
  _screen_line += 8;
  // screen_loop();
}

void screen_update() {
  if (display) display->display();
}

void screen_setup() {
  // Display instance
  display = new SSD1306Wire(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);
  display->init();
  display->flipScreenVertically();
  display->setFont(Custom_ArialMT_Plain_10);

  // Scroll buffer
  display->setLogBuffer(4, 30);
}

#include <axp20x.h>
extern AXP20X_Class axp;  // TODO: This is evil

void screen_header(unsigned long int tx_interval_ms, float min_dist_moved, char *cached_sf_name, int sats) {
  if (!display) return;

  char buffer[40];

  // Cycle display every 3 seconds
  if (millis() % 6000 < 3000) {
    // 2 bytes of Device EUI with Voltage and Current
    snprintf(buffer, sizeof(buffer), "#%03X", ((DEVEUI[7] << 4) | (DEVEUI[6] & 0xF0) >> 4));
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(0, 2, buffer);

    snprintf(buffer, sizeof(buffer), "%.2fV %.0fmA", axp.getBattVoltage() / 1000, axp.getBattChargeCurrent() - axp.getBattDischargeCurrent());
  } else {
    // Message count and time
    // snprintf(buffer, sizeof(buffer), "%4d", ttn_get_count() % 10000);
    // display->setTextAlignment(TEXT_ALIGN_LEFT);
    // display->drawString(0, 2, buffer);

    gps_time(buffer, sizeof(buffer));
  }

  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(display->getWidth() / 2, 2, buffer);

  // Satellite count
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(display->getWidth() - SATELLITE_IMAGE_WIDTH - 4, 2, itoa(sats, buffer, 10));
  display->drawXbm(display->getWidth() - SATELLITE_IMAGE_WIDTH, 0, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT, SATELLITE_IMAGE);

  // Second status row:
  snprintf(buffer, sizeof(buffer), "%lus %.0fm", tx_interval_ms / 1000, min_dist_moved);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0, 12, buffer);

  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(display->getWidth(), 12, cached_sf_name);

  display->drawHorizontalLine(0, SCREEN_HEADER_HEIGHT, display->getWidth());
}

void screen_loop(unsigned long int tx_interval_ms, float min_dist_moved, char *cached_sf_name, int sats) {
  if (!display) return;

  display->clear();
  screen_header(tx_interval_ms, min_dist_moved, cached_sf_name, sats);
  display->drawLogBuffer(0, SCREEN_HEADER_HEIGHT);
  display->display();
}