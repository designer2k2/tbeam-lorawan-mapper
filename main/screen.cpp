/**
 *
 * SSD1306 - Screen module
 *
 * Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>
 *
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
 *
 */

#include "screen.h"

#include "configuration.h"
// #include "credentials.h"
#include <OLEDDisplay.h>
#include <SH1106Wire.h>
#include <SSD1306Wire.h>
#include <Wire.h>

#include "font.h"
#include "gps.h"
#include "images.h"

#define SCREEN_HEADER_HEIGHT 23
uint16_t logBufferSize = 200;
uint16_t logBufferFilled = 0;
uint16_t logBufferLine = 0;
uint16_t logBufferLineLen = 30;
uint8_t logBufferMaxLines = 3;
char logBuffer[200];

OLEDDisplay *display;
uint8_t _screen_line = SCREEN_HEADER_HEIGHT - 1;

typedef enum DisplayType_T { E_DISPLAY_UNKNOWN, E_DISPLAY_SSD1306, E_DISPLAY_SH1106 } DisplayType_T;

DisplayType_T display_type = E_DISPLAY_UNKNOWN;

void screen_off() {
  if (!display)
    return;

  display->displayOff();
}

void screen_on() {
  if (!display)
    return;

  display->displayOn();
}

void screen_clear() {
  if (!display)
    return;

  display->clear();
}

void screen_print(const char *text, uint8_t x, uint8_t y, uint8_t alignment) {
  // DEBUG_MSG(text);

  if (!display)
    return;

  display->setTextAlignment((OLEDDISPLAY_TEXT_ALIGNMENT)alignment);
  display->drawString(x, y, text);
}

void screen_print(const char *text, uint8_t x, uint8_t y) {
  screen_print(text, x, y, TEXT_ALIGN_LEFT);
}

size_t screen_buffer_write(uint8_t c) {
  // Don't waste space on \r\n line endings, dropping \r
  if (c == 13)
    return 1;

  // convert UTF-8 character to font table index
  c = (DefaultFontTableLookup)(c);
  // drop unknown character
  if (c == 0)
    return 1;

  bool maxLineReached = logBufferLine >= logBufferMaxLines;
  bool bufferFull = logBufferFilled >= logBufferSize;

  // Can we write to the buffer? If not, make space.
  if (bufferFull || maxLineReached) {
    // See if we can chop off the first line
    uint16_t firstLineEnd = 0;
    for (uint16_t i = 0; i < logBufferFilled; i++) {
      if (logBuffer[i] == 10) {
        // Include last char too
        firstLineEnd = i + 1;
        // Calculate the new logBufferFilled value
        logBufferFilled = logBufferFilled - firstLineEnd;
        // Now move other lines to front of the buffer
        memcpy(logBuffer, &logBuffer[firstLineEnd], logBufferFilled);
        // And voila, buffer one line shorter
        logBufferLine--;
        break;
      }
    }
    // In we can't take off first line, we just empty the buffer
    if (!firstLineEnd) {
      logBufferFilled = 0;
      logBufferLine = 0;
    }
  }

  // So now we know for sure we have space in the buffer

  // Find the length of the last line
  uint16_t lastLineLen = 0;
  for (uint16_t i = 0; i < logBufferFilled; i++) {
    lastLineLen++;
    if (logBuffer[i] == 10)
      lastLineLen = 0;
  }
  // if last line is max length, ignore anything but linebreaks
  if (lastLineLen >= logBufferLineLen) {
    if (c != 10)
      return 1;
  }

  // Write to buffer
  logBuffer[logBufferFilled++] = c;
  // Keep track of lines written
  if (c == 10)
    logBufferLine++;

  // We always claim we printed it all
  return 1;
}

size_t screen_buffer_write(const char *str) {
  // interpretation from OLEDDisplay::write()
  if (str == NULL)
    return 0;
  size_t length = strlen(str);
  for (size_t i = 0; i < length; i++) {
    screen_buffer_write(str[i]);
  }
  return length;
}

void screen_print(const char *text) {
  // Serial.printf("Screen: %s\n", text);
  if (!display)
    return;

  // this needs some better handling:
  // if a newline is in the new text and linecount at max then throw away the oldest line
  // if not then just append the text
  // how much can i copy from the lib here? OLEDDisplay::drawLogBuffer()
  // also needs OLEDDisplay::write()
  // start with just showing the last input
  // strcpy(logBuffer, text);
  screen_buffer_write(text);

  /*
  display->print(text);
  if (_screen_line + 8 > display->getHeight()) {
      // scroll
  }
  _screen_line += 8;
  // screen_loop();
  */
}

void screen_buffer_print() {
  // interpretation from OLEDDisplay::drawLogBuffer()
  // this is mostly hardcoded, not nice but getting it done.
  uint16_t lineHeight = 13;

  //

  // Always align left
  display->setTextAlignment(TEXT_ALIGN_LEFT);

  // State values
  uint16_t length = 0;
  uint16_t line = 0;
  uint16_t lastPos = 0;

  // If the lineHeight and the display height are not cleanly divisible, we need
  // to start off the screen when the buffer has logBufferMaxLines so that the
  // first line, and not the last line, drops off.
  uint16_t shiftUp =
      (logBufferLine == logBufferMaxLines) ? (lineHeight - (display->getHeight() % lineHeight)) % lineHeight : 0;

  for (uint16_t i = 0; i < logBufferFilled; i++) {
    length++;
    // Everytime we have a \n print
    if (logBuffer[i] == 10) {
      // Draw string on line `line` from lastPos to length
      // Passing 0 as the lenght because we are in TEXT_ALIGN_LEFT
      char buffer[length + 1];
      memcpy(&buffer, &logBuffer[lastPos], length);
      // Serial.printf("Screen: %s\n", buffer);
      display->drawString(0, SCREEN_HEADER_HEIGHT - shiftUp + (line++) * lineHeight, buffer);
      // Remember last pos
      lastPos = i;
      // Reset length
      length = 0;
    }
  }
  // Draw the remaining string
  if (length > 0) {
    display->drawString(0, SCREEN_HEADER_HEIGHT - shiftUp + line * lineHeight, &logBuffer[lastPos]);
  }
}

void screen_update() {
  if (display)
    display->display();
}

/**
 * The SSD1306 and SH1106 controllers are almost the same, but different.
 * Most importantly here, the SH1106 allows reading from the frame buffer,
 * while the SSD1306 does not.
 * We exploit this by writing two bytes and reading them back.  A mismatch
 * probably means SSD1306.
 * Probably.
 */
DisplayType_T display_get_type(uint8_t id) {
  uint8_t err;
  uint8_t b1, b2;

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(7000000);

  Wire.beginTransmission(id);
  uint8_t a[] = {
      0,     // co=0 DC=0 000000 to start command
      0x00,  // Lower Column Address = 0
      0x10,  // Higher Column Address = 0
      0xB0   // Set Page Address 0
  };
  Wire.write(a, sizeof(a));
  if ((err = Wire.endTransmission(false)) != 0) {
    // Serial.printf("err=%d EndTransmission=%d(%s)",
    //     err,
    //     Wire.lastError(),
    //     Wire.getErrorText(Wire.lastError())
    // );
    return E_DISPLAY_UNKNOWN;
  }

  Wire.beginTransmission(id);
  uint8_t b[] = {0x40, 'M', 'P'};  // co=0 DC=1 & 000000, then two bytes of data
  Wire.write(b, sizeof(b));
  if ((err = Wire.endTransmission(false)) != 0) {
    // Serial.printf("err=%d EndTransmission=%d(%s)",
    //     err,
    //     Wire.lastError(),
    //     Wire.getErrorText(Wire.lastError())
    // );
    return E_DISPLAY_UNKNOWN;
  }

  Wire.beginTransmission(id);
  uint8_t c[] = {0, 0, 0x10};  // Back to Lower & Higher Column address 0
  Wire.write(c, sizeof(c));
  if ((err = Wire.endTransmission(false)) != 0) {
    // Serial.printf("err=%d EndTransmission=%d(%s)",
    //     err,
    //     Wire.lastError(),
    //     Wire.getErrorText(Wire.lastError())
    // );
    return E_DISPLAY_UNKNOWN;
  }

  Wire.beginTransmission(id);
  Wire.write(0x40);  // Data next
  if ((err = Wire.endTransmission(false)) != 0) {
    // Serial.printf("err=%d EndTransmission=%d(%s)",
    //     err,
    //     Wire.lastError(),
    //     Wire.getErrorText(Wire.lastError())
    // );
    return E_DISPLAY_UNKNOWN;
  }
  err = Wire.requestFrom((int)id, (int)3, (int)1);
  if (err != 3) {
    return E_DISPLAY_UNKNOWN;
  }
  Wire.read();  // Must discard 1 byte
  b1 = Wire.read();
  b2 = Wire.read();
  Wire.endTransmission();

  /** If we read back what we wrote, memory is readable: */
  if (b1 == 'M' && b2 == 'P') {
    return E_DISPLAY_SH1106;
  } else {
    return E_DISPLAY_SSD1306;
  }
}

void screen_setup(uint8_t addr) {
  /* Attempt to determine which kind of display we're dealing with */
  if (display_type == E_DISPLAY_UNKNOWN)
    display_type = display_get_type(addr);

  // Display instance
  if (display_type == E_DISPLAY_SSD1306)
    display = new SSD1306Wire(addr, I2C_SDA, I2C_SCL);
  else if (display_type == E_DISPLAY_SH1106)
    display = new SH1106Wire(addr, I2C_SDA, I2C_SCL);
  else
    return;
  display->init();
  display->flipScreenVertically();
  display->setFont(Custom_Font);

  // Scroll buffer
  display->setLogBuffer(4, 30);
}

void screen_end() {
  if (display) {
    screen_off();
    display->end();
    delete display;
  }
}

#include <XPowersLib.h>
extern XPowersLibInterface *PMU;

void screen_header(unsigned int tx_interval_s, float min_dist_moved, char *cached_sf_name, boolean in_deadzone,
                   boolean stay_on, boolean never_rest) {
  if (!display)
    return;

  char buffer[40];
  uint32_t sats = tGPS.satellites.value();
  boolean no_gps = (sats < 3);
  // uint16_t devid_hint = ((devEUI[7] << 4) | (devEUI[6] & 0xF0) >> 4);

  display->clear();

  // Cycle display every 3 seconds
  if (millis() % 6000 < 3000) {
    // Voltage and Current
    snprintf(buffer, sizeof(buffer), "%.2fV  ", PMU ? PMU->getBattVoltage() / 1000.0 : 0.0);

    // display->setTextAlignment(TEXT_ALIGN_CENTER);
    // display->drawString(display->getWidth() / 2, 2, buffer);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(0, 2, buffer);
  } else {
    // ID & Time
    if (no_gps) {
      // snprintf(buffer, sizeof(buffer), "#%03X", devid_hint);
      display->setTextAlignment(TEXT_ALIGN_LEFT);
      display->drawString(0, 2, buffer);

      display->setTextAlignment(TEXT_ALIGN_CENTER);
      display->drawString(display->getWidth() / 2, 2, "*** NO GPS ***");

      snprintf(buffer, sizeof(buffer), "(%d)", sats);
      display->setTextAlignment(TEXT_ALIGN_RIGHT);
      display->drawString(display->getWidth(), 2, buffer);

    } else {
      snprintf(buffer, sizeof(buffer), "#%02d:%02d:%02d",
               // devid_hint,
               tGPS.time.hour(), tGPS.time.minute(), tGPS.time.second());

      display->setTextAlignment(TEXT_ALIGN_LEFT);
      display->drawString(0, 2, buffer);
    }
  }

  // HDOP & Satellite count
  if (!no_gps) {
    snprintf(buffer, sizeof(buffer), "%2.1f   %d", tGPS.hdop.hdop(), sats);
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->drawString(display->getWidth() - SATELLITE_IMAGE_WIDTH - 4, 2, buffer);
    display->drawXbm(display->getWidth() - SATELLITE_IMAGE_WIDTH, 0, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT,
                     SATELLITE_IMAGE);
  }

  // Second status row:
  snprintf(buffer, sizeof(buffer), "%us %.0fm %c%c%c", tx_interval_s, min_dist_moved, in_deadzone ? 'D' : ' ',
           stay_on ? 'S' : ' ', never_rest ? 'N' : ' ');
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0, 12, buffer);

  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(display->getWidth(), 12, cached_sf_name);

  display->drawHorizontalLine(0, SCREEN_HEADER_HEIGHT, display->getWidth());
}

#define MARGIN 15
void screen_body(boolean in_menu, const char *menu_prev, const char *menu_cur, const char *menu_next,
                 boolean highlighted) {
  if (!display) {
    return;
  }

  if (in_menu) {
    char buffer[40];

    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(display->getWidth() / 2, SCREEN_HEADER_HEIGHT + 5, menu_prev);
    display->drawString(display->getWidth() / 2, SCREEN_HEADER_HEIGHT + 28, menu_next);
    if (highlighted)
      display->clear();
    display->drawHorizontalLine(MARGIN, SCREEN_HEADER_HEIGHT + 16, display->getWidth() - MARGIN * 2);
    snprintf(buffer, sizeof(buffer), highlighted ? ">>> %s <<<" : "%s", menu_cur);
    display->drawString(display->getWidth() / 2, SCREEN_HEADER_HEIGHT + 16, buffer);
    display->drawHorizontalLine(MARGIN, SCREEN_HEADER_HEIGHT + 28, display->getWidth() - MARGIN * 2);
    display->drawVerticalLine(MARGIN, SCREEN_HEADER_HEIGHT + 16, 28 - 16);
    display->drawVerticalLine(display->getWidth() - MARGIN, SCREEN_HEADER_HEIGHT + 16, 28 - 16);
  } else {
    screen_buffer_print();
  }
  display->display();
}