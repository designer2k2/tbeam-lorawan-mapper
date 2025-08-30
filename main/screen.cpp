/**
 *
 * SSD1306 - Screen module
 *
 * Copyright (C) 2025 designer2k2 Stephan M.
 * Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>
 *
 * Based on the work from Xose Pérez
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

// --- Screenshot Helper Classes ---
// These simple subclasses expose the protected 'buffer' from the base library
// so we can access it for screen capture functionality without modifying the library.

class ScreenCaptureSSD1306 : public SSD1306Wire {
public:
  ScreenCaptureSSD1306(uint8_t addr, uint8_t sda, uint8_t scl) : SSD1306Wire(addr, sda, scl) {}
  uint8_t* getBuffer() {
    return this->buffer;
  }
};

class ScreenCaptureSH1106 : public SH1106Wire {
public:
  ScreenCaptureSH1106(uint8_t addr, uint8_t sda, uint8_t scl) : SH1106Wire(addr, sda, scl) {}
  uint8_t* getBuffer() {
    return this->buffer;
  }
};

#define SCREEN_HEADER_HEIGHT 23
const uint16_t logBufferLineLen = 30;
const uint8_t logBufferMaxLines = 4;
const uint16_t LOG_BUFFER_SIZE = 200;
char logBuffer[LOG_BUFFER_SIZE];
uint16_t logHead = 0;
uint16_t logTail = 0;
uint16_t lineStartIndices[logBufferMaxLines];
uint8_t lineStartIndex = 0; // The 'head' for the indices array
uint8_t lineCount = 0;      // How many lines are currently in the buffer

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
    // Ignore carriage returns
    if (c < 32 && c != '\n') return 1; // Ignore non-printable characters except newline

    // --- Part 1: Manage the main logBuffer ---
    logBuffer[logHead] = c;
    logHead = (logHead + 1) % LOG_BUFFER_SIZE;

    // If the buffer is full, advance the tail
    if (logHead == logTail) {
        // Check if the character we are about to overwrite was a newline.
        // If so, we are losing a line and must decrease our line count.
        if (logBuffer[logTail] == '\n' && lineCount > 0) {
            lineCount--;
        }
        logTail = (logTail + 1) % LOG_BUFFER_SIZE;
    }

    // --- Part 2: Manage the lineStartIndices array ---
    if (c == '\n') {
        // Store the starting position of the *next* line
        lineStartIndices[lineStartIndex] = logHead;

        // Advance the index for the line starts, wrapping if needed
        lineStartIndex = (lineStartIndex + 1) % logBufferMaxLines;

        // Keep track of how many lines we have, but don't exceed the max
        if (lineCount < logBufferMaxLines) {
            lineCount++;
        }
    }
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

  screen_buffer_write(text);
}

void screen_buffer_print() {
    if (!display) return;

    display->setTextAlignment(TEXT_ALIGN_LEFT);

    uint16_t startIndex;

    if (lineCount < logBufferMaxLines) {
        // If we have fewer lines than the max, start from the very beginning.
        startIndex = logTail;
    } else {
        // Start from the oldest line in lineStartIndices
    startIndex = lineStartIndices[(lineStartIndex - lineCount + logBufferMaxLines) % logBufferMaxLines];
    }

    const uint16_t lineHeight = 10;
    uint8_t linesDrawn = 0;
    char lineBuffer[logBufferLineLen + 1];
    uint8_t linePos = 0;
    uint16_t i = startIndex;

    while (i != logHead) {
        char character = logBuffer[i];

        if (character == '\n' || linePos >= logBufferLineLen) {
          lineBuffer[linePos] = '\0';
          if (linesDrawn < logBufferMaxLines) {
              display->drawString(0, SCREEN_HEADER_HEIGHT + (linesDrawn * lineHeight), lineBuffer);
              linesDrawn++;
          }
          linePos = 0;
        } else {
            lineBuffer[linePos++] = character;
        }
        i = (i + 1) % LOG_BUFFER_SIZE;
    }

    if (linePos > 0) {
        lineBuffer[linePos] = '\0';
        display->drawString(0, SCREEN_HEADER_HEIGHT + (linesDrawn * lineHeight), lineBuffer);
    }
}

void screen_update() {
  if (display)
    display->display();
  // screen_serial_dump_compressed(); // Send screenshot over serial
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

  // Display instance - CHANGED TO USE OUR WRAPPER CLASSES
  if (display_type == E_DISPLAY_SSD1306)
    display = new ScreenCaptureSSD1306(addr, I2C_SDA, I2C_SCL); // Use our class
  else if (display_type == E_DISPLAY_SH1106)
    display = new ScreenCaptureSH1106(addr, I2C_SDA, I2C_SCL); // Use our class
  else
    return;

  display->init();
  display->flipScreenVertically();
  display->setFont(Custom_Font);
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

void screen_header(unsigned int tx_interval_s, float min_dist_moved, char *cached_sf_name, uint8_t tx_power, boolean in_deadzone,
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
    // Voltage and Battery %
    snprintf(buffer, sizeof(buffer), "%d%%, %.2fV  ", PMU->getBatteryPercent(),
             PMU ? PMU->getBattVoltage() / 1000.0 : 0.0);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(0, 2, buffer);
  } else {
    // ID & Time
    if (no_gps) {
      display->setTextAlignment(TEXT_ALIGN_CENTER);
      display->drawString(display->getWidth() / 2, 2, "*** NO GPS ***");

      snprintf(buffer, sizeof(buffer), "(%d)", sats);
      display->setTextAlignment(TEXT_ALIGN_RIGHT);
      display->drawString(display->getWidth(), 2, buffer);

    } else {
      snprintf(buffer, sizeof(buffer), "#%02d:%02d:%02d", tGPS.time.hour(), tGPS.time.minute(), tGPS.time.second());
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

  // Format the SF and Tx Power together (e.g., "SF7/16dB")
  snprintf(buffer, sizeof(buffer), "%s/%ddB", cached_sf_name, tx_power);
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(display->getWidth(), 12, buffer);

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
  // screen_serial_dump_compressed(); // Send screenshot over serial
}

/**
 * @brief Gets the state of a single pixel directly from the display's memory buffer.
 * This version uses a subclassing trick to access the protected buffer.
 * @param x The x-coordinate of the pixel.
 * @param y The y-coordinate of the pixel.
 * @return 1 if the pixel is on (white), 0 if it is off (black).
 */
int getPixelFromBuffer(int16_t x, int16_t y) {
  if (!display) return 0;
  
  uint8_t* buffer = nullptr;

  // Cast the display pointer to our specific subclass to access getBuffer()
  if (display_type == E_DISPLAY_SSD1306) {
    buffer = static_cast<ScreenCaptureSSD1306*>(display)->getBuffer();
  } else if (display_type == E_DISPLAY_SH1106) {
    buffer = static_cast<ScreenCaptureSH1106*>(display)->getBuffer();
  }

  if (!buffer) return 0; // Safety check

  // Check if coordinates are out of bounds
  if (x < 0 || x >= display->getWidth() || y < 0 || y >= display->getHeight()) {
    return 0;
  }

  // The rest of the logic is the same as before
  int byte_index = x + (y / 8) * display->getWidth();
  int bit_index = y % 8;

  if ((buffer[byte_index] >> bit_index) & 1) {
    return 1;
  }
  
  return 0;
}

/**
 * @brief Dumps the current screen buffer to the Serial port as ASCII art.
 * Useful for debugging without having physical access to the screen.
 */
void screen_serial_dump() {
  if (!display) {
    return;
  }

  Serial.println(F("\n--- SCREEN DUMP BEGIN ---"));
  for (int16_t y = 0; y < display->getHeight(); y++) {
    for (int16_t x = 0; x < display->getWidth(); x++) {
      // Use the new helper function to read from the buffer
      Serial.print(getPixelFromBuffer(x, y) ? "#" : ".");
    }
    Serial.println(); // Newline after each row
  }
  Serial.println(F("--- SCREEN DUMP END ---"));
}


/**
 * @brief Dumps the current screen buffer to Serial using Run-Length Encoding (RLE).
 * This is much faster than the uncompressed dump.
 * Format: B<count> W<count> ... (e.g., B128 W15 B1000)
 */
void screen_serial_dump_compressed() {
  if (!display) {
    return;
  }

  Serial.println(F("\n--- RLE DUMP BEGIN ---"));

  // Get the state of the very first pixel to start the first run
  int currentRunState = getPixelFromBuffer(0, 0);
  int runLength = 0;

  for (int16_t y = 0; y < display->getHeight(); y++) {
    for (int16_t x = 0; x < display->getWidth(); x++) {
      // Use the new helper function here as well
      int pixelState = getPixelFromBuffer(x, y);
      if (pixelState == currentRunState) {
        // If the pixel is the same, extend the current run
        runLength++;
      } else {
        // The pixel changed, so the run has ended. Print it.
        Serial.print(currentRunState ? 'W' : 'B');
        Serial.print(runLength);
        Serial.print(' ');
        
        // Start a new run
        currentRunState = pixelState;
        runLength = 1;
      }
    }
  }

  // After the loops, print the very last run
  Serial.print(currentRunState ? 'W' : 'B');
  Serial.print(runLength);
  Serial.println(); // Final newline
  
  Serial.println(F("--- RLE DUMP END ---"));
}