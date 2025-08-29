#
# Screenshot receiver
#
# Copyright (C) 2025 designer2k2 Stephan M.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import serial
import argparse
from PIL import Image
import sys
import os
from datetime import datetime
import re

# --- Constants for different screen sizes ---
# If you use a different display, change the width and height here.
DISPLAY_WIDTH = 128
DISPLAY_HEIGHT = 64

def screenshot_listener(port, baud, base_output_file):
    """
    Listens on a serial port for screen dumps (compressed or uncompressed)
    and saves each as a timestamped PNG image.
    """
    print(f"--- Listening on port {port} at {baud} bps ---")
    try:
        ser = serial.Serial(port, baud, timeout=2)
        print("--- Port opened. Press Ctrl+C to exit. ---")
    except serial.SerialException as e:
        print(f"Error: Could not open port {port}. {e}")
        sys.exit(1)

    try:
        while True:
            print(f"\n--- Waiting for screenshot dump (Format: Auto-Detect) ---")
            line_bytes = ser.readline()
            if not line_bytes:
                continue

            line = line_bytes.decode('utf-8', errors='ignore').strip()

            # --- Auto-detect format ---
            if "--- RLE DUMP BEGIN ---" in line:
                print("--- Compressed (RLE) dump detected. Capturing... ---")
                rle_data_line = ser.readline().decode('utf-8', errors='ignore').strip()
                ser.readline() # Consume the END marker
                print("--- Capture complete. ---")
                process_rle_and_save(rle_data_line, base_output_file)

            elif "--- SCREEN DUMP BEGIN ---" in line:
                print("--- Uncompressed dump detected. Capturing... ---")
                screenshot_lines = []
                while True:
                    line_bytes = ser.readline()
                    if not line_bytes or b"--- SCREEN DUMP END ---" in line_bytes:
                        break
                    screenshot_lines.append(line_bytes.decode('utf-8', errors='ignore').strip())
                print("--- Capture complete. ---")
                process_uncompressed_and_save(screenshot_lines, base_output_file)

    except KeyboardInterrupt:
        print("\n--- Program interrupted by user. Exiting. ---")
    except serial.SerialException as e:
        print(f"\n--- Serial port error: {e}. Exiting. ---")
    finally:
        if ser.is_open:
            ser.close()
            print("--- Serial port closed. ---")

def process_rle_and_save(rle_data, base_output_file):
    """
    Decodes RLE data and saves it as a white-on-black PNG image.
    """
    if not rle_data:
        print("--- RLE data is empty. Cannot create image. ---")
        return

    print(f"--- Creating a {DISPLAY_WIDTH}x{DISPLAY_HEIGHT} image from RLE data... ---")
    img = Image.new('1', (DISPLAY_WIDTH, DISPLAY_HEIGHT), 0) # Black background
    pixels = img.load()
    
    x, y = 0, 0
    runs = re.findall(r'([WB])(\d+)', rle_data)

    for color_char, length_str in runs:
        length = int(length_str)
        pixel_color = 255 if color_char == 'W' else 0 # White or Black

        for _ in range(length):
            if x < DISPLAY_WIDTH and y < DISPLAY_HEIGHT:
                pixels[x, y] = pixel_color
            
            x += 1
            if x >= DISPLAY_WIDTH:
                x = 0
                y += 1
    
    save_image(img, base_output_file)

def process_uncompressed_and_save(lines, base_output_file):
    """
    Processes uncompressed ASCII art and saves it as a PNG image.
    """
    if not lines:
        print("--- No data captured. Cannot create image. ---")
        return

    height = len(lines)
    width = len(lines[0]) if height > 0 else 0
    
    print(f"--- Creating a {width}x{height} image from uncompressed data... ---")
    img = Image.new('1', (width, height), 0) # Black background
    pixels = img.load()

    for y, row_str in enumerate(lines):
        for x, char in enumerate(row_str):
            if char == '#':
                pixels[x, y] = 255 # White pixel

    save_image(img, base_output_file)

def save_image(img, base_output_file):
    """
    Saves a PIL image object with a timestamped filename.
    """
    name, ext = os.path.splitext(base_output_file)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    final_output_file = f"{name}_{timestamp}{ext}"
    try:
        img.save(final_output_file)
        print(f"--- Screenshot successfully saved to '{final_output_file}' ---")
    except IOError as e:
        print(f"Error: Could not save image file. {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Capture screen dumps from serial and save as PNGs. Auto-detects RLE compression.",
    )
    parser.add_argument('-p', '--port', required=True, help="Serial port name (e.g., COM3, /dev/ttyUSB0)")
    parser.add_argument('-b', '--baud', type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument('-o', '--output', default='screenshot.png', help="Base name for output files (default: screenshot.png)")

    args = parser.parse_args()
    screenshot_listener(args.port, args.baud, args.output)