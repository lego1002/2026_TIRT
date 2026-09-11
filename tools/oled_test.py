#!/usr/bin/env python3
"""Standalone SPI OLED bring-up test - no ROS. Confirms the panel itself
lights up before wiring it into any robot-status display.

Wiring (BCM numbering): DC=GPIO24 (pin 18), RST=GPIO25 (pin 22),
SCL/SCK=GPIO11 (pin 23), SDA/MOSI=GPIO10 (pin 19), VCC=3.3V (pin 1),
GND (pin 6). The module has no CS pin, so CE0 stays unconnected.

Usage:
    python3 oled_test.py              # SSD1306 controller (most common)
    python3 oled_test.py sh1106       # try this if SSD1306 stays blank

If it still shows nothing: check RST continuity - a floating RST pin
holds the controller in permanent reset and the panel never lights.
"""
import sys

from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.oled.device import ssd1306, sh1106

DC_PIN = 24
RST_PIN = 25

driver = sh1106 if "sh1106" in sys.argv else ssd1306
print(f"using controller: {driver.__name__}, DC=GPIO{DC_PIN}, RST=GPIO{RST_PIN}")

serial = spi(port=0, device=0, gpio_DC=DC_PIN, gpio_RST=RST_PIN)
device = driver(serial, width=128, height=64)
# Leave the image on screen after this process exits, instead of blanking.
device.persist = True

with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white")
    draw.text((14, 20), "TIRT robot", fill="white")
    draw.text((14, 34), "OLED OK", fill="white")

print("drawn - the panel should now show a bordered box with text.")
