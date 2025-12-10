#!/usr/bin/env python3
from picamera2 import Picamera2
import time
from PIL import Image
import numpy as np

SAVE_DIR = "/home/pi/robot/"

picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

time.sleep(2)  # Warm-up

print("Capturing raw image...")
raw = picam2.capture_array()
picam2.stop()

# Convert to PIL
img_raw = Image.fromarray(raw)
img_raw.save(SAVE_DIR + "test_raw.jpg")
print("Saved:", SAVE_DIR + "test_raw.jpg")

# Rotate 90 CW
img_cw = img_raw.rotate(-90, expand=True)
img_cw.save(SAVE_DIR + "test_rot90CW.jpg")
print("Saved:", SAVE_DIR + "test_rot90CW.jpg")

# Rotate 90 CCW
img_ccw = img_raw.rotate(90, expand=True)
img_ccw.save(SAVE_DIR + "test_rot90CCW.jpg")
print("Saved:", SAVE_DIR + "test_rot90CCW.jpg")

# Rotate 180
img_180 = img_raw.rotate(180, expand=True)
img_180.save(SAVE_DIR + "test_rot180.jpg")
print("Saved:", SAVE_DIR + "test_rot180.jpg")

print("\n📸 Done! Check all 4 images and pick the correct orientation.")