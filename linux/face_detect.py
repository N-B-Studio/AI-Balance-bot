#!/usr/bin/env python3
from picamera2 import Picamera2
import cv2
import time
import os

CASCADE_PATH = "/home/pi/haarcascade_frontalface_default.xml"

# Load Haar cascade
if not os.path.exists(CASCADE_PATH):
    raise FileNotFoundError(f"Cascade file not found: {CASCADE_PATH}")

face_cascade = cv2.CascadeClassifier(CASCADE_PATH)
if face_cascade.empty():
    raise RuntimeError("Failed to load Haar cascade!")

# Init Pi camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Camera running... Looking for faces.")
time.sleep(2)

HEADLESS = True  # set True if no monitor

try:
    while True:
        # Capture frame
        frame = picam2.capture_array()   # RGB

        # 🔁 Rotate 90° CCW so face is upright
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Detect faces
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.3,
            minNeighbors=5,
            minSize=(60, 60),
        )

        if len(faces) > 0:
            print(f"Faces detected: {len(faces)}")
        else:
            print("No face")

        if not HEADLESS:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            for (x, y, w, h) in faces:
                cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.imshow("Face Detection (rot 90° CCW)", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    picam2.stop()
    cv2.destroyAllWindows()