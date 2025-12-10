#!/usr/bin/env python3
import serial
import time
import os
from evdev import InputDevice, ecodes
from select import select

from picamera2 import Picamera2
import cv2

# -----------------------------
# 1. Serial to H725
# -----------------------------
ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

def send_frame(fb_int, lr_int):
    """Send RC frame: 0xAA 0x55 fb lr checksum, fb/lr in -100..100."""
    fb_int = max(-100, min(100, int(fb_int)))
    lr_int = max(-100, min(100, int(lr_int)))

    fb = fb_int & 0xFF
    lr = lr_int & 0xFF

    frame = bytearray(5)
    frame[0] = 0xAA
    frame[1] = 0x55
    frame[2] = fb
    frame[3] = lr
    frame[4] = (frame[0] + frame[1] + frame[2] + frame[3]) & 0xFF

    ser.write(frame)


# -----------------------------
# 2. Gamepad setup
# -----------------------------
DEV_PATH = "/dev/input/event2"  # adjust if needed
gamepad = InputDevice(DEV_PATH)
print(f"Using gamepad: device {gamepad.path}, name \"{gamepad.name}\"")

axis = {
    "ABS_X": 0,   # left/right
    "ABS_Y": 0,   # forward/back
}

btn_TL_pressed = False  # hold to enable face-yaw mode

def scale_axis(value):
    """Map raw -32768..32767 to -100..100."""
    return int((value / 32767) * 100) if value is not None else 0


# -----------------------------
# 3. Camera + face detector
# -----------------------------
CASCADE_PATH = "/home/pi/haarcascade_frontalface_default.xml"
if not os.path.exists(CASCADE_PATH):
    raise FileNotFoundError(f"Cascade file not found: {CASCADE_PATH}")

face_cascade = cv2.CascadeClassifier(CASCADE_PATH)
if face_cascade.empty():
    raise RuntimeError("Failed to load Haar cascade!")

picam2 = Picamera2()
cam_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(cam_config)
picam2.start()
time.sleep(2)
print("Camera started (640x480, rotated 90° CCW in software).")

HEADLESS = True  # set False if you want OpenCV window

# Yaw control from face
FACE_YAW_MAX = 40         # max |lr| from face controller
FACE_DEADZONE_PX = 40     # don't move if face close to center
FACE_CMD_HOLD = 0.4       # seconds to HOLD last face command

last_face_lr = 0          # last yaw command from face
last_face_time = 0.0      # when last face command was updated


# -----------------------------
# 4. Main loop: joystick + face
# -----------------------------
print("\n🚀 Live joystick + (BTN_TL → face yaw) -> UART started\n")

last_print = 0

try:
    while True:
        now = time.time()

        # ---- 4.1 Read joystick (non-blocking) ----
        r, _, _ = select([gamepad.fd], [], [], 0)
        if r:
            for event in gamepad.read():
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        axis["ABS_X"] = event.value
                    elif event.code == ecodes.ABS_Y:
                        axis["ABS_Y"] = event.value

                elif event.type == ecodes.EV_KEY:
                    if event.code == ecodes.BTN_TL:
                        # 1 when pressed, 0 when released
                        btn_TL_pressed = (event.value == 1)

        # Convert joystick axes
        lr_joy = -scale_axis(axis["ABS_X"])
        fb_joy = -scale_axis(axis["ABS_Y"])

        # Deadzone
        if abs(lr_joy) < 5:
            lr_joy = 0
        if abs(fb_joy) < 5:
            fb_joy = 0

        # ---- 4.2 Capture camera frame + rotate 90° CCW ----
        frame = picam2.capture_array()  # RGB
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # ---- 4.3 Face detection (only if BTN_TL pressed) ----
        lr_face = 0

        if btn_TL_pressed:
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.3,
                minNeighbors=5,
                minSize=(60, 60),
            )

            if len(faces) > 0:
                # Pick largest face (closest)
                (x, y, w, h) = max(faces, key=lambda f: f[2] * f[3])

                frame_h, frame_w = gray.shape[:2]
                frame_cx = frame_w / 2.0
                face_cx = x + w / 2.0

                error_px = face_cx - frame_cx  # right is positive

                if abs(error_px) < FACE_DEADZONE_PX:
                    lr_face = 0
                else:
                    norm = error_px / (frame_w / 2.0)  # -1..+1
                    # flip sign if turning the wrong way
                    lr_face = -norm * FACE_YAW_MAX

                # ✅ update hold buffer
                last_face_lr = lr_face
                last_face_time = now

                if not HEADLESS:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.line(frame_bgr, (int(frame_cx), 0), (int(frame_cx), frame_h), (255, 0, 0), 1)
                    cv2.imshow("Joystick + Face (rot 90° CCW)", frame_bgr)
                    cv2.waitKey(1)

            else:
                # ❗ No face this frame, but keep old command for a short time
                if now - last_face_time < FACE_CMD_HOLD:
                    lr_face = last_face_lr
                else:
                    lr_face = 0

        else:
            # Button not held → reset hold state slowly
            if now - last_face_time > FACE_CMD_HOLD:
                last_face_lr = 0
            if not HEADLESS:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("Joystick + Face (rot 90° CCW)", frame_bgr)
                cv2.waitKey(1)

        # ---- 4.4 Decide final commands ----
        if btn_TL_pressed:
            # Face controls yaw, joystick keeps forward/back
            fb_cmd = fb_joy
            lr_cmd = lr_face
            mode = "FACE_YAW"
        else:
            # Pure joystick
            fb_cmd = fb_joy
            lr_cmd = lr_joy
            mode = "JOYSTICK"

        # ---- 4.5 Send to H725 ----
        send_frame(fb_cmd, lr_cmd)

        # ---- 4.6 Debug print at ~10 Hz ----
        if now - last_print > 0.1:
            age = now - last_face_time
            print(
                f"[{mode}] FB_joy={fb_joy:4d}  LR_joy={lr_joy:4d}  "
                f"LR_face={int(lr_face):4d}  last_face_age={age:4.2f}s  "
                f"->  FB={int(fb_cmd):4d}  LR={int(lr_cmd):4d}  BTN_TL={btn_TL_pressed}"
            )
            last_print = now

        # 🔁 Faster command rate: ~100 Hz
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    picam2.stop()
    if not HEADLESS:
        cv2.destroyAllWindows()
    gamepad.close()
    ser.close()
