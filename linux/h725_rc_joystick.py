import serial
import time
from evdev import InputDevice, ecodes

# -----------------------------
# 1. Setup Serial
# -----------------------------
ser = serial.Serial("/dev/serial0", 115200, timeout=0.1)

# -----------------------------
# 2. Try to find joystick device
# -----------------------------
dev_path = "/dev/input/event2"   # change to event1/event2 if needed
gamepad = InputDevice(dev_path)

print(f"Using gamepad: {gamepad}")

# Store latest joystick values
axis = {
    "ABS_X": 0,   # CH1  left/right
    "ABS_Y": 0,   # CH2  forward/back
}

# -----------------------------
# Helper: map joystick raw to -100..100
# raw range is roughly -32768..32767
# -----------------------------
def scale(value):
    return int((value / 32767) * 100)

# -----------------------------
# Your UART Frame Sender
# -----------------------------
def send_frame(fb_int, lr_int):
    fb_int = max(-100, min(100, fb_int))
    lr_int = max(-100, min(100, lr_int))

    fb = fb_int & 0xFF
    lr = lr_int & 0xFF

    frame = bytearray(5)
    frame[0] = 0xAA
    frame[1] = 0x55
    frame[2] = fb
    frame[3] = lr
    frame[4] = (frame[0] + frame[1] + frame[2] + frame[3]) & 0xFF

    ser.write(frame)


print("\n🚀 Live joystick → UART control started!")
print("Move joystick to send realtime commands.\n")


# -----------------------------
# 3. Main Loop
# -----------------------------
for event in gamepad.read_loop():

    if event.type == ecodes.EV_ABS:
        if event.code == ecodes.ABS_X:
            axis["ABS_X"] = event.value
        elif event.code == ecodes.ABS_Y:
            axis["ABS_Y"] = event.value

        # Convert joystick → RC channels
        lr = -scale(axis["ABS_X"])     # left/right
        fb = -scale(axis["ABS_Y"])     # forward/back

        # Deadzone (optional)
        if abs(lr) < 5: lr = 0
        if abs(fb) < 5: fb = 0

        # Send to H725
        send_frame(fb, lr)

        print(f"FB={fb:4d}   LR={lr:4d}")

        time.sleep(0.02)   # 50 Hz update loop