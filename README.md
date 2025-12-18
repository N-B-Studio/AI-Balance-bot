# FOC-STM32-Balance-bot
FOC balance bot based on odrive-mini, STM32 and Linux PC
### Build video: https://youtu.be/rl2r_75Lwew
![cover](https://github.com/user-attachments/assets/ca9a1dbb-3895-411c-bd08-cdf5c83a3056)

## Project Overview
This project implements a self-balancing robot with AI capabilities, combining embedded STM32H7 firmware with Linux-based control scripts and face detection capabilities.

## Directory Structure & File Functions

### `/linux/` - Linux PC Control Scripts
Control scripts for the Raspberry Pi that interface with the STM32 microcontroller:

- **`h725_rc_joystick.py`** - Basic gamepad control script
  - Reads gamepad input from `/dev/input/event2`
  - Maps joystick axes to RC control values (-100 to 100)
  - Sends UART frames to STM32H725 via `/dev/serial0` (115200 baud)
  - Frame format: `0xAA 0x55 [FB] [LR] [checksum]` for forward/back and left/right control

- **`h725_rc_joystick_face.py`** - Gamepad control with face detection integration
  - Extends `h725_rc_joystick.py` with camera-based face detection
  - Uses Picamera2 to capture video frames (640x480)
  - Integrates with `face_detect.py` to detect faces using Haar Cascade classifier
  - Enables face-yaw mode when TL button is held (controls yaw based on face position)
  - Combines gamepad input with AI-based visual feedback

- **`face_detect.py`** - Face detection module
  - Uses OpenCV Haar Cascade classifier for real-time face detection
  - Captures frames from Raspberry Pi camera via Picamera2
  - Rotates frames 90° counterclockwise for proper orientation
  - Detects faces with configurable parameters (scaleFactor=1.3, minNeighbors=5, minSize=60x60)
  - Outputs detected face count and coordinates

- **`read_gamepad.py`** - Simple gamepad input reader
  - Basic utility to test gamepad connectivity
  - Reads raw controller events and displays them to console
  - Useful for debugging input device configuration

- **`read_elrs_crsf.py`** - ELRS/CRSF protocol decoder
  - Decodes ELRS (ExpressLRS) RC control signals via CRSF protocol
  - Runs at 420000 baud on `/dev/serial0`
  - Decodes RC_CHANNELS_PACKED frames (16 channels, 11-bit resolution: 0-2047)
  - Provides alternative to gamepad input using RC transmitter

- **`take_photo.py`** - Camera utility script
  - Captures still images from Raspberry Pi camera (640x480 resolution)
  - Saves photos in raw format and rotated versions (90° CW/CCW)
  - Useful for camera calibration and testing

### `/STM32/` - STM32H723 Embedded Firmware
Main firmware for the balance control MCU:

- **`Core/Inc/`** - Header files
  - `main.h` - Main program definitions
  - `BMI088*.h` - 6-axis IMU (accelerometer + gyroscope) driver headers
  - `bsp_fdcan.h`, `fdcan.h` - FDCAN/CAN communication interface
  - `usart.h`, `spi.h`, `dma.h` - Peripheral communication drivers
  - `gpio.h`, `tim.h` - GPIO and timer configurations

- **`Core/Src/`** - Source implementation files
  - `main.c` - Main control loop
  - `BMI088driver.c` - Low-level IMU register read/write
  - `BMI088Middleware.c` - High-level IMU data processing
  - `bsp_fdcan.c` - CAN bus initialization and handling
  - `usart.c`, `spi.c` - Serial and SPI communication
  - Other support files for system initialization

- **`Drivers/`** - STM32 HAL libraries and CMSIS
  - Hardware Abstraction Layer (HAL) for STM32H7xx
  - CMSIS (Cortex Microcontroller Software Interface Standard) headers

### `/3D-Model/` - CAD Files
3D CAD models for the robot mechanical structure and parts

### `/LICENSE` and `/README.md`
Project licensing information and this documentation file

## Hardware Components
- **MCU**: STM32H723 (ARM Cortex-M7)
- **IMU**: BMI088 (6-axis: 3-axis accelerometer + 3-axis gyroscope)
- **Motor Driver**: ODrive Mini (FOC - Field Oriented Control)
- **Controller**: Raspberry Pi or similar Linux board
- **Camera**: Raspberry Pi Camera Module 2
- **Communication**: UART (serial) for command interface, FDCAN for advanced control

## Communication Protocol
The main control protocol between Linux and STM32 is:
```
Frame: [0xAA] [0x55] [Forward/Back] [Left/Right] [Checksum]
- Forward/Back: -100 to 100 (signed 8-bit)
- Left/Right: -100 to 100 (signed 8-bit)
- Checksum: Sum of all bytes modulo 256
Baud Rate: 115200
```

