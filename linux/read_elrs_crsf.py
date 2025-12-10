#!/usr/bin/env python3
import serial

PORT = "/dev/serial0"   # UART on GPIO14/15
BAUD = 420000           # CRSF default

CRSF_ADDR_MODULE = 0xC8
CRSF_TYPE_RC_CHANNELS_PACKED = 0x16

def decode_crsf_rc_channels(payload: bytes):
    """
    Decode CRSF RC_CHANNELS_PACKED payload (expected 22 bytes)
    into 16 x 11-bit values (0..2047).
    Returns list of 16 ints.
    """
    if len(payload) < 22:
        return None

    channels = []
    bitbuf = 0
    bits_in_buf = 0

    for b in payload[:22]:
        bitbuf |= b << bits_in_buf
        bits_in_buf += 8

        while bits_in_buf >= 11 and len(channels) < 16:
            ch = bitbuf & 0x7FF  # 11 bits
            channels.append(ch)
            bitbuf >>= 11
            bits_in_buf -= 11

    return channels[:16]

def crsf_to_norm(ch_val: int) -> float:
    """
    Very simple normalization:
    CRSF raw ~ 172..1811, mid ~ 992.
    Map roughly to -1.0 .. +1.0.
    """
    # keep it simple and robust:
    return max(-1.0, min(1.0, (ch_val - 992) / 820.0))

def main():
    print(f"Opening {PORT} @ {BAUD} baud for CRSF...")
    ser = serial.Serial(
        PORT,
        BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )

    state = 0
    frame_len = 0
    frame_data = bytearray()

    while True:
        b = ser.read(1)
        if not b:
            continue
        byte = b[0]

        if state == 0:
            # look for CRSF address
            if byte == CRSF_ADDR_MODULE:
                state = 1
        elif state == 1:
            frame_len = byte
            frame_data = bytearray()
            state = 2
        elif state == 2:
            frame_data.append(byte)
            if len(frame_data) >= frame_len:
                # we have full frame: [type][payload...][crc]
                frame_type = frame_data[0]
                payload = frame_data[1:-1]  # ignore CRC for now

                if frame_type == CRSF_TYPE_RC_CHANNELS_PACKED:
                    channels = decode_crsf_rc_channels(payload)
                    if channels:
                        ch8 = channels[:8]

                        # pretty print raw + normalized
                        raw_str = " ".join(f"{v:4d}" for v in ch8)
                        norm_str = " ".join(f"{crsf_to_norm(v):+0.2f}" for v in ch8)
                        print(f"RAW  CH1-8: {raw_str}   |   NORM: {norm_str}")

                # reset state to search next frame
                state = 0

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")