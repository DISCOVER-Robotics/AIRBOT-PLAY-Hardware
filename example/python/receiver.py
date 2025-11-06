#!/usr/bin/env python3
"""
Python receiver equivalent to receiver.cpp using airbot_hardware Python bindings.

This script receives TOK2Message data over serial communication and displays the parsed data.
"""

import airbot_hardware_py as ah
import struct
import time
import math
from typing import List, Tuple

# Constants
FRAME_SIZE = 32
FREQ = 500
SERIAL_DEVICE = "/dev/ttyUSB0"


class TOK2Message:
    """Python equivalent of the TOK2Message struct from receiver.cpp"""

    def __init__(self):
        self.left_arm = [0.0] * 7  # 7 float values for left arm
        self.right_arm = [0.0] * 7  # 7 float values for right arm
        self.base = 0  # uint32 base value

    @staticmethod
    def map_to(value: float) -> int:
        """Map float value to uint16 range"""
        return int((value + math.pi) / (2 * math.pi) * 65535)

    @staticmethod
    def map_from(value: int) -> float:
        """Map uint16 value back to float range"""
        return (float(value) / 65535.0) * 2 * math.pi - math.pi

    def serialize(self) -> List[int]:
        """Serialize the message to a 32-byte frame"""
        # Map float values to uint16
        mapped_left = [self.map_to(val) for val in self.left_arm]
        mapped_right = [self.map_to(val) for val in self.right_arm]

        # Pack into bytes: 14 uint16 values + 1 uint32 = 30 bytes (with 2 bytes padding)
        frame_data = []

        # Pack left arm (7 uint16 values)
        for val in mapped_left:
            frame_data.extend(struct.pack("<H", val))

        # Pack right arm (7 uint16 values)
        for val in mapped_right:
            frame_data.extend(struct.pack("<H", val))

        # Pack base (1 uint32 value)
        frame_data.extend(struct.pack("<I", self.base))

        # Convert to list of integers
        return list(frame_data)

    @classmethod
    def deserialize(cls, frame: ah.SerialFrame32) -> "TOK2Message":
        """Deserialize a 32-byte frame back to TOK2Message"""
        if len(frame) < 32:
            raise RuntimeError("Frame size is too small for TOK2Message structure")

        # Extract bytes from frame
        frame_bytes = bytes([frame[i] for i in range(32)])

        # Unpack the data
        msg = cls()

        # Unpack 14 uint16 values (left_arm + right_arm) + 1 uint32 (base)
        # 14 uint16 values (28 bytes) + 1 uint32 (4 bytes) = 32 bytes total
        unpacked = struct.unpack("<14HI", frame_bytes)

        # Extract left arm (first 7 uint16 values)
        for i in range(7):
            msg.left_arm[i] = cls.map_from(unpacked[i])

        # Extract right arm (next 7 uint16 values)
        for i in range(7):
            msg.right_arm[i] = cls.map_from(unpacked[7 + i])

        # Extract base (last uint32 value)
        msg.base = unpacked[14]

        return msg

    def format(self) -> str:
        """Format the message for display"""
        left_str = " ".join([f"{val:.2f}" for val in self.left_arm])
        right_str = " ".join([f"{val:.2f}" for val in self.right_arm])
        return f"Left Arm: {left_str}  Right Arm: {right_str}  Base: {self.base}"


def format_frame(frame: ah.SerialFrame32, prefix: str = "") -> str:
    """Format a frame by parsing TOK2Message data (matches C++ receiver behavior)"""
    try:
        # Parse the frame to TOK2Message and format the values
        message = TOK2Message.deserialize(frame)
        return message.format()
    except Exception as e:
        # If parsing fails, return error info
        return f"Error parsing frame: {e}"


def main():
    """Main receiver function"""
    print("TOK2 Python Receiver")
    print("===================")

    # Create executor with 1 thread
    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    # Create serial communication handler for 32-byte frames
    serial_handler = ah.SerialCommHandler32.create(SERIAL_DEVICE, FREQ, io_context)

    if serial_handler is None:
        print(f"Failed to create serial communication handler for {SERIAL_DEVICE}")
        return 1

    # Add read callback for incoming frames
    def on_frame_received(frame: ah.SerialFrame32):
        try:
            message = TOK2Message.deserialize(frame)
            print(message.format())
        except Exception as e:
            print(f"Error deserializing frame: {e}")
            # Print raw frame for debugging
            print(format_frame(frame, "RAW"))

    serial_handler.add_read_callback("tok2_receiver", on_frame_received)

    # Start the communication handler
    if not serial_handler.start():
        print("Failed to start serial communication handler")
        return 1

    print(f"Listening on {SERIAL_DEVICE} at {FREQ} Hz...")
    print("Press Ctrl+C to stop")

    try:
        # Run for 2 minutes or until interrupted
        start_time = time.time()
        while time.time() - start_time < 120:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping receiver...")

    # Stop the communication handler (treat as synchronous due to binding limitations)
    print("Stopping serial communication handler...")
    serial_handler.stop()
    print("Serial communication handler stopped")

    # Give a moment for cleanup
    time.sleep(0.1)

    return 0


if __name__ == "__main__":
    exit(main())
