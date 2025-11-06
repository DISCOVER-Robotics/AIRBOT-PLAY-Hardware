#!/usr/bin/env python3
"""
A simple Python example to initialize 6 EC motors, and use a thread
to repeatedly send ping commands and print the current joint positions.

This script is a Python equivalent of the C++ `free_drive.cpp` example.
"""

import argparse
import threading
import time
import sys
import math
import airbot_hardware_py as ah


def motor_ping_and_print_task(motors, stop_event):
    """
    A task that repeatedly pings motors and prints their positions.
    """
    while not stop_event.is_set():
        print("========== Motor Positions: ===========")
        for i, motor in enumerate(motors):
            # Send a ping command to request the latest state.
            # In the C++ example, this happens in the main thread, but for simplicity
            # in this Python script, we do it in the worker thread.
            # A short sleep helps prevent bus saturation if ping is very fast.
            time.sleep(0.001)
            # if i == 0:
            #     if not motor.pvt(ah.MotorCommand(math.sin(time.time() * 3), 10.0, 0.0, 0.0, 0.0, 10.0)):
            #         print(f"Failed to ping motor {i+1}.", file=sys.stderr)
            #         continue
            # else:
            #     if not motor.pvt(ah.MotorCommand(0.0, 10.0, 0.0, 0.0, 0.0, 10.0)):
            #         print(f"Failed to ping motor {i+1}.", file=sys.stderr)
            #         continue
            if not motor.mit(ah.MotorCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
                print(f"Failed to ping motor {i+1}.", file=sys.stderr)
                continue

            # Get the state (the ping will have updated it) and print position.
            state = motor.state()
            if state.is_valid:
                print(f"Motor {i+1}: {state.pos:.4f} rad")
            else:
                print(f"Motor {i+1}: Received invalid state.")

        # Wait before the next round of prints
        time.sleep(0.01)


def main(can_interface="can0"):
    """
    Main function to set up motors and run the ping/print loop.
    """
    print("=== Motor Ping and Print Example ===")
    print(f"Using CAN interface '{can_interface}'")
    print("Press Ctrl+C to exit.")

    # 1. Create a single-threaded ASIO executor.
    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    # 2. Create and initialize six EC type motors.
    motors = [
        ah.Motor.create(ah.MotorType.OD, 1),
        ah.Motor.create(ah.MotorType.OD, 2),
        ah.Motor.create(ah.MotorType.OD, 3),
        ah.Motor.create(ah.MotorType.DM, 4),
        ah.Motor.create(ah.MotorType.DM, 5),
        ah.Motor.create(ah.MotorType.DM, 6),
    ]
    print("Creating and initializing motors...")
    for motor in motors:
        if not motor.init(io_context, can_interface, 250):
            print(f"Failed to initialize motor. Aborting.", file=sys.stderr)
            # Clean up already initialized motors
            for m in motors:
                m.disable()
            return

    # 3. Enable each motor.
    print("Enabling motors...")
    for i, motor in enumerate(motors):
        if not motor.enable():
            print(f"Failed to enable motor {i+1}. Aborting.", file=sys.stderr)
            # Clean up all motors
            for m in motors:
                m.disable()
            return
        if not motor.set_param(
            "control_mode",
            ah.ParamValue(ah.ParamType.UINT32_LE, ah.MotorControlMode.MIT),
        ):
            print(
                f"Failed to set control mode to MIT for motor {i+1}. Aborting.",
                file=sys.stderr,
            )
            # FIXME: this set_param would always return False, but the motor is in PVT mode
            # # Clean up all motors
            # for m in motors:
            #     m.disable()
            # return

    print("Motors enabled. Starting ping and print thread...")
    # 4. Start a thread to ping and print motor states.
    stop_event = threading.Event()
    worker_thread = threading.Thread(
        target=motor_ping_and_print_task, args=(motors, stop_event)
    )
    worker_thread.start()

    # 5. Wait for user interruption.
    try:
        # Keep the main thread alive, waiting for the worker or a Ctrl+C
        while worker_thread.is_alive():
            worker_thread.join(timeout=0.5)
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down...")
    finally:
        # 6. Clean up resources.
        print("Cleaning up motors...")
        if worker_thread.is_alive():
            stop_event.set()
            worker_thread.join()

        for motor in motors:
            motor.disable()
        print("Motor ping and print example finished.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Ping EC motors and print their positions."
    )
    parser.add_argument(
        "--can-if",
        default="can0",
        help="CAN interface to use (e.g., 'can0', 'can1').",
    )
    args = parser.parse_args()

    main(args.can_if)
