import argparse
import time
import threading
import sys
import airbot_hardware_py as ah
import math


def main():
    parser = argparse.ArgumentParser(description="replay")
    parser.add_argument(
        "-i", "--can", default="can0", help="can口名称 (如: can0, can1)"
    )

    args = parser.parse_args()

    can_interface = args.can

    executor = ah.create_asio_executor(8)
    io_context = executor.get_io_context()

    teacharm = ah.PlayWithEEF.create(
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.EEFType.E2,
        ah.MotorType.EC,
    )

    if not teacharm.init(io_context, can_interface, 250):
        raise RuntimeError("The teacharm initialization failed.")
    teacharm.enable()
    teacharm.set_param("arm.control_mode", ah.MotorControlMode.MIT)

    try:
        while True:
            teacharm.mit(
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
            )
            print("teacharm pose: ", teacharm.state().pos)

    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在停止机械臂并释放资源...")

    finally:
        teacharm.disable()
        teacharm.uninit()


if __name__ == "__main__":
    main()
