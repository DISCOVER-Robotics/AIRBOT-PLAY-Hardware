import argparse
import time
import threading
import sys
import airbot_hardware_py as ah
import math


def all_zero(pos):
    """Check if all positions are close to zero (within 1e-3)"""
    return all(abs(p) < 1e-1 for p in pos)


def main():
    parser = argparse.ArgumentParser(description="1f1")
    parser.add_argument(
        "-i_arm", "--can_arm", default="can0", help="can口名称 (如: can0, can1)"
    )
    parser.add_argument(
        "-i_teacharm",
        "--can_teacharm",
        default="can1",
        help="can口名称 (如: can0, can1)",
    )

    args = parser.parse_args()

    can_interface_arm = args.can_arm
    can_interface_teacharm = args.can_teacharm

    executor = ah.create_asio_executor(8)
    io_context = executor.get_io_context()

    arm = ah.PlayWithEEF.create(
        ah.MotorType.OD,
        ah.MotorType.OD,
        ah.MotorType.OD,
        ah.MotorType.DM,
        ah.MotorType.DM,
        ah.MotorType.DM,
        ah.EEFType.G2,
        ah.MotorType.DM,
    )

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

    if not teacharm.init(io_context, can_interface_teacharm, 250):
        raise RuntimeError("The teacharm initialization failed.")
    teacharm.enable()
    teacharm.set_param("arm.control_mode", ah.MotorControlMode.MIT)

    if not arm.init(io_context, can_interface_arm, 250):
        raise RuntimeError("The arm initialization failed.")
    arm.enable()
    arm.set_param("arm.control_mode", ah.MotorControlMode.PVT)
    arm.set_param("eef.control_mode", ah.MotorControlMode.PVT)

    while True:
        arm.pvt(
            [0, 0, 0, 0, 0, 0, 0],
            [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
            [8, 8, 8, 8, 8, 8, 8],
        )
        if arm.state().is_valid:
            if all_zero(arm.state().pos):
                break
        time.sleep(0.004)

    try:
        while True:
            teacharm.mit(
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0],
            )
            velocity = [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 25.0]
            eff = [8, 8, 8, 8, 8, 8, 8]
            pos = teacharm.state().pos
            pos[6] = pos[6] * 1.5
            pos[6] = max(0.0, min(pos[6], 0.072))
            print(pos[6])
            arm.pvt(pos, velocity, eff)
    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在停止机械臂并释放资源...")

    finally:
        arm.disable()
        arm.uninit()
        teacharm.disable()
        teacharm.uninit()


if __name__ == "__main__":
    main()
