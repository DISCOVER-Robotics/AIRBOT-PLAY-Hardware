import time
import argparse
import airbot_hardware_py as ah


def check_all_elements(values, lower_bound=0, upper_bound=1000):
    # 检查每个元素是否都是整数且在指定范围内
    for value in values:
        if not isinstance(value, int):
            return False
        if not (lower_bound <= value <= upper_bound):
            return False
    return True


def all_zero(pos):
    """Check if all positions are close to zero (within 1e-3)"""
    return all(abs(p) < 1e-3 for p in pos)


def main():
    parser = argparse.ArgumentParser("airbot_brain_co_hand_ctrl")
    parser.add_argument(
        "-i", "--can", default="can0", help="can口名称 (如: can0, can1)"
    )
    parser.add_argument(
        "-j",
        "--joints",
        type=int,
        nargs=6,
        required=False,
        default=[200, 300, 400, 500, 600, 600],
        help="Target joint positions for the hand (6 values)\nExample: -j 0 0 500 0 0 0 (default: 200 300 400 500 600 600)",
    )

    args = parser.parse_args()
    target_pos = args.joints
    can_interface = args.can

    if not check_all_elements(target_pos):
        print("-j parameter input error, 6 parameters need integers between 0 and 1000")
        return

    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    arm = ah.Play.create(
        ah.MotorType.OD,
        ah.MotorType.OD,
        ah.MotorType.OD,
        ah.MotorType.DM,
        ah.MotorType.DM,
        ah.MotorType.DM,
        ah.EEFType.NA,
        ah.MotorType.NA,
    )

    hand = ah.create_brain_co_hand(1)

    if not arm.init(io_context, can_interface, 250):
        raise RuntimeError("The arm initialization failed.")

    hand.init(io_context, can_interface, ah.BaudRate.BR_115200)

    arm.enable()
    arm.set_param("arm.control_mode", ah.MotorControlMode.PVT)

    while True:
        arm.pvt(
            [0, 0, 0, 0, 0, 0],
            [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
            [8, 8, 8, 8, 8, 8],
        )
        if arm.state().is_valid:
            if all_zero(arm.state().pos):
                break
        time.sleep(0.004)

    hand_cmd = ah.HandState()
    hand_cmd.positions = target_pos

    hand.set_pos(hand_cmd)

    print("Moving hand to target position:", target_pos)

    try:
        while True:

            state = hand.get_cached_state()
            current_angle = list(state.positions)
            print("Current hand angles:", current_angle)
            if current_angle == target_pos:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped by user")

    hand.uninit()
    arm.disable()
    arm.uninit()


if __name__ == "__main__":
    main()
