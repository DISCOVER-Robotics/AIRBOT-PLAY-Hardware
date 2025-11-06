import time
import argparse
import airbot_hardware_py as ah


def main():
    parser = argparse.ArgumentParser("airbot_ins_hand_ctrl")
    parser.add_argument(
        "-j",
        "--joints",
        type=int,
        nargs=6,
        required=False,
        default=[500, 500, 500, 500, 500, 500],
        help="Target joint positions for the hand (6 values)\nExample: -j 0 0 0 0 0 0",
    )

    args = parser.parse_args()
    target_pos = args.joints

    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()
    hand = ah.create_hand(1)

    hand.init(io_context, "can0", 250)
    hand.reset()
    hand.reset_error()

    hand_cmd = ah.HandState()
    hand_cmd.forces = [50, 50, 50, 50, 50, 50]
    hand_cmd.velocities = [500, 500, 500, 500, 500, 500]
    hand_cmd.positions = target_pos

    # pvt supports position+velocity+force control mode
    # In pvt mode, position is controlled to reach the target under velocity and force constraints
    hand.pvt(hand_cmd)
    # set pos only supports position control mode
    hand.set_pos(hand_cmd)

    print("Moving hand to target position:", target_pos)

    try:
        while True:
            hand.get_param("ANGLE_ACT")
            state = hand.state()
            current_angle = list(state.positions)
            print("Current hand angles:", current_angle)
            if current_angle == target_pos:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped by user")

    hand.uninit()


if __name__ == "__main__":
    main()
