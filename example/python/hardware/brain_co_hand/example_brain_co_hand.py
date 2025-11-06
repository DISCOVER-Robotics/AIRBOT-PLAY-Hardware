import time
import argparse
import airbot_hardware_py as ah


def main():
    parser = argparse.ArgumentParser("airbot_brain_co_hand_ctrl")
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

    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    hand = ah.create_brain_co_hand(1)

    hand.init(io_context, "can0", ah.BaudRate.BR_115200)

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


if __name__ == "__main__":
    main()
