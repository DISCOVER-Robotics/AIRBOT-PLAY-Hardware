import time
import argparse
import airbot_hardware_py as ah


def main():
    parser = argparse.ArgumentParser("airbot_ro_hand_ctrl")
    parser.add_argument(
        "-j",
        "--joints",
        type=int,
        nargs=6,
        required=False,
        default=[500, 500, 500, 500, 500, 500],
        help="Target joint positions for the hand (6 values: 0~1000)\n"
        "HandState order: ThumbFlex, ThumbAux, Index, Middle, Ring, Pinky\n"
        "0=open, 1000=closed\n"
        "Example: -j 500 500 500 500 500 500",
    )

    args = parser.parse_args()
    target_pos = args.joints

    # Validate ranges for HandState (0~1000)
    finger_names = ["ThumbFlex", "ThumbAux", "Index", "Middle", "Ring", "Pinky"]

    for i, pos in enumerate(target_pos):
        if not (0 <= pos <= 1000):
            print(
                f"Warning: {finger_names[i]} position {pos} is outside valid range [0, 1000]"
            )
            target_pos[i] = max(0, min(1000, pos))  # Clamp to valid range
            print(f"Clamped {finger_names[i]} to {target_pos[i]}")

    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    hand = ah.create_ro_hand(2)  # Use device ID 2 as determined in testing

    hand.init(io_context, "can0")

    hand_cmd = ah.HandState()
    hand_cmd.positions = target_pos
    # hand_cmd.forces = [0] * 6  # Initialize forces to zero
    # hand_cmd.velocities = [0] * 6  # Initialize velocities to zero

    hand.set_pos(hand_cmd)

    print("Moving hand to target position:", target_pos)

    try:
        while True:
            state = hand.get_cached_state()
            current_pos = list(state.positions)
            print("Current hand positions:", [f"{pos}" for pos in current_pos])

            # Check if close enough to target (within 10 units tolerance for 0~1000 range)
            if all(
                abs(current - target) < 10
                for current, target in zip(current_pos, target_pos)
            ):
                print("Target position reached!")
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped by user")

    hand.uninit()


if __name__ == "__main__":
    main()
