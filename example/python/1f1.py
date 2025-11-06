import argparse
import time
import threading
import sys
import airbot_hardware_py as ah
import math


def init_motor(io_ctx, can_if, motor_id):
    if motor_id in [1, 2, 3]:
        motor = ah.Motor.create(ah.MotorType.OD, motor_id)
    else:
        motor = ah.Motor.create(ah.MotorType.DM, motor_id)

    if not motor.init(io_ctx, can_if, 250):
        raise RuntimeError(f"Failed to init motor {motor_id} on {can_if}")

    if not motor.enable():
        raise RuntimeError(f"Failed to enable motor {motor_id} on {can_if}")

    # 尝试设置MIT模式，但不要因为失败而终止程序
    # 根据参考代码，这个调用可能总是返回False，但这是正常的
    if not motor.set_param(
        "control_mode", ah.ParamValue(ah.ParamType.UINT32_LE, ah.MotorControlMode.PVT)
    ):
        print(
            f"Warning: set_param for MIT mode returned False on motor {motor_id} on {can_if} (this is often normal)"
        )

    return motor


def synced(pos_arr, tgt_arr):
    return all(abs(pos - tgt) < 0.01 for pos, tgt in zip(pos_arr, tgt_arr))


def sync_loop(master_motors, slave_motors, stop_event):
    synced = False
    threshold = 0.1  # rad

    while not stop_event.is_set():
        all_valid = True
        diffs = []

        for m in master_motors + slave_motors:
            if not m.pvt(ah.MotorCommand(0.0, math.pi, 0.0, 0.0, 0.0, 10.0)):
                print(f"Failed to send MIT command to master motor", file=sys.stderr)
                all_valid = False
                break
            time.sleep(0.004)

        while not all(i.state().is_valid for i in (master_motors + slave_motors)):
            print("Waiting for valid states...")
            time.sleep(0.1)

        sync_state = False

        if not sync_state:
            # 缓慢跟随
            for i, (m, s) in enumerate(zip(master_motors, slave_motors)):
                if not s.pvt(ah.MotorCommand(m.state().pos, 2.0, 0.0, 0.0, 0.0, 2.0)):
                    print(
                        f"Failed to send MIT command to slave motor {i+1}",
                        file=sys.stderr,
                    )
                time.sleep(0.004)
            sync_state = synced(
                [i.state().pos for i in master_motors],
                [i.state().pos for i in slave_motors],
            )
        else:
            # 紧密跟随
            for i, (m, s) in enumerate(zip(master_motors, slave_motors)):
                if not s.pvt(ah.MotorCommand(m.state().pos, 30.0, 0.0, 0.0, 0.0, 5.0)):
                    print(
                        f"Failed to send MIT command to slave motor {i+1}",
                        file=sys.stderr,
                    )
                time.sleep(0.004)

        time.sleep(0.004)


def main():
    parser = argparse.ArgumentParser(
        description="Sync joints from master to slave arm."
    )
    parser.add_argument(
        "--leader",
        type=str,
        required=True,
        help="Master arm CAN interface (e.g., can0)",
    )
    parser.add_argument(
        "--follower",
        type=str,
        required=True,
        help="Slave arm CAN interface (e.g., can1)",
    )
    parser.add_argument(
        "--joints",
        type=int,
        nargs="+",
        default=[1, 2, 3, 4, 5, 6],
        help="Joint IDs to sync (e.g., 1 2 3)",
    )
    args = parser.parse_args()

    print(
        f"Master CAN: {args.leader}, Slave CAN: {args.follower}, Joints: {args.joints}"
    )
    print("Press Ctrl+C to exit.")

    executor = ah.create_asio_executor(8)

    master_motors = []
    slave_motors = []

    try:
        print("Initializing motors...")
        for jid in args.joints:
            master_motors.append(
                init_motor(executor.get_io_context(), args.leader, jid)
            )
            slave_motors.append(
                init_motor(executor.get_io_context(), args.follower, jid)
            )

        print("Motors initialized. Starting sync thread...")

        # 启动同步线程
        stop_event = threading.Event()
        sync_thread = threading.Thread(
            target=sync_loop, args=(master_motors, slave_motors, stop_event)
        )
        sync_thread.start()

        # 等待用户中断
        try:
            while sync_thread.is_alive():
                sync_thread.join(timeout=0.5)
        except KeyboardInterrupt:
            print("\nCtrl+C detected. Shutting down...")

    except Exception as e:
        print(f"Error during initialization: {e}", file=sys.stderr)

    finally:
        print("Cleaning up motors...")
        if "stop_event" in locals():
            stop_event.set()
        if "sync_thread" in locals() and sync_thread.is_alive():
            sync_thread.join()

        for m in master_motors + slave_motors:
            m.disable()

        print("Sync example finished.")


if __name__ == "__main__":
    main()
