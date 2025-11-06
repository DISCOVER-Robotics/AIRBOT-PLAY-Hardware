import argparse
import json
import math
import socket
import threading
import airbot_hardware_py as ah
import time
import sys

# UDP server configuration (PlotJuggler's listening address)
UDP_SERVER_IP = "127.0.0.1"
UDP_SERVER_PORT = 9870

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = (UDP_SERVER_IP, UDP_SERVER_PORT)

print(f"Sending data to {UDP_SERVER_IP}:{UDP_SERVER_PORT}")


def send_to_plotjuggler(arm, stop_event):
    while not stop_event.is_set():
        current_time = time.time()
        motor_num = 6
        state_positions = [0.0] * motor_num
        state_velocitys = [0.0] * motor_num
        state_forces = [0.0] * motor_num

        for i in range(motor_num):
            if arm.state().is_valid:
                state_positions[i] = arm.state().pos[i]
                state_velocitys[i] = arm.state().vel[i]
                state_forces[i] = arm.state().eff[i]

        # Create the JSON object
        json_obj = {
            "state/position": state_positions,
            "state/velocity": state_velocitys,
            "state/force": state_forces,
            "timestamp": current_time,
        }

        # Serialize the JSON object to a string and then encode to bytes
        message = json.dumps(json_obj).encode("utf-8")

        try:
            sock.sendto(message, server_address)
        except Exception as e:
            print(f"Failed to send state data: {e}")

        # Wait before the next round of prints
        time.sleep(0.01)


def main():
    # 设置命令行参数解析
    parser = argparse.ArgumentParser(
        description="Play 关节空间 MIT 控制 2 - 1 号关节正弦摆动"
    )
    parser.add_argument("-T", type=float, required=True, help="摆动的时间周期")
    parser.add_argument(
        "-i", "--can", default="can0", help="can口名称 (如: can0, can1)"
    )
    args = parser.parse_args()
    can_interface = args.can
    T = args.T
    if T < 2 or T > 20:
        print(
            "Parameter T settings are unreasonable. The recommended value is between 2 and 20."
        )
        sys.exit()

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

    if not arm.init(io_context, can_interface, 250):
        raise RuntimeError("The arm initialization failed.")
    arm.enable()
    arm.set_param("arm.control_mode", ah.MotorControlMode.PVT)

    stop_event = threading.Event()
    print_thread = threading.Thread(target=send_to_plotjuggler, args=(arm, stop_event))
    print_thread.daemon = True
    print_thread.start()

    while not arm.state().is_valid or sum([abs(i) for i in arm.state().pos]) > 0.01:
        arm.pvt(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [
                math.pi / 10,
                math.pi / 10,
                math.pi / 10,
                math.pi / 10,
                math.pi / 10,
                math.pi / 10,
            ],  # velocities
            [100, 100, 100, 100, 100, 100],
        )
        time.sleep(0.004)
    print("Arrived at zero position")

    arm.set_param("arm.control_mode", ah.MotorControlMode.MIT)

    try:
        now = time.time()
        start_time = now  # 记录循环开始的绝对时间
        while True:
            elapsed_time = time.time() - start_time  # 计算从循环开始已经过去的时间

            # 计算当前周期内的相对时间（0到10秒之间）
            time_in_period = elapsed_time % T
            # 计算角度：确保每个周期内完成一个完整的 2π 弧度
            angle = 2 * math.pi * (time_in_period / T)

            # 正弦值
            sin_value = math.sin(angle)

            arm.mit(
                [sin_value, 0, 0, 0, 0, 0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0, 0, 0, 0, 0, 0],
                [250.0, 250.0, 250.0, 50.0, 50.0, 50.0],
                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            )
            time.sleep(0.004)
    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在停止机械臂并释放资源...")
    finally:
        stop_event.set()
        if print_thread.is_alive():
            print_thread.join()

        arm.disable()
        arm.uninit()


if __name__ == "__main__":
    main()
