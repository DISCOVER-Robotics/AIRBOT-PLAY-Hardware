import argparse
import airbot_hardware_py as ah
import time
import threading
import json
import socket
import math
import sys

# UDP server configuration (PlotJuggler's listening address)
UDP_SERVER_IP = "127.0.0.1"
UDP_SERVER_PORT = 9870

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = (UDP_SERVER_IP, UDP_SERVER_PORT)

print(f"Sending data to {UDP_SERVER_IP}:{UDP_SERVER_PORT}")


def send_to_plotjuggler(eef, stop_event):
    while not stop_event.is_set():
        current_time = time.time()
        motor_num = 1
        state_positions = [0.0] * motor_num

        for i in range(motor_num):
            if eef.state().is_valid:
                state_positions[i] = eef.state().pos[i]

        # Create the JSON object
        json_obj = {
            "state/position": state_positions,
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


def check_gripper_position(gripper_position):
    if gripper_position < 0.0 or gripper_position > 0.072:
        return 0
    else:
        return 1


def main():
    # 设置命令行参数解析
    parser = argparse.ArgumentParser(
        description="单独 G2 夹爪位置控制 1 - 移动到命令行指定位置"
    )
    parser.add_argument(
        "-g", "--gripper", type=float, required=True, help="夹爪位置值（0~0.07）"
    )
    parser.add_argument(
        "-i", "--can", default="can0", help="can口名称 (如: can0, can1)"
    )

    args = parser.parse_args()

    # 获取解析后的参数值
    gripper_position = args.gripper
    can_interface = args.can

    if not check_gripper_position(gripper_position):
        print("加爪位置参数阈值为[0.0, 0.072]")
        sys.exit()

    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    eef = ah.EEF1.create(ah.EEFType.G2, ah.MotorType.DM)
    if not eef.init(io_context, can_interface, 250):
        raise RuntimeError("The EEF initialization failed.")
    eef.enable()
    eef.set_param("control_mode", ah.MotorControlMode.PVT)
    stop_event = threading.Event()
    print_thread = threading.Thread(target=send_to_plotjuggler, args=(eef, stop_event))
    print_thread.daemon = True
    print_thread.start()
    try:
        while True:
            cmd = ah.EEFCommand1()
            cmd.pos = [gripper_position]
            cmd.vel = [10.0]
            cmd.current_threshold = [10.0]
            eef.pvt(cmd)
            if eef.state().is_valid:
                if abs(eef.state().pos[0] - gripper_position) < 0.0001:
                    break
            # print(eef.state().pos)
            time.sleep(0.004)
    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在停止机械臂并释放资源...")
    finally:
        stop_event.set()
        if print_thread.is_alive():
            print_thread.join()

        eef.disable()
        eef.uninit()


if __name__ == "__main__":
    main()
