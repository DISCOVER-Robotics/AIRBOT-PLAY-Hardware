import argparse
import airbot_hardware_py as ah
import time
import threading
import json
import socket
import math

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
        state_positions = 0.0

        if eef.state().is_valid:
            state_positions = eef.state().pos

        # Create the JSON object
        json_obj = {
            "state/e2_position": state_positions,
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
    parser = argparse.ArgumentParser(description="单独 E2 位置反馈")
    parser.add_argument(
        "-i", "--can", default="can0", help="can口名称 (如: can0, can1)"
    )

    args = parser.parse_args()

    # 获取解析后的参数值
    can_interface = args.can

    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    eef = ah.EEF1.create(ah.EEFType.E2, ah.MotorType.OD)
    if not eef.init(io_context, can_interface, 250):
        raise RuntimeError("The E2 initialization failed.")

    eef.enable()
    # eef.set_param("eef.control_mode", ah.MotorControlMode.MIT)
    # eef.set_param("eef.e2.mode", ah.ParamType.UINT32_LE)
    # eef.set_param("eef.e2.mode", 4)
    param_value = ah.ParamValue(ah.ParamType.UINT32_LE, 4)
    eef.set_param("eef.e2.mode", param_value)

    stop_event = threading.Event()
    print_thread = threading.Thread(target=send_to_plotjuggler, args=(eef, stop_event))
    print_thread.daemon = True
    print_thread.start()

    try:
        while True:
            eef.mit(ah.EEFCommand1())
            if eef.state().is_valid:
                print(f"The parallel width of E2: {eef.state().pos[0]:.3f}")

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
