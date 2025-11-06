import argparse
import json
import socket
import threading
import airbot_hardware_py as ah
import time
import math


def all_zero(pos):
    """Check if all positions are close to zero (within 1e-3)"""
    return all(abs(p) < 1e-3 for p in pos)


# UDP server configuration (PlotJuggler's listening address)
UDP_SERVER_IP = "127.0.0.1"
UDP_SERVER_PORT = 9870

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = (UDP_SERVER_IP, UDP_SERVER_PORT)

print(f"Sending data to {UDP_SERVER_IP}:{UDP_SERVER_PORT}")


def send_to_plotjuggler(arm, stop_event, g2_is_exist):
    while not stop_event.is_set():
        current_time = time.time()
        motor_num = 6
        if g2_is_exist:
            motor_num = 7

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
        description="Play + G2 关节空间位置控制 4 - 返回零点"
    )
    parser.add_argument(
        "-i", "--can", default="can0", help="can口名称 (如: can0, can1)"
    )

    args = parser.parse_args()

    # 获取解析后的参数值
    joint_positions = [0, 0, 0, 0, 0, 0]
    gripper_position = 0
    can_interface = args.can

    g2_is_exist = True

    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    arm = None

    eef = ah.EEF1.create(ah.EEFType.G2, ah.MotorType.DM)
    if not eef.init(io_context, can_interface, 250):
        g2_is_exist = False
        print("G2 is not exist")
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
    else:
        eef.uninit()
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

    if not arm.init(io_context, can_interface, 250):
        raise RuntimeError("The arm initialization failed.")
    arm.enable()
    arm.set_param("arm.control_mode", ah.MotorControlMode.PVT)

    if g2_is_exist == True:
        arm.set_param("eef.control_mode", ah.ParamValue(ah.ParamType.UINT32_LE, 4))

    stop_event = threading.Event()
    print_thread = threading.Thread(
        target=send_to_plotjuggler, args=(arm, stop_event, g2_is_exist)
    )
    print_thread.daemon = True
    print_thread.start()

    if g2_is_exist == True:
        target_position = joint_positions + [gripper_position]
        velocity = [
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
        ]
        k = [100, 100, 100, 100, 100, 100, 100]
    else:
        target_position = joint_positions
        velocity = [
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
            math.pi / 10,
        ]
        k = [100, 100, 100, 100, 100, 100]

    try:
        now = time.time()
        print(arm.state().pos)
        while time.time() - now < 10:
            arm.pvt(
                target_position,
                velocity,
                k,
            )
            if arm.state().is_valid:
                if all_zero(arm.state().pos):
                    break
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
