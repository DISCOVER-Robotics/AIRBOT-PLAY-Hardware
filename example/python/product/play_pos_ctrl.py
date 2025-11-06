import airbot_hardware_py as ah
import time
import threading
import json
import socket
import math
import argparse
import sys

# UDP server configuration (PlotJuggler's listening address)
UDP_SERVER_IP = "127.0.0.1"
UDP_SERVER_PORT = 9870

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = (UDP_SERVER_IP, UDP_SERVER_PORT)

print(f"Sending data to {UDP_SERVER_IP}:{UDP_SERVER_PORT}")


def check_parameter(arm_cmd, eef_pos):
    MIN_THRESHOLDS = [-3.1416, -2.9671, -0.087266, -3.0107, -1.7628, -3.0107]
    MAX_THRESHOLDS = [2.0944, 0.17453, 3.1416, 3.0107, 1.7628, 3.0107]

    # 检查arm_cmd的每个元素是否在阈值范围内
    for i in range(len(arm_cmd)):
        if arm_cmd[i] < MIN_THRESHOLDS[i] or arm_cmd[i] > MAX_THRESHOLDS[i]:
            return False

    # 检查eef_pos是否在范围内
    if eef_pos < 0.0 or eef_pos > 0.072:
        return False

    return True


def print_joints_thresholds():
    print("关节角输入阈值如下：")
    print("joint1: lower -3.1416, upper 2.0944.")
    print("joint2: lower -2.9671, upper 0.17453.")
    print("joint3: lower -0.087266, upper 3.1416.")
    print("joint4: lower -3.0107, upper 3.0107.")
    print("joint5: lower -1.7628, upper 1.7628.")
    print("joint6: lower -3.0107, upper 3.0107.")
    print("eef g2 joint: lower 0.0, upper 0.072")


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
    parser = argparse.ArgumentParser(description="控制机械臂到指定关节位置")
    parser.add_argument(
        "-j",
        "--joints",
        nargs=6,
        type=float,
        required=True,
        help="6个关节位置值（弧度），例如：0 0 0 0 0 0 0",
    )
    parser.add_argument(
        "-g", "--gripper", type=float, default=0.0, help="夹爪位置值（0~0.07）"
    )
    parser.add_argument(
        "-i", "--can", default="can0", help="can口名称 (如: can0, can1)"
    )

    args = parser.parse_args()

    # 获取解析后的参数值
    joint_positions = args.joints
    gripper_position = args.gripper
    can_interface = args.can

    if not check_parameter(joint_positions, gripper_position):
        print_joints_thresholds()
        sys.exit()

    g2_is_exist = True

    executor = ah.create_asio_executor(8)
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
        k = [6, 6, 6, 6, 6, 6]

    try:
        while True:
            arm.pvt(
                target_position,
                velocity,
                k,
            )
            print(arm.state().pos)
            if arm.state().is_valid:
                if all(
                    abs(arm.state().pos[i] - target_position[i]) < 0.02
                    for i in range(len(target_position))
                ):
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
