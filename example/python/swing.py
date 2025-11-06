import threading
import time
import math

import airbot_hardware_py as ah


def main():
    executor = ah.create_asio_executor(8)
    io_ctx = executor.get_io_context()
    motors = [
        ah.Motor.create(ah.MotorType.OD, 1),
        ah.Motor.create(ah.MotorType.OD, 2),
        ah.Motor.create(ah.MotorType.OD, 3),
        ah.Motor.create(ah.MotorType.ODM, 4),
        ah.Motor.create(ah.MotorType.ODM, 5),
        ah.Motor.create(ah.MotorType.ODM, 6),
    ]
    for idx, m in enumerate(motors):
        m.init(io_ctx, "can0", 1000)
        m.enable()
        if idx < 3:
            pv = ah.ParamValue(ah.ParamType.UINT32_LE, ah.MotorControlMode.MIT.value)
        else:
            pv = ah.ParamValue(ah.ParamType.UINT32_LE, ah.MotorControlMode.MIT.value)
        m.set_param("control_mode", pv)

    def print_states():
        positions = [0.0] * len(motors)
        while True:
            for idx, m in enumerate(motors):
                state = m.state()
                positions[idx] = state.pos
            print("Motor States:", " ".join(f"{p:+.4f}" for p in positions))
            time.sleep(0.01)  # 100 ms

    printer_thread = threading.Thread(target=print_states, daemon=True)
    printer_thread.start()

    start_time = time.time()
    while time.time() - start_time < 120.0:
        t = time.time()
        now_p = math.sin(t * 3.0) * math.pi / 2.0

        # Build a MotorCommand: {pos, vel, eff, mit_kp, mit_kd, current_threshold}
        # pos_cmd = ah.MotorCommand(0.0, 1.0, 0.0, 0.0, 0.0, 10.0)
        # pos_cmd_1 = ah.MotorCommand(now_p, 50.0, 0.0, 0.0, 0.0, 10.0)
        mit_cmd = ah.MotorCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        results = [
            motors[0].mit(mit_cmd),
            motors[1].mit(mit_cmd),
            motors[2].mit(mit_cmd),
            motors[3].mit(mit_cmd),
            motors[4].mit(mit_cmd),
            motors[5].mit(mit_cmd),
        ]
        if not all(results):
            print("Error in sending commands to motors, skipping...")

        time.sleep(0.001)  # 1 ms

    for m in motors:
        m.disable()
        m.uninit()

    io_ctx.stop()


if __name__ == "__main__":
    main()
