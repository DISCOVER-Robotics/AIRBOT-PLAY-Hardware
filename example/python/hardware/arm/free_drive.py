import airbot_hardware_py as ah
import time


def main():
    executor = ah.create_asio_executor(8)
    io_context = executor.get_io_context()

    arm = ah.PlayWithEEF.create(
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.MotorType.EC,
        ah.EEFType.E2,
        ah.MotorType.EC,
    )

    if not arm.init(io_context, "can0", 250):
        raise RuntimeError("The arm initialization failed.")
    print(arm.state().is_valid, arm.state().pos)

    arm.enable()
    arm.set_param("arm.control_mode", ah.MotorControlMode.MIT)
    now = time.time()
    while time.time() - now < 10:
        print(arm.state().is_valid, [f"{i:.4f}" for i in arm.state().pos])
        arm.mit(
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
        )
        time.sleep(0.004)

    arm.disable()
    arm.uninit()


if __name__ == "__main__":
    main()
