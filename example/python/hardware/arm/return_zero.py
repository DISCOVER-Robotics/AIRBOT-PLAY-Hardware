import airbot_hardware_py as ah
import time
import math


def all_zero(pos):
    """Check if all positions are close to zero (within 1e-3)"""
    return all(abs(p) < 1e-3 for p in pos)


def main():
    executor = ah.create_asio_executor(8)
    io_context = executor.get_io_context()

    arm = ah.Play.create(
        ah.MotorType.OD,
        ah.MotorType.OD,
        ah.MotorType.OD,
        ah.MotorType.DM,
        ah.MotorType.DM,
        ah.MotorType.DM,
        ah.EEFType.G2,
        ah.MotorType.DM,
    )

    if not arm.init(io_context, "can0", 250):
        raise RuntimeError("Failed to initialize arm")

    arm.enable()
    arm.set_param("arm.control_mode", ah.MotorControlMode.PVT)

    now = time.time()
    while not all_zero(arm.state().pos) and time.time() - now <= 10:
        arm.pvt(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # target positions
            [
                math.pi / 10,
                math.pi / 10,
                math.pi / 10,
                math.pi / 10,
                math.pi / 10,
                math.pi / 10,
            ],  # velocities
        )
        time.sleep(0.001)  # 1ms sleep

    arm.disable()
    arm.uninit()


if __name__ == "__main__":
    main()
