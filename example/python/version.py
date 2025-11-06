import airbot_hardware_py as ah
import sys

executor = ah.create_asio_executor(1)
io_context = executor.get_io_context()

motors = [
    ah.Motor.create(ah.MotorType.OD, 1),
    ah.Motor.create(ah.MotorType.OD, 2),
    ah.Motor.create(ah.MotorType.OD, 3),
    ah.Motor.create(ah.MotorType.DM, 4),
    ah.Motor.create(ah.MotorType.DM, 5),
    ah.Motor.create(ah.MotorType.DM, 6),
]

for motor in motors:
    if not motor.init(io_context, "can0", 250):
        print(f"Failed to initialize motor. Aborting.", file=sys.stderr)
        # Clean up already initialized motors
        for m in motors:
            m.disable()

for motor in motors:
    motor.get_param("fw_version")

import time

time.sleep(2)

for motor in motors:
    print(motor.params()["fw_version"].format())
