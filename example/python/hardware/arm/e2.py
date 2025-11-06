import airbot_hardware_py as ah
import time


def main():
    executor = ah.create_asio_executor(1)
    io_context = executor.get_io_context()

    eef = ah.EEF1.create(ah.EEFType.E2, ah.MotorType.OD)
    if not eef.init(io_context, "can0", 250):
        raise RuntimeError("The arm initialization failed.")
    now = time.time()
    while time.time() - now < 10:
        eef.pvt(ah.EEFCommand1())
        print(f"{eef.state().pos[0]:+.3f}")
        time.sleep(0.004)
    eef.uninit()


if __name__ == "__main__":
    main()
