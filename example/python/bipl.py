import airbot_hardware_py as ah
import time


def main():
    executor = ah.AsioExecutor.create(8)
    base_board = ah.GPIO.create(ah.GPIOType.PLAY_BASE_BOARD)
    end_board = ah.GPIO.create(ah.GPIOType.PLAY_END_BOARD)
    base_board.init(executor.get_io_context(), "can0", 250)
    end_board.init(executor.get_io_context(), "can0", 250)
    base_board.add_callback("button", "button", lambda p: print(p))
    end_board.add_callback("button", "button", lambda p: print(p))
    print(base_board.params()["craft_flag"])
    print(base_board.params()["manufacture_flag"])

    base_board.set_param(
        "manufacture_flag", ah.ParamValue(0, 1, ah.ManufactureFlag.UNSET)
    )
    print(base_board.params()["manufacture_flag"])

    base_board.set_param(
        "manufacture_flag", ah.ParamValue(0, 1, ah.ManufactureFlag.TESTED)
    )
    print(base_board.params()["manufacture_flag"])

    base_board.set_param(
        "manufacture_flag", ah.ParamValue(0, 1, ah.ManufactureFlag.PASSED)
    )
    print(base_board.params()["manufacture_flag"])

    base_board.set_param(
        "manufacture_flag", ah.ParamValue(1, 1, ah.ManufactureFlag.PASSED)
    )
    print(base_board.params()["manufacture_flag"])

    base_board.set_param(
        "manufacture_flag", ah.ParamValue(2, 1, ah.ManufactureFlag.PASSED)
    )
    print(base_board.params()["manufacture_flag"])

    base_board.set_param(
        "manufacture_flag", ah.ParamValue(3, 1, ah.ManufactureFlag.PASSED)
    )
    print(base_board.params()["manufacture_flag"])
    light_effects = [
        ah.LightEffect.RED_CONSTANT,
        ah.LightEffect.ORANGE_CONSTANT,
        ah.LightEffect.YELLOW_CONSTANT,
        ah.LightEffect.GREEN_CONSTANT,
        ah.LightEffect.CYAN_CONSTANT,
        ah.LightEffect.BLUE_CONSTANT,
        ah.LightEffect.PURPLE_CONSTANT,
        ah.LightEffect.WHITE_CONSTANT,
        ah.LightEffect.RED_BREATHING,
        ah.LightEffect.ORANGE_BREATHING,
        ah.LightEffect.YELLOW_BREATHING,
        ah.LightEffect.GREEN_BREATHING,
        ah.LightEffect.CYAN_BREATHING,
        ah.LightEffect.BLUE_BREATHING,
        ah.LightEffect.PURPLE_BREATHING,
        ah.LightEffect.WHITE_BREATHING,
        ah.LightEffect.RED_FLASHING,
        ah.LightEffect.ORANGE_FLASHING,
        ah.LightEffect.YELLOW_FLASHING,
        ah.LightEffect.GREEN_FLASHING,
        ah.LightEffect.CYAN_FLASHING,
        ah.LightEffect.BLUE_FLASHING,
        ah.LightEffect.PURPLE_FLASHING,
        ah.LightEffect.WHITE_FLASHING,
        ah.LightEffect.RED_WAVE,
        ah.LightEffect.ORANGE_WAVE,
        ah.LightEffect.YELLOW_WAVE,
        ah.LightEffect.GREEN_WAVE,
        ah.LightEffect.CYAN_WAVE,
        ah.LightEffect.BLUE_WAVE,
        ah.LightEffect.PURPLE_WAVE,
        ah.LightEffect.WHITE_WAVE,
        ah.LightEffect.RAINBOW_WAVE,
    ]
    for light_effect in light_effects:
        base_board.set_param(
            "light_effect", ah.ParamValue(ah.ParamType.UINT8_LE, light_effect.value)
        )
        time.sleep(0.5)
    base_board.set_param(
        "light_effect",
        ah.ParamValue(ah.ParamType.UINT8_LE, ah.LightEffect.WHITE_CONSTANT.value),
    )
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
