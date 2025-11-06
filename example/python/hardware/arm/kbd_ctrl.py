import airbot_hardware_py as ah
import time
import threading
import math

INSTRUCTIONS = """Keyboard control examples (Low-level SDK, joint-space)
Key 1, Key 2: Motor #1
Key 3, Key 4: Motor #2
Key 5, Key 6: Motor #3
Key 7, Key 8: Motor #4
Key 9, Key 0: Motor #5
Key -, Key +: Motor #6
Key a: Decrease max velocity
Key s: Reset max velocity to PI rad/s
Key d: Increase max velocity
Key q: Quit
Joint limit is not effective. Use with care!"""


class KeyboardController:
    def __init__(self):
        self.key_event_latched = 0
        self.key_trigger_time = [0.0] * 6
        self.acc = 0.1 * math.pi
        self.dec = 0.01 * math.pi
        self.vel_max = math.pi
        self.running = True
        self.vel_target = [0.0] * 6
        self.vel_target_lock = threading.Lock()

    def handle_key_input(self, key_char):
        """Handle keyboard input from TTY comm handler"""
        now = time.time()

        if key_char == "1":
            self.key_event_latched |= 1 << 0
            self.key_trigger_time[0] = now
        elif key_char == "2":
            self.key_event_latched |= 1 << 1
            self.key_trigger_time[0] = now
        elif key_char == "3":
            self.key_event_latched |= 1 << 2
            self.key_trigger_time[1] = now
        elif key_char == "4":
            self.key_event_latched |= 1 << 3
            self.key_trigger_time[1] = now
        elif key_char == "5":
            self.key_event_latched |= 1 << 4
            self.key_trigger_time[2] = now
        elif key_char == "6":
            self.key_event_latched |= 1 << 5
            self.key_trigger_time[2] = now
        elif key_char == "7":
            self.key_event_latched |= 1 << 6
            self.key_trigger_time[3] = now
        elif key_char == "8":
            self.key_event_latched |= 1 << 7
            self.key_trigger_time[3] = now
        elif key_char == "9":
            self.key_event_latched |= 1 << 8
            self.key_trigger_time[4] = now
        elif key_char == "0":
            self.key_event_latched |= 1 << 9
            self.key_trigger_time[4] = now
        elif key_char == "-":
            self.key_event_latched |= 1 << 10
            self.key_trigger_time[5] = now
        elif key_char == "=":
            self.key_event_latched |= 1 << 11
            self.key_trigger_time[5] = now
        elif key_char == "a":
            if self.vel_max > 0.1:
                self.vel_max -= 0.1
                print(f"Max joint velocity decreased to {self.vel_max:.1f} rad/s")
        elif key_char == "s":
            self.vel_max = math.pi
            print(f"Max joint velocity reset to {self.vel_max:.1f} rad/s")
        elif key_char == "d":
            if self.vel_max < math.pi * 2 - 0.1:
                self.vel_max += 0.1
                print(f"Max joint velocity increased to {self.vel_max:.1f} rad/s")
        elif key_char == "q":
            self.running = False

    def throttle_keys(self):
        """Throttle key events to prevent too rapid triggering"""
        while self.running:
            now = time.time()
            for i in range(12):
                if (self.key_event_latched & (1 << i)) and (
                    now - self.key_trigger_time[i // 2] > 0.1
                ):  # 100ms
                    self.key_event_latched &= ~(1 << i)
            time.sleep(0.001)  # 1ms sleep

    def update_velocities(self):
        """Update target velocities based on key events"""
        while self.running:
            with self.vel_target_lock:
                for i in range(6):
                    if self.key_event_latched & (
                        1 << (2 * i)
                    ):  # Positive direction key
                        if self.vel_target[i] < self.vel_max - self.acc:
                            self.vel_target[i] += self.acc
                        else:
                            self.vel_target[i] = self.vel_max
                    elif self.key_event_latched & (
                        1 << (2 * i + 1)
                    ):  # Negative direction key
                        if self.vel_target[i] > -self.vel_max + self.acc:
                            self.vel_target[i] -= self.acc
                        else:
                            self.vel_target[i] = -self.vel_max
                    else:  # No key pressed, decelerate
                        if self.vel_target[i] > self.dec:
                            self.vel_target[i] -= self.dec
                        elif self.vel_target[i] < -self.dec:
                            self.vel_target[i] += self.dec
                        else:
                            self.vel_target[i] = 0.0

            time.sleep(0.001)  # 1ms sleep

    def get_velocity_targets(self):
        """Get current velocity targets thread-safely"""
        with self.vel_target_lock:
            return self.vel_target.copy()


def main():
    controller = KeyboardController()

    try:
        # Create executor
        executor = ah.create_asio_executor(1)
        io_context = executor.get_io_context()

        # Create arm
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

        if not arm.init(io_context, "can0", 250):
            raise RuntimeError("The arm initialization failed.")

        arm.enable()
        arm.set_param("arm.control_mode", ah.MotorControlMode.CSV)

        # Create TTY communication handler (same as C++ version)
        tty_comm = ah.TTYCommHandler.create("tty_comm", "tty", "", 1000, io_context)
        if tty_comm is None:
            raise RuntimeError("Failed to create TTY communication handler")

        # Add read callback for TTY input
        tty_comm.add_read_callback("test", controller.handle_key_input)

        print(INSTRUCTIONS)

        # Start background threads
        throttle_thread = threading.Thread(target=controller.throttle_keys, daemon=True)
        velocity_thread = threading.Thread(
            target=controller.update_velocities, daemon=True
        )

        throttle_thread.start()
        velocity_thread.start()

        # Start TTY communication
        if not tty_comm.start():
            raise RuntimeError("Failed to start TTY communication handler")

        # Main control loop
        while controller.running:
            vel_targets = controller.get_velocity_targets()
            arm.csv(vel_targets)
            time.sleep(0.004)  # 4ms sleep, same as C++ version

        # Cleanup
        tty_comm.delete_read_callback("test")
        arm.disable()
        arm.uninit()

    except KeyboardInterrupt:
        controller.running = False
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        controller.running = False


if __name__ == "__main__":
    main()
