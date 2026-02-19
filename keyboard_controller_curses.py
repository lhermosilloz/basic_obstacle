#!/usr/bin/env python3
import asyncio
import curses
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

class DroneKeyboardControllerCurses:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.drone = System()
        self.is_flying = False
        self.max_velocity = 2.0  # m/s
        self.max_yaw_rate = 90.0  # deg/s
        self.should_exit = False
        self.should_land = False
        self.key_state = set()

    async def connect_and_setup(self):
        await self.drone.connect(system_address="udpin://0.0.0.0:14550")
        self.stdscr.addstr(0, 0, "Waiting for drone to connect...\n")
        self.stdscr.refresh()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.stdscr.addstr(1, 0, "Drone connected!\n")
                self.stdscr.refresh()
                break
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        self.stdscr.addstr(2, 0, "Starting offboard mode...\n")
        self.stdscr.refresh()
        await self.drone.offboard.start()
        self.stdscr.addstr(3, 0, "Arming drone...\n")
        self.stdscr.refresh()
        await self.drone.action.arm()
        self.stdscr.addstr(4, 0, "Drone ready for keyboard control!\n")
        self.stdscr.refresh()
        self.is_flying = True

    def print_controls(self):
        self.stdscr.clear()
        controls = [
            "="*50,
            "DRONE KEYBOARD CONTROLLER (CURSES)",
            "="*50,
            "Controls:",
            "  ↑/↓     - Forward/Backward",
            "  ←/→     - Left/Right Roll",
            "  SPACE   - Up",
            "  SHIFT   - Down",
            "  A/D     - Yaw Left/Right",
            "  L       - Land",
            "  Q/ESC   - Emergency Stop & Exit",
            "="*50,
            "Drone is ready! Use keys to control movement.",
            "Press Q or ESC to exit safely."
        ]
        max_y, max_x = self.stdscr.getmaxyx()
        # Only print as many lines as fit in the terminal
        for idx, line in enumerate(controls):
            if 6 + idx < max_y:
                self.stdscr.addstr(6 + idx, 0, line[:max_x-1])
        self.stdscr.refresh()

    def get_velocity_from_keys(self):
        forward = 0.0
        right = 0.0
        down = 0.0
        yaw_rate = 0.0
        # Arrow keys
        if curses.KEY_UP in self.key_state:
            forward = self.max_velocity
        elif curses.KEY_DOWN in self.key_state:
            forward = -self.max_velocity
        if curses.KEY_LEFT in self.key_state:
            right = -self.max_velocity
        elif curses.KEY_RIGHT in self.key_state:
            right = self.max_velocity
        # Space/Shift for up/down
        if ord(' ') in self.key_state:
            down = -self.max_velocity
        # A/D for yaw
        if ord('a') in self.key_state:
            yaw_rate = -self.max_yaw_rate
        if ord('d') in self.key_state:
            yaw_rate = self.max_yaw_rate
        return forward, right, down, yaw_rate

    async def control_loop(self):
        self.print_controls()
        self.stdscr.keypad(True)  # Enable keypad mode for arrow keys
        curses.halfdelay(1)  # Wait up to 0.1s for input, then continue
        max_y, max_x = self.stdscr.getmaxyx()
        try:
            while self.is_flying:
                key = self.stdscr.getch()
                # Emergency stop
                if key in (ord('q'), 27):  # Q or ESC
                    if max_y > 20:
                        self.stdscr.addstr(20, 0, "Q/ESC pressed - Emergency stop!"[:max_x-1])
                        self.stdscr.refresh()
                    break
                # Land
                if key == ord('l'):
                    if max_y > 21:
                        self.stdscr.addstr(21, 0, "L pressed - Landing..."[:max_x-1])
                        self.stdscr.refresh()
                    break
                # Movement keys: add to key_state on press, remove on release
                movement_keys = [curses.KEY_UP, curses.KEY_DOWN, curses.KEY_LEFT, curses.KEY_RIGHT, ord(' '), ord('a'), ord('d')]
                if key in movement_keys:
                    self.key_state.add(key)
                # Remove keys that are no longer pressed (simulate key release)
                # If no key is pressed, getch() returns -1, so we do not clear key_state immediately
                # Instead, we keep keys in key_state for continuous movement
                # To stop movement, user must release the key and press another key or ESC
                forward, right, down, yaw_rate = self.get_velocity_from_keys()
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(forward, right, down, yaw_rate)
                )
                await asyncio.sleep(0.05)
                # Remove non-held keys (simulate key release)
                # This is a workaround: after sending the command, clear key_state unless the key is still being held
                if key != -1 and key in self.key_state:
                    self.key_state.remove(key)
        except KeyboardInterrupt:
            if max_y > 22:
                self.stdscr.addstr(22, 0, "Keyboard interrupt received!"[:max_x-1])
                self.stdscr.refresh()
        await self.emergency_stop()

    async def emergency_stop(self):
        max_y, max_x = self.stdscr.getmaxyx()
        if max_y > 23:
            self.stdscr.addstr(23, 0, "Stopping drone..."[:max_x-1])
            self.stdscr.refresh()
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1)
        if max_y > 24:
            self.stdscr.addstr(24, 0, "Landing"[:max_x-1])
            self.stdscr.refresh()
        await self.drone.action.land()
        if max_y > 25:
            self.stdscr.addstr(25, 0, "Waiting for disarm..."[:max_x-1])
            self.stdscr.refresh()
        async for armed in self.drone.telemetry.armed():
            if not armed:
                if max_y > 26:
                    self.stdscr.addstr(26, 0, "Drone has disarmed successfully!"[:max_x-1])
                    self.stdscr.refresh()
                break
        if max_y > 27:
            self.stdscr.addstr(27, 0, "Stopping offboard mode..."[:max_x-1])
            self.stdscr.refresh()
        await self.drone.offboard.stop()
        if max_y > 28:
            self.stdscr.addstr(28, 0, "Control session ended safely."[:max_x-1])
            self.stdscr.refresh()
        self.is_flying = False

async def main_curses(stdscr):
    controller = DroneKeyboardControllerCurses(stdscr)
    try:
        await controller.connect_and_setup()
        await controller.control_loop()
    except Exception as e:
        max_y, max_x = stdscr.getmaxyx()
        if max_y > 29:
            stdscr.addstr(29, 0, f"Error: {e}"[:max_x-1])
            stdscr.refresh()
        if controller.is_flying:
            await controller.emergency_stop()

if __name__ == "__main__":
    print("Starting Drone Keyboard Controller (curses)...")
    print("Make sure the drone is connected and ready!")
    print("Press Q or ESC at any time to emergency stop.\n")
    import curses
    asyncio.run(curses.wrapper(main_curses))
