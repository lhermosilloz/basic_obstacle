#!/usr/bin/env python3
import sys
import asyncio
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

class DroneKeyboardControllerQt(QWidget):
    def __init__(self):
        super().__init__()
        self.drone = System()
        self.is_flying = False
        self.max_velocity = 2.0  # m/s
        self.max_yaw_rate = 90.0  # deg/s
        self.key_state = set()
        self.should_exit = False
        self.should_land = False
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_velocity)
        self.timer.start(50)  # 20Hz

    def init_ui(self):
        self.setWindowTitle('Drone Keyboard Controller (PyQt5)')
        layout = QVBoxLayout()
        self.label = QLabel()
        self.label.setText(
            "Controls:\n"
            "  ↑/↓     - Forward/Backward\n"
            "  ←/→     - Left/Right Roll\n"
            "  SPACE   - Up\n"
            "  SHIFT   - Down\n"
            "  A/D     - Yaw Left/Right\n"
            "  L       - Land\n"
            "  ESC     - Emergency Stop & Exit\n"
            "\nDrone is ready! Use keys to control movement.\n"
            "Press ESC to exit safely.\n"
            "(If keys are not detected, click the window to focus.)"
        )
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.resize(400, 300)
        self.setFocusPolicy(Qt.StrongFocus)

    async def connect_and_setup(self):
        self.label.setText(self.label.text() + "\nWaiting for drone to connect...")
        await self.drone.connect(system_address="udpin://0.0.0.0:14550")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.label.setText(self.label.text() + "\nDrone connected!")
                break
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        self.label.setText(self.label.text() + "\nStarting offboard mode...")
        await self.drone.offboard.start()
        self.label.setText(self.label.text() + "\nArming drone...")
        await self.drone.action.arm()
        self.label.setText(self.label.text() + "\nDrone ready for keyboard control!")
        self.is_flying = True

    def keyPressEvent(self, event):
        key = event.key()
        # Emergency stop
        if key == Qt.Key_Escape:
            self.should_exit = True
        # Land
        if key == Qt.Key_L:
            self.should_land = True
        # Movement keys
        if key in (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right, Qt.Key_Space, Qt.Key_A, Qt.Key_D):
            self.key_state.add(key)

    def keyReleaseEvent(self, event):
        key = event.key()
        if key in self.key_state:
            self.key_state.remove(key)

    def get_velocity_from_keys(self):
        forward = 0.0
        right = 0.0
        down = 0.0
        yaw_rate = 0.0
        if Qt.Key_Up in self.key_state:
            forward = self.max_velocity
        elif Qt.Key_Down in self.key_state:
            forward = -self.max_velocity
        if Qt.Key_Left in self.key_state:
            right = -self.max_velocity
        elif Qt.Key_Right in self.key_state:
            right = self.max_velocity
        if Qt.Key_Space in self.key_state:
            down = -self.max_velocity
        # A/D for yaw
        if Qt.Key_A in self.key_state:
            yaw_rate = -self.max_yaw_rate
        if Qt.Key_D in self.key_state:
            yaw_rate = self.max_yaw_rate
        return forward, right, down, yaw_rate

    def send_velocity(self):
        if not self.is_flying:
            return
        if self.should_exit:
            self.label.setText(self.label.text() + "\nESC pressed - Emergency stop!")
            asyncio.ensure_future(self.emergency_stop())
            self.is_flying = False
            self.timer.stop()
            return
        if self.should_land:
            self.label.setText(self.label.text() + "\nL pressed - Landing...")
            asyncio.ensure_future(self.emergency_stop())
            self.is_flying = False
            self.timer.stop()
            return
        forward, right, down, yaw_rate = self.get_velocity_from_keys()
        asyncio.ensure_future(self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward, right, down, yaw_rate)
        ))

    async def emergency_stop(self):
        self.label.setText(self.label.text() + "\nStopping drone...")
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1)
        self.label.setText(self.label.text() + "\nLanding")
        await self.drone.action.land()
        self.label.setText(self.label.text() + "\nWaiting for disarm...")
        async for armed in self.drone.telemetry.armed():
            if not armed:
                self.label.setText(self.label.text() + "\nDrone has disarmed successfully!")
                break
        self.label.setText(self.label.text() + "\nStopping offboard mode...")
        await self.drone.offboard.stop()
        self.label.setText(self.label.text() + "\nControl session ended safely.")

async def main():
    app = QApplication(sys.argv)
    controller = DroneKeyboardControllerQt()
    controller.show()
    await controller.connect_and_setup()
    loop = asyncio.get_event_loop()
    loop.call_soon(app.exec_)
    await asyncio.sleep(0)  # Let Qt event loop start
    while controller.is_flying:
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    print("Starting Drone Keyboard Controller (PyQt5)...")
    print("Make sure the drone is connected and ready!")
    print("Press ESC at any time to emergency stop.\n")
    try:
        from PyQt5.QtWidgets import QApplication
    except ImportError:
        print("Installing PyQt5 library...")
        import subprocess
        subprocess.run(["pip", "install", "PyQt5"])
        from PyQt5.QtWidgets import QApplication
    asyncio.run(main())
