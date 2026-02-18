#!/usr/bin/env python3
import asyncio
import keyboard
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

class DroneKeyboardController:
    def __init__(self):
        self.drone = System()
        self.is_flying = False
        self.max_velocity = 2.0  # m/s
        self.max_yaw_rate = 90.0  # deg/s
        
    async def connect_and_setup(self):
        """Connect to drone and start offboard mode"""
        await self.drone.connect(system_address="udpin://0.0.0.0:14551")
        
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break
                
        # Set initial velocity to zero
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        
        # Start offboard mode
        print("Starting offboard mode...")
        await self.drone.offboard.start()
        
        # Arm the drone
        print("Arming drone...")
        await self.drone.action.arm()
        
        print("Drone ready for keyboard control!")
        self.is_flying = True

    def get_velocity_from_keys(self):
        """Convert keyboard input to velocity commands"""
        forward = 0.0
        right = 0.0
        down = 0.0
        yaw_rate = 0.0
        
        # Forward/Backward (Up/Down arrow keys)
        if keyboard.is_pressed('up'):
            forward = self.max_velocity
        elif keyboard.is_pressed('down'):
            forward = -self.max_velocity
            
        # Left/Right Roll (Left/Right arrow keys)  
        if keyboard.is_pressed('left'):
            right = -self.max_velocity
        elif keyboard.is_pressed('right'):
            right = self.max_velocity
            
        # Up/Down (Space/Shift keys)
        if keyboard.is_pressed('space'):
            down = -self.max_velocity  # Negative for up in NED frame
        elif keyboard.is_pressed('shift'):
            down = self.max_velocity   # Positive for down in NED frame
            
        # Yaw Left/Right (A/D keys)
        if keyboard.is_pressed('a'):
            yaw_rate = -self.max_yaw_rate  # Negative for left
        elif keyboard.is_pressed('d'):
            yaw_rate = self.max_yaw_rate   # Positive for right
            
        return forward, right, down, yaw_rate

    def print_controls(self):
        """Print control instructions"""
        print("\n" + "="*50)
        print("DRONE KEYBOARD CONTROLLER")
        print("="*50)
        print("Controls:")
        print("  ↑/↓     - Forward/Backward")
        print("  ←/→     - Left/Right Roll")  
        print("  SPACE   - Up")
        print("  SHIFT   - Down")
        print("  A/D     - Yaw Left/Right")
        print("  L       - Land")
        print("  ESC     - Emergency Stop & Exit")
        print("="*50)     
        print("Drone is ready! Use keys to control movement.")
        print("Press ESC to exit safely.\n")

    async def control_loop(self):
        """Main control loop"""
        self.print_controls()
        
        try:
            while self.is_flying:
                # Check for exit conditions
                if keyboard.is_pressed('esc'):
                    print("ESC pressed - Emergency stop!")
                    break
                    
                if keyboard.is_pressed('l'):
                    print("L pressed - Landing...")
                    break
                
                # Get velocity commands from keyboard
                forward, right, down, yaw_rate = self.get_velocity_from_keys()
                
                # Send velocity command to drone
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(forward, right, down, yaw_rate)
                )
                
                # Small delay to prevent overwhelming the system
                await asyncio.sleep(0.05)  # 20Hz update rate
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received!")
        
        await self.emergency_stop()

    async def emergency_stop(self):
        """Stop drone and land safely"""
        print("Stopping drone...")
        
        # Stop all movement
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1)
        
        # Now land at current position
        print("Landing")
        await self.drone.action.land()
        
        # Wait for disarming
        print("Waiting for disarm...")
        async for armed in self.drone.telemetry.armed():
            if not armed:
                print("Drone has disarmed successfully!")
                break
        
        # Stop offboard mode
        print("Stopping offboard mode...")
        await self.drone.offboard.stop()

        print("Control session ended safely.")
        self.is_flying = False

async def main():
    """Main function"""
    controller = DroneKeyboardController()
    
    try:
        await controller.connect_and_setup()
        await controller.control_loop()
    except Exception as e:
        print(f"Error: {e}")
        if controller.is_flying:
            await controller.emergency_stop()

if __name__ == "__main__":
    print("Starting Drone Keyboard Controller...")
    print("Make sure the drone is connected and ready!")
    print("Press Ctrl+C at any time to emergency stop.\n")
    
    # Install keyboard library if not available
    try:
        import keyboard
    except ImportError:
        print("Installing keyboard library...")
        import subprocess
        subprocess.run(["pip", "install", "keyboard"])
        import keyboard
    
    asyncio.run(main())