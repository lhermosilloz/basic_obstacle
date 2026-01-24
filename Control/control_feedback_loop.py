#!/usr/bin/env python3

import asyncio
import math
import time
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

class SimpleFeedbackController:
    def __init__(self, drone):
        self.drone = drone
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -1.0  # 1 meter altitude
        self.target_yaw = 90.0
        
        # Current position (will be updated from telemetry)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        
        # Control parameters
        self.position_tolerance = 0.2  # meters
        self.yaw_tolerance = 5.0  # degrees
        self.max_step_size = 0.5  # maximum movement per command
        
    async def update_current_position(self):
        """Get current position from drone telemetry"""
        try:
            # Get position in NED coordinates
            async for position_ned in self.drone.telemetry.position_velocity_ned():
                self.current_x = position_ned.position.north_m
                self.current_y = position_ned.position.east_m
                self.current_z = position_ned.position.down_m
                break
                
            async for heading in self.drone.telemetry.heading():
                self.current_yaw = heading.heading_deg
                if self.current_yaw < 0:
                    self.current_yaw += 360  # Convert to [0, 360]
                break
        except Exception as e:
            print(f"Telemetry error: {e}")
            print("Using estimated position (telemetry unavailable)")
    
    def calculate_position_error(self):
        """Calculate position error in each axis"""
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_z = self.target_z - self.current_z
        
        # Calculate total distance error
        distance_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        return error_x, error_y, error_z, distance_error
    
    def calculate_yaw_error(self):
        """Calculate yaw error with proper angle wrapping"""
        error = self.target_yaw - self.current_yaw
        
        # Wrap angle to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
            
        return error
    
    def at_target(self):
        """Check if we're at the target position"""
        _, _, _, distance_error = self.calculate_position_error()
        yaw_error = abs(self.calculate_yaw_error())
        
        position_ok = distance_error < self.position_tolerance
        yaw_ok = yaw_error < self.yaw_tolerance
        
        return position_ok and yaw_ok
    
    async def move_to_target(self, target_x, target_y, target_z=-1.0, target_yaw=90.0):
        """Move to target using feedback control"""
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_yaw = target_yaw
        
        print(f"Moving to target: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}, {target_yaw:.0f}°)")
        
        max_iterations = 100
        iteration = 0
        
        while not self.at_target() and iteration < max_iterations:
            # Update current position from telemetry
            await self.update_current_position()
            
            # Calculate errors
            error_x, error_y, error_z, distance_error = self.calculate_position_error()
            yaw_error = self.calculate_yaw_error()
            
            print(f"Iteration {iteration}: Position ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f}), "
                  f"Errors: x={error_x:.2f}, y={error_y:.2f}, z={error_z:.2f}, yaw={yaw_error:.1f}°, "
                  f"distance={distance_error:.2f}")
            
            # Send command directly to target (let flight controller handle the path)
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(self.target_x, self.target_y, self.target_z, self.target_yaw)
            )
            
            # Wait before next iteration
            await asyncio.sleep(0.5)  # 2 Hz control loop
            iteration += 1
        
        if self.at_target():
            print(f"Target reached in {iteration} iterations!")
        else:
            print(f"Target not reached after {max_iterations} iterations")
        
        return self.at_target()

async def test_feedback_loop():
    """Test the feedback control loop"""
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate OK!")
            break
    
    # Arm and start offboard mode
    print("Arming drone...")
    await drone.action.arm()
    
    # Start with current position
    print("Setting initial position...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 90.0))
    
    print("Starting offboard mode...")
    await drone.offboard.start()
    
    # Take off
    print("Taking off...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 90.0))
    await asyncio.sleep(10)  # Give more time for takeoff
    
    # Create feedback controller
    controller = SimpleFeedbackController(drone)
    
    # Test sequence: move to different waypoints
    waypoints = [
        (2.0, 0.0, -1.0, 90.0),   # Move forward 2m
        (2.0, 2.0, -1.0, 180.0),  # Move right 2m, turn around
        (0.0, 2.0, -1.0, 270.0),  # Move back, turn left
        (0.0, 0.0, -1.0, 0.0),    # Return to start, turn north
    ]
    
    for i, (x, y, z, yaw) in enumerate(waypoints):
        print(f"\n--- Waypoint {i+1}: ({x}, {y}, {z}, {yaw}°) ---")
        success = await controller.move_to_target(x, y, z, yaw)
        if success:
            print("✓ Waypoint reached successfully")
        else:
            print("✗ Waypoint not reached")
        
        # Hold position for a moment
        await asyncio.sleep(2)
    
    # Land
    print("\nLanding...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)
    
    await drone.offboard.stop()
    await drone.action.land()
    print("Test complete!")

if __name__ == "__main__":
    print("Starting simple feedback control loop test...")
    asyncio.run(test_feedback_loop())
