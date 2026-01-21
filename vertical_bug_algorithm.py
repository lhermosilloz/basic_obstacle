#!/usr/bin/env python3
import os
# Set protobuf implementation to python before importing gz modules
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
import asyncio
import time
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from simple_lidar_reader import SimpleLidarReader

try:
    import gz.transport13 as gz_transport
except ImportError:
    print("Error: gz-transport Python bindings not found")
    print("Install with: sudo apt install python3-gz-transport13")
    exit(1)

class VerticalBugAlgorithm:
    def __init__(self, target_x, target_y, target_altitude=-3.0):
        self.drone = System()
        self.lidar = SimpleLidarReader()
        
        # Parameters
        self.min_safe_distance = 2.0  # meters
        self.step_size = 0.3  # meters per step (horizontal)
        self.climb_step = 1.0  # meters per climb step
        self.min_altitude = -15.0  # Maximum height (NED is negative)
        self.target_altitude = target_altitude  # Desired cruising altitude
        
        # Target and position
        self.target_x = target_x
        self.target_y = target_y
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_altitude = target_altitude
        self.current_yaw = 0.0
        
        # State
        self.state = "GO_TO_GOAL"  # GO_TO_GOAL or CLIMBING
        
    def distance_to_target(self):
        """Calculate horizontal distance to target"""
        return math.sqrt((self.target_x - self.current_x)**2 + 
                        (self.target_y - self.current_y)**2)
    
    def angle_to_target(self):
        """Calculate angle to target in radians"""
        return math.atan2(self.target_y - self.current_y, 
                         self.target_x - self.current_x)
    
    def check_obstacle_in_direction(self, direction_angle, sector_width=40):
        """Check if there's an obstacle in a specific direction"""
        if self.lidar.latest_scan is None:
            return False
            
        msg = self.lidar.latest_scan
        angle_min = msg.angle_min
        angle_increment = msg.angle_step
        
        direction_deg = math.degrees(direction_angle)
        
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            angle = angle_min + (i * angle_increment)
            angle_deg = math.degrees(angle)
            
            # Normalize angle difference to -180 to 180
            angle_diff = ((angle_deg - direction_deg + 180) % 360) - 180
            
            if abs(angle_diff) <= sector_width/2:
                if range_val < self.min_safe_distance:
                    return True
        return False
    
    async def move_to_position(self, x, y, altitude):
        """Move drone to specific position and face the direction of movement"""
        # Calculate movement direction
        dx = x - self.current_x
        dy = y - self.current_y
        movement_distance = math.sqrt(dx*dx + dy*dy)
        
        # Update heading based on horizontal movement direction
        if movement_distance > 0.01:
            yaw_rad = math.atan2(dy, dx)
            self.current_yaw = math.degrees(yaw_rad)
        
        self.current_x = x
        self.current_y = y
        self.current_altitude = altitude
        
        # Set position with current yaw (in degrees)
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(x, y, altitude, self.current_yaw))
        await asyncio.sleep(0.2)
    
    async def navigate_to_target(self):
        """Main navigation loop using Vertical Bug Algorithm"""
        print(f"Starting Vertical Bug Algorithm navigation")
        print(f"Target: ({self.target_x}, {self.target_y}) at altitude {self.target_altitude}m")
        print(f"Safe distance: {self.min_safe_distance}m")
        
        while self.distance_to_target() > 0.5:
            distance = self.distance_to_target()
            
            print(f"\nPosition: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_altitude:.1f})")
            print(f"Distance to target: {distance:.1f}m")
            print(f"State: {self.state}")
            
            if self.state == "GO_TO_GOAL":
                # Try to move toward goal at current altitude
                target_angle = self.angle_to_target()
                
                # Small step size for safety
                current_step = 0.1
                if distance < 10.0:
                    current_step = max(0.05, distance * 0.1)
                
                # Calculate next position
                next_x = self.current_x + current_step * math.cos(target_angle)
                next_y = self.current_y + current_step * math.sin(target_angle)
                
                # Move first
                print(f"Moving toward goal: ({next_x:.1f}, {next_y:.1f})")
                await self.move_to_position(next_x, next_y, self.current_altitude)
                
                # Then check if we hit an obstacle
                if self.check_obstacle_in_direction(target_angle, 40):
                    print(">> OBSTACLE DETECTED! Switching to climbing mode.")
                    self.state = "CLIMBING"
                
            elif self.state == "CLIMBING":
                # Check altitude limit first
                if self.current_altitude <= self.min_altitude:
                    print("ERROR: Maximum altitude reached! Cannot climb higher.")
                    break
                
                # Climb up
                new_altitude = self.current_altitude - self.climb_step
                print(f"Climbing to altitude: {new_altitude:.1f}m")
                await self.move_to_position(self.current_x, self.current_y, new_altitude)
                
                # After climbing, check if path is clear
                target_angle = self.angle_to_target()
                if not self.check_obstacle_in_direction(target_angle, 40):
                    print(">> PATH CLEAR! Returning to goal-seeking mode.")
                    self.state = "GO_TO_GOAL"
            
            # Safety boundary check
            if abs(self.current_x) > 100 or abs(self.current_y) > 100:
                print("ERROR: Safety boundary exceeded! Stopping navigation.")
                break
        
        print(f"\nNavigation complete!")
        print(f"Final position: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_altitude:.1f})")
        print(f"Final distance to target: {self.distance_to_target():.1f}m")
        
        # Descend to target altitude
        if abs(self.current_altitude - self.target_altitude) > 0.5:
            print(f"\nDescending to target altitude {self.target_altitude}m...")
            await self.move_to_position(self.current_x, self.current_y, self.target_altitude)
            await asyncio.sleep(3)

async def run():
    # Set target coordinates (modify these as needed)
    TARGET_X = 0.0  # meters
    TARGET_Y = 30.0  # meters
    TARGET_ALTITUDE = -3.0  # meters (negative in NED)
    
    bug = VerticalBugAlgorithm(TARGET_X, TARGET_Y, TARGET_ALTITUDE)
    drone = bug.drone
    
    # Connect to drone
    print("Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    # Start LiDAR
    print("Starting LiDAR...")
    if not bug.lidar.start_listening():
        print("ERROR: Failed to start LiDAR")
        return
    
    # Wait for LiDAR data
    print("Waiting for LiDAR data...")
    while bug.lidar.latest_scan is None:
        await asyncio.sleep(0.1)
    print("LiDAR active!")

    # Arm the drone
    print("Arming drone...")
    await drone.action.arm()

    # Set initial position
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    # Start offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    # Takeoff to target altitude
    print(f"Taking off to {TARGET_ALTITUDE}m...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, TARGET_ALTITUDE, 0.0))
    await asyncio.sleep(8)

    # Navigate to target using Vertical Bug Algorithm
    await bug.navigate_to_target()
    
    # Hold position
    print("\nHolding position...")
    await asyncio.sleep(3)

    # Land
    print("Landing...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(bug.current_x, bug.current_y, 0.0, 0.0))
    await asyncio.sleep(8)

    print("Stopping offboard mode...")
    await drone.offboard.stop()

    print("Disarming...")
    await drone.action.land()

    print("Mission complete!")

if __name__ == "__main__":
    asyncio.run(run())
