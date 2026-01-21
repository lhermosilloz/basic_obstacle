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

class BugAlgorithm:
    def __init__(self, target_x, target_y):
        self.drone = System()
        self.lidar = SimpleLidarReader()
        
        # Parameters
        self.min_safe_distance = 2.0  # meters
        self.step_size = 0.3  # meters per step
        self.altitude = -3.0  # meters (negative for NED)
        
        # Target and position
        self.target_x = target_x
        self.target_y = target_y
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Bug algorithm state
        self.state = "GO_TO_GOAL"  # GO_TO_GOAL or FOLLOW_WALL
        self.hit_point = None  # Where we first encountered obstacle
        
    def distance_to_target(self):
        """Calculate distance to target"""
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
    
    def find_clear_direction_left_preference(self):
        """Find a clear direction, preferring left (counter-clockwise)"""
        if self.lidar.latest_scan is None:
            return None
            
        # Try angles in order of left preference
        # 0° = forward, 90° = left, -90° = right, 180° = back
        test_angles = [45, 90, 135, 0, -45, 180, -90, -135]
        
        for angle_deg in test_angles:
            angle_rad = math.radians(angle_deg)
            if not self.check_obstacle_in_direction(angle_rad, 30):
                print(f"  Clear direction found at {angle_deg}°")
                return angle_rad
        
        return None
    
    async def move_to_position(self, x, y):
        """Move drone to specific position and face the direction of movement"""
        # Calculate movement direction
        dx = x - self.current_x
        dy = y - self.current_y
        movement_distance = math.sqrt(dx*dx + dy*dy)
        
        # Always update heading based on movement direction
        if movement_distance > 0.01:
            yaw_rad = math.atan2(dy, dx)
            self.current_yaw = math.degrees(yaw_rad)  # Convert to degrees
        
        self.current_x = x
        self.current_y = y
        
        # Set position with current yaw (in degrees)
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(x, y, self.altitude, self.current_yaw))
        await asyncio.sleep(0.2)
    
    async def navigate_to_target(self):
        """Main navigation loop using Bug Algorithm"""
        print(f"Starting Bug Algorithm navigation")
        print(f"Target: ({self.target_x}, {self.target_y})")
        print(f"Safe distance: {self.min_safe_distance}m")
        
        step_count = 0
        
        while self.distance_to_target() > 0.5:
            step_count += 1
            distance = self.distance_to_target()
            
            print(f"\n--- Step {step_count} ---")
            print(f"Position: ({self.current_x:.1f}, {self.current_y:.1f})")
            print(f"Distance to target: {distance:.1f}m")
            print(f"State: {self.state}")
            
            if self.state == "GO_TO_GOAL":
                # Move directly toward goal
                target_angle = self.angle_to_target()
                
                # Check if obstacle is blocking the path
                if self.check_obstacle_in_direction(target_angle, 40):
                    print(">> OBSTACLE DETECTED! Switching to wall following mode.")
                    self.state = "FOLLOW_WALL"
                    self.hit_point = (self.current_x, self.current_y)
                    continue
                
                # Adjust step size based on distance to target (slow down when close)
                current_step = self.step_size
                if distance < 10.0:
                    # EXTREME slowing - use only 10% of remaining distance
                    current_step = max(0.03, distance * 0.01)
                    print(f"Slowing down, step size: {current_step:.3f}m, distance: {distance:.2f}m")
                
                # Move towards goal
                next_x = self.current_x + current_step * math.cos(target_angle)
                next_y = self.current_y + current_step * math.sin(target_angle)
                print(f"Moving toward goal: ({next_x:.1f}, {next_y:.1f})")
                await self.move_to_position(next_x, next_y)
                
            elif self.state == "FOLLOW_WALL":
                # Follow wall until we can go to goal again
                clear_direction = self.find_clear_direction_left_preference()
                
                if clear_direction is None:
                    print("ERROR: No clear direction found! Stuck!")
                    break
                
                # Move in the clear direction
                next_x = self.current_x + self.step_size * math.cos(clear_direction)
                next_y = self.current_y + self.step_size * math.sin(clear_direction)
                print(f"Following wall: ({next_x:.1f}, {next_y:.1f})")
                await self.move_to_position(next_x, next_y)
                
                # Check if we can return to goal-seeking
                target_angle = self.angle_to_target()
                
                if not self.check_obstacle_in_direction(target_angle, 40):
                    # Path to goal is clear, but only switch if we've made progress
                    if self.hit_point:
                        hit_to_target_dist = math.sqrt(
                            (self.target_x - self.hit_point[0])**2 + 
                            (self.target_y - self.hit_point[1])**2)
                        current_to_target_dist = self.distance_to_target()
                        
                        print(f"  Hit point distance: {hit_to_target_dist:.1f}m")
                        print(f"  Current distance: {current_to_target_dist:.1f}m")
                        
                        if current_to_target_dist < hit_to_target_dist - 0.5:
                            print(">> PROGRESS MADE! Returning to goal-seeking mode.")
                            self.state = "GO_TO_GOAL"
                            self.hit_point = None
            
            # Safety boundary check
            if abs(self.current_x) > 100 or abs(self.current_y) > 100:
                print("ERROR: Safety boundary exceeded! Stopping navigation.")
                break
            
            # Infinite loop protection
            if step_count > 1000:
                print("ERROR: Too many steps! Stopping navigation.")
                break
        
        print(f"\nNavigation complete!")
        print(f"Final position: ({self.current_x:.1f}, {self.current_y:.1f})")
        print(f"Final distance to target: {self.distance_to_target():.1f}m")

async def run():
    # Set target coordinates (modify these as needed)
    TARGET_X = 0.0  # meters
    TARGET_Y = 70.0   # meters
    
    bug = BugAlgorithm(TARGET_X, TARGET_Y)
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

    # Takeoff
    print("Taking off to 3 meters...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(8)

    # Navigate to target using Bug Algorithm
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
