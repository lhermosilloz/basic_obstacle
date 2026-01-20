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

class BugAlgorithmDrone:
    def __init__(self, target_x, target_y):
        self.drone = System()
        self.lidar = SimpleLidarReader()
        self.min_safe_distance = 2.0  # meters
        self.step_size = 0.5  # meters
        self.altitude = -3.0  # meters (negative for NED)
        
        # Drone footprint parameters
        self.drone_width = 1.0  # drone width in meters
        self.drone_length = 1.0  # drone length in meters
        self.safety_margin = 0.5  # additional safety margin in meters
        
        # Bug algorithm state
        self.target_x = target_x
        self.target_y = target_y
        self.current_x = 0.0
        self.current_y = 0.0
        self.state = "GO_TO_GOAL"  # GO_TO_GOAL, FOLLOW_WALL
        self.wall_follow_direction = "left"  # left or right
        self.hit_point = None  # Where we first hit the obstacle
        
    def distance_to_target(self):
        """Calculate distance to target"""
        return math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
    
    def angle_to_target(self):
        """Calculate angle to target in radians"""
        return math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
    
    def check_footprint_collision(self, move_direction):
        """Check if the drone footprint will collide with obstacles"""
        if self.lidar.latest_scan is None:
            return True  # Assume collision if no LiDAR data
            
        msg = self.lidar.latest_scan
        angle_min = msg.angle_min
        angle_increment = msg.angle_step
        
        # Calculate the corners of the drone footprint after the move
        half_width = (self.drone_width / 2) + self.safety_margin
        half_length = (self.drone_length / 2) + self.safety_margin
        
        # Future position after the move
        future_x = self.current_x + self.step_size * math.cos(move_direction)
        future_y = self.current_y + self.step_size * math.sin(move_direction)
        
        # Check multiple points around the drone's footprint
        check_points = [
            # Corners
            (future_x + half_length, future_y + half_width),   # Front-right
            (future_x + half_length, future_y - half_width),   # Front-left
            (future_x - half_length, future_y + half_width),   # Back-right
            (future_x - half_length, future_y - half_width),   # Back-left
            # Mid-points of edges
            (future_x + half_length, future_y),                # Front center
            (future_x - half_length, future_y),                # Back center
            (future_x, future_y + half_width),                 # Right center
            (future_x, future_y - half_width),                 # Left center
            # Center
            (future_x, future_y)                               # Center
        ]
        
        for point_x, point_y in check_points:
            # Calculate angle and distance from current position to check point
            angle_to_point = math.atan2(point_y - self.current_y, point_x - self.current_x)
            distance_to_point = math.sqrt((point_x - self.current_x)**2 + (point_y - self.current_y)**2)
            
            # Find corresponding LiDAR reading
            if self.check_lidar_collision_at_angle(angle_to_point, distance_to_point, msg):
                print(f"Footprint collision detected at point ({point_x:.1f}, {point_y:.1f})")
                return True
        
        return False
    
    def check_lidar_collision_at_angle(self, target_angle, min_distance, msg):
        """Check if LiDAR detects obstacle at specific angle and distance"""
        angle_min = msg.angle_min
        angle_increment = msg.angle_step
        
        target_angle_deg = math.degrees(target_angle)
        
        # Check LiDAR readings in a small sector around the target angle
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            angle = angle_min + (i * angle_increment)
            angle_deg = math.degrees(angle)
            
            # Normalize angles to -180 to 180
            angle_diff = ((angle_deg - target_angle_deg + 180) % 360) - 180
            
            # If this LiDAR ray is close to our target angle
            if abs(angle_diff) <= 5:  # ±5 degrees tolerance
                if range_val < min_distance:
                    return True
        
        return False
    
    def check_obstacle_in_direction(self, direction_angle, sector_width=20):
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
            
            # Normalize angles to -180 to 180
            angle_diff = ((angle_deg - direction_deg + 180) % 360) - 180
            
            if abs(angle_diff) <= sector_width/2:
                if range_val < self.min_safe_distance:
                    return True
        return False
    
    def find_clear_direction_for_wall_follow(self):
        """Find a clear direction for wall following"""
        if self.lidar.latest_scan is None:
            return None
            
        # Try different angles around the drone
        test_angles = []
        if self.wall_follow_direction == "left":
            # For left wall following, prefer turning left (counter-clockwise)
            test_angles = [90, 45, 135, 0, 180, -45, -90, -135]
        else:
            # For right wall following, prefer turning right (clockwise)
            test_angles = [-90, -45, -135, 0, 180, 45, 90, 135]
        
        for angle_deg in test_angles:
            angle_rad = math.radians(angle_deg)
            # Check both simple obstacle detection and footprint collision
            if (not self.check_obstacle_in_direction(angle_rad, 30) and 
                not self.check_footprint_collision(angle_rad)):
                return angle_rad
        
        return None  # No clear direction found
    
    async def move_to_position(self, x, y):
        """Move drone to specific position"""
        self.current_x = x
        self.current_y = y
        await self.drone.offboard.set_position_ned(PositionNedYaw(x, y, self.altitude, 0.0))
        await asyncio.sleep(1.0)
    
    async def navigate_to_target(self):
        """Main navigation loop using bug algorithm"""
        print(f"Starting navigation to target: ({self.target_x}, {self.target_y})")
        print(f"Drone footprint: {self.drone_width}x{self.drone_length}m + {self.safety_margin}m safety margin")
        
        while self.distance_to_target() > 1.0:  # 1 meter tolerance
            print(f"Current position: ({self.current_x:.1f}, {self.current_y:.1f})")
            print(f"Distance to target: {self.distance_to_target():.1f}m")
            print(f"Current state: {self.state}")
            
            if self.state == "GO_TO_GOAL":
                # Try to go directly to goal
                target_angle = self.angle_to_target()
                
                # Check both simple obstacle detection and footprint collision
                if (self.check_obstacle_in_direction(target_angle, 30) or 
                    self.check_footprint_collision(target_angle)):
                    # Obstacle detected, switch to wall following
                    print("Obstacle or footprint collision detected! Switching to wall following mode.")
                    self.state = "FOLLOW_WALL"
                    self.hit_point = (self.current_x, self.current_y)
                    continue
                
                # Move towards goal
                next_x = self.current_x + self.step_size * math.cos(target_angle)
                next_y = self.current_y + self.step_size * math.sin(target_angle)
                await self.move_to_position(next_x, next_y)
                
            elif self.state == "FOLLOW_WALL":
                # Wall following mode
                clear_direction = self.find_clear_direction_for_wall_follow()
                
                if clear_direction is None:
                    print("No clear direction found! Stopping.")
                    break
                
                # Move in the clear direction
                next_x = self.current_x + self.step_size * math.cos(clear_direction)
                next_y = self.current_y + self.step_size * math.sin(clear_direction)
                await self.move_to_position(next_x, next_y)
                
                # Check if we can switch back to goal-seeking
                target_angle = self.angle_to_target()
                if (not self.check_obstacle_in_direction(target_angle, 30) and 
                    not self.check_footprint_collision(target_angle)):
                    # Check if we've made progress past the hit point
                    if self.hit_point:
                        hit_to_target_dist = math.sqrt((self.target_x - self.hit_point[0])**2 + 
                                                     (self.target_y - self.hit_point[1])**2)
                        current_to_target_dist = self.distance_to_target()
                        
                        if current_to_target_dist < hit_to_target_dist:
                            print("Clear path to goal found! Switching back to goal-seeking mode.")
                            self.state = "GO_TO_GOAL"
                            self.hit_point = None
            
            # Safety check
            if abs(self.current_x) > 100 or abs(self.current_y) > 100:
                print("Safety boundary reached! Stopping navigation.")
                break
        
        print("Navigation complete!")

async def run():
    # Set target coordinates (you can modify these)
    TARGET_X = 10.0  # meters
    TARGET_Y = 5.0   # meters
    
    controller = BugAlgorithmDrone(TARGET_X, TARGET_Y)
    drone = controller.drone
    
    # Connect to drone
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    # Start LiDAR
    print("Starting LiDAR...")
    if not controller.lidar.start_listening():
        print("Failed to start LiDAR, aborting mission")
        return
    
    # Wait for initial LiDAR data
    print("Waiting for LiDAR data...")
    while controller.lidar.latest_scan is None:
        await asyncio.sleep(0.1)
    print("LiDAR data received!")

    # Arm the drone
    print("Arming drone...")
    await drone.action.arm()

    # Set initial position
    print("Setting initial position setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    # Start offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    # Takeoff to 3 meters
    print("Taking off to 3 meters...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(8)  # Give time to reach altitude

    # Navigate to target using bug algorithm
    await controller.navigate_to_target()
    
    # Hold position briefly
    print("Holding position for safety...")
    await asyncio.sleep(3)

    # Land safely
    print("Landing safely...")
    await drone.offboard.set_position_ned(PositionNedYaw(controller.current_x, controller.current_y, 0.0, 0.0))
    await asyncio.sleep(8)  # Give time to descend

    print("Stopping offboard mode...")
    await drone.offboard.stop()

    print("Final landing command...")
    await drone.action.land()

    print("Mission complete!")

if __name__ == "__main__":
    asyncio.run(run())