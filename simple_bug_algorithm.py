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

class DynamicObstacleAvoidanceDrone:
    def __init__(self):
        self.min_safe_distance = 1  # meters
        self.move_step = 0.001  # meters to move per step

    def check_where_obstacle(self, lidar):
        """Check where the obstacles are relative to the drone."""
        direction_distances = []
        if lidar.latest_scan is None:
            return direction_distances
        
        msg = lidar.latest_scan
        angle_min = msg.angle_min
        angle_increment = msg.angle_step

        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue

            angle = angle_min + (i * angle_increment)
            angle_deg = math.degrees(angle)

            if range_val < self.min_safe_distance:
                direction_distances.append((angle_deg, range_val))
        
        direction_distances.sort(key=lambda x: x[1])
        return direction_distances[:2] if len(direction_distances) >= 2 else direction_distances

    def calculate_scissor_escape_direction(self, obstacles):
        """Calculate the escape direction using scissor effect"""
        if len(obstacles) < 2:
            if len(obstacles) == 1:
                obstacle_angle = obstacles[0][0]
                escape_angle = (obstacle_angle + 180) % 360
                if escape_angle > 180:
                    escape_angle -= 360
                return escape_angle
            else:
                return None
        
        angle1, dist1 = obstacles[0]
        angle2, dist2 = obstacles[1]
        
        blade1_angle = (angle1 + 180) % 360
        blade2_angle = (angle2 + 180) % 360
        
        if blade1_angle > 180:
            blade1_angle -= 360
        if blade2_angle > 180:
            blade2_angle -= 360
        
        angle_diff = abs(blade1_angle - blade2_angle)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        escape_angle = (blade1_angle + blade2_angle) / 2
        
        if abs(blade1_angle - blade2_angle) > 180:
            escape_angle = (escape_angle + 180) % 360
            if escape_angle > 180:
                escape_angle -= 360
        
        return escape_angle

    def angle_to_movement(self, angle_deg):
        """Convert angle in degrees to x,y movement components"""
        if angle_deg is None:
            return 0.0, 0.0
            
        angle_rad = math.radians(angle_deg)
        dx = self.move_step * math.cos(angle_rad)
        dy = -self.move_step * math.sin(angle_rad)
        return dx, dy

class HybridBugAlgorithmDrone:
    def __init__(self, target_x, target_y):
        self.drone = System()
        self.lidar = SimpleLidarReader()
        self.dynamic_avoidance = DynamicObstacleAvoidanceDrone()
        
        # Bug algorithm parameters
        self.min_safe_distance = 1  # meters
        self.step_size = 0.001  # meters
        self.altitude = -2.0  # meters (negative for NED)
        
        # Target and position
        self.target_x = target_x
        self.target_y = target_y
        self.current_x = 0.0
        self.current_y = 0.0
        
        # State machine
        self.state = "GO_TO_GOAL"  # GO_TO_GOAL, DYNAMIC_AVOID, FOLLOW_WALL
        self.hit_point = None
        self.dynamic_avoid_timeout = 0
        self.max_dynamic_avoid_steps = 50  # Maximum steps in dynamic avoidance mode
        
    def distance_to_target(self):
        """Calculate distance to target"""
        return math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
    
    def angle_to_target(self):
        """Calculate angle to target in radians"""
        return math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
    
    def check_obstacle_in_direction(self, direction_angle, sector_width=30):
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
            
            angle_diff = ((angle_deg - direction_deg + 180) % 360) - 180
            
            if abs(angle_diff) <= sector_width/2:
                if range_val < self.min_safe_distance:
                    return True
        return False
    
    def find_clear_direction_for_wall_follow(self):
        """Find a clear direction for wall following"""
        if self.lidar.latest_scan is None:
            return None
            
        test_angles = [90, 45, 135, 0, 180, -45, -90, -135]
        
        for angle_deg in test_angles:
            angle_rad = math.radians(angle_deg)
            if not self.check_obstacle_in_direction(angle_rad, 30):
                return angle_rad
        
        return None
    
    async def move_to_position(self, x, y):
        """Move drone to specific position"""
        self.current_x = x
        self.current_y = y
        await self.drone.offboard.set_position_ned(PositionNedYaw(x, y, self.altitude, 0.0))
        await asyncio.sleep(0.001)
    
    async def navigate_to_target(self):
        """Main navigation loop using hybrid bug algorithm with dynamic avoidance"""
        print(f"Starting hybrid navigation to target: ({self.target_x}, {self.target_y})")
        
        while self.distance_to_target() > 1.0:  # 1 meter tolerance
            print(f"Position: ({self.current_x:.1f}, {self.current_y:.1f}), Distance: {self.distance_to_target():.1f}m, State: {self.state}")
            
            # Check for nearby obstacles for dynamic avoidance
            obstacles = self.dynamic_avoidance.check_where_obstacle(self.lidar)
            
            if self.state == "GO_TO_GOAL":
                target_angle = self.angle_to_target()
                
                # Check if we need obstacle avoidance
                if obstacles and len(obstacles) > 0:
                    print("Obstacles detected! Switching to dynamic avoidance mode.")
                    self.state = "DYNAMIC_AVOID"
                    self.dynamic_avoid_timeout = 0
                    continue
                
                # Move towards goal
                next_x = self.current_x + self.step_size * math.cos(target_angle)
                next_y = self.current_y + self.step_size * math.sin(target_angle)
                await self.move_to_position(next_x, next_y)
                
            elif self.state == "DYNAMIC_AVOID":
                # Use dynamic obstacle avoidance
                if obstacles:
                    escape_angle = self.dynamic_avoidance.calculate_scissor_escape_direction(obstacles)
                    
                    if escape_angle is not None:
                        dx, dy = self.dynamic_avoidance.angle_to_movement(escape_angle)
                        next_x = self.current_x + dx
                        next_y = self.current_y + dy
                        
                        print(f"Dynamic avoidance: moving {dx:.3f}, {dy:.3f}")
                        await self.move_to_position(next_x, next_y)
                    
                    self.dynamic_avoid_timeout += 1
                    
                    # Check if we can return to goal-seeking
                    target_angle = self.angle_to_target()
                    if not self.check_obstacle_in_direction(target_angle, 45):
                        print("Path to goal is clear! Returning to goal-seeking mode.")
                        self.state = "GO_TO_GOAL"
                        self.dynamic_avoid_timeout = 0
                    
                    # Timeout - switch to wall following if stuck too long
                    elif self.dynamic_avoid_timeout > self.max_dynamic_avoid_steps:
                        print("Dynamic avoidance timeout! Switching to wall following.")
                        self.state = "FOLLOW_WALL"
                        self.hit_point = (self.current_x, self.current_y)
                        self.dynamic_avoid_timeout = 0
                
                else:
                    # No obstacles - return to goal seeking
                    print("No obstacles detected! Returning to goal-seeking mode.")
                    self.state = "GO_TO_GOAL"
                    self.dynamic_avoid_timeout = 0
                
            elif self.state == "FOLLOW_WALL":
                # Traditional wall following
                clear_direction = self.find_clear_direction_for_wall_follow()
                
                if clear_direction is None:
                    print("No clear direction found! Stopping.")
                    break
                
                next_x = self.current_x + self.step_size * math.cos(clear_direction)
                next_y = self.current_y + self.step_size * math.sin(clear_direction)
                await self.move_to_position(next_x, next_y)
                
                # Check if we can switch back to goal-seeking
                target_angle = self.angle_to_target()
                if not self.check_obstacle_in_direction(target_angle, 30):
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
    # Set target coordinates
    TARGET_X = 0  # meters
    TARGET_Y = 30   # meters
    
    controller = HybridBugAlgorithmDrone(TARGET_X, TARGET_Y)
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

    # Takeoff
    print("Taking off to 2 meters...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(8)  # Give time to reach altitude

    # Navigate to target using hybrid algorithm
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