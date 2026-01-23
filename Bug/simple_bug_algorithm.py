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
import random

try:
    import gz.transport13 as gz_transport
except ImportError:
    print("Error: gz-transport Python bindings not found")
    print("Install with: sudo apt install python3-gz-transport13")
    exit(1)

class BugAlgorithm:
    def __init__(self, drone, lidar, goal_x, goal_y, altitude=-1.0):
        self.drone = drone
        self.lidar = lidar
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.altitude = altitude
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 90.0

        # Synchronize waypoints commands with current positions
        self.target_x = 0.0
        self.target_y = 0.0
        self.position_tolerance = 0.2  # meters
        
        # Bug algorithm states
        self.state = "GO_TO_GOAL"  # or "WALL_FOLLOW"
        self.wall_follow_start_x = None
        self.wall_follow_start_y = None
        self.closest_distance_to_goal = float('inf')
    
    def distance_to_goal(self, x=None, y=None):
        """Calculate distance from current position (or given x,y) to goal"""
        if x is None:
            x = self.current_x
        if y is None:
            y = self.current_y
        return math.sqrt((self.goal_x - x)**2 + (self.goal_y - y)**2)
    
    def angle_to_goal(self):
        """Calculate angle from current position to goal"""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        return math.degrees(math.atan2(dy, dx))
    
    def at_goal(self, tolerance=0.5):
        """Check if we're close enough to the goal"""
        return self.distance_to_goal() < tolerance
    
    async def move_toward_goal(self, step_size=0.2):
        """Move one step toward the goal"""
        distance = self.distance_to_goal()
        if distance < step_size:
            # Move directly to goal
            self.current_x = self.goal_x
            self.current_y = self.goal_y
        else:
            # Move step_size toward goal
            angle_rad = math.radians(self.angle_to_goal())
            self.current_x += step_size * math.cos(angle_rad)
            self.current_y += step_size * math.sin(angle_rad)
        
        # Update yaw to face goal
        self.current_yaw = self.angle_to_goal()
        
        print(f"Moving toward goal: ({self.current_x:.1f}, {self.current_y:.1f}), distance: {distance:.1f}m")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(self.current_x, self.current_y, self.altitude, self.current_yaw)
        )

    async def follow_left_wall(self, step_size=0.2):
        """Follow wall by moving left when blocked in front"""
        if not self.lidar.check_front_obstacles():
            print("Path to goal clear, switching back to GO_TO_GOAL mode.")
            self.state = "GO_TO_GOAL"
            return

        # Simple wall following: move left when front blocked, forward when front clear
        if self.lidar.check_front_obstacles() and not self.lidar.check_left_obstacles():
            # Move left
            angle_rad = math.radians(self.current_yaw - 90)  # 90 degrees left
            self.current_x += step_size * math.cos(angle_rad)
            self.current_y += step_size * math.sin(angle_rad)
            print(f"Wall following - moving left: ({self.current_x:.1f}, {self.current_y:.1f})")
        elif self.lidar.check_front_obstacles() and self.lidar.check_left_obstacles():
            # Blocked on front and left, turn left and continue process
            angle_rad = math.radians(self.current_yaw - 90)  # 90 degrees left
            self.current_x += step_size * math.cos(angle_rad)
            self.current_y += step_size * math.sin(angle_rad)
            print(f"Wall following - blocked front and left, turning left: ({self.current_x:.1f}, {self.current_y:.1f})")
        elif not self.lidar.check_right_obstacles() and not self.lidar.check_front_obstacles():
            # Clear path to goal possible
            print("Path to goal clear, switching back to GO_TO_GOAL mode.")
            self.state = "GO_TO_GOAL"
        else:
            # Move forward whilst wall following
            angle_rad = math.radians(self.current_yaw)
            self.current_x += step_size * math.cos(angle_rad)
            self.current_y += step_size * math.sin(angle_rad)
            print(f"Wall following - moving forward: ({self.current_x:.1f}, {self.current_y:.1f})")
        
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(self.current_x, self.current_y, self.altitude, self.current_yaw)
        )

    async def follow_right_wall(self, step_size=0.2):
        """Simplified right wall following"""
        
        # Priority order for wall following:
        # 1. If right is clear, turn right and move
        # 2. If front is clear, move forward  
        # 3. If front is blocked, turn left
        
        if not self.lidar.check_right_obstacles():
            # Right is clear - turn right and move (stay close to wall)
            self.current_yaw = (self.current_yaw + 90) % 360
            angle_rad = math.radians(self.current_yaw)
            self.current_x += step_size * math.cos(angle_rad)
            self.current_y += step_size * math.sin(angle_rad)
            print(f"Right clear - turned right and moved to: ({self.current_x:.1f}, {self.current_y:.1f})")
            
        elif not self.lidar.check_front_obstacles():
            # Front is clear - move forward
            angle_rad = math.radians(self.current_yaw)
            self.current_x += step_size * math.cos(angle_rad)
            self.current_y += step_size * math.sin(angle_rad)
            print(f"Front clear - moving forward: ({self.current_x:.1f}, {self.current_y:.1f})")
            
        else:
            # Front is blocked - turn left (don't move)
            self.current_yaw = (self.current_yaw - 90) % 360
            print(f"Front blocked - turning left to: {self.current_yaw:.0f}°")
        
        # Check if we can switch back to goal mode
        if not self.lidar.check_front_obstacles():
            current_distance = self.distance_to_goal()
            if current_distance < self.closest_distance_to_goal:
                print("Closer to goal - switching to GO_TO_GOAL")
                self.state = "GO_TO_GOAL"
                return
        
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(self.current_x, self.current_y, self.altitude, self.current_yaw)
        )

    async def run_bug_algorithm(self):
        """Main Bug algorithm loop"""
        step_size = 0.2
        
        while not self.at_goal():
            if self.state == "GO_TO_GOAL":
                # Check if path to goal is blocked
                if self.lidar.check_front_obstacles():
                    print("Obstacle detected! Switching to wall following mode.")
                    self.state = "WALL_FOLLOW"
                    self.wall_follow_start_x = self.current_x
                    self.wall_follow_start_y = self.current_y
                    self.closest_distance_to_goal = self.distance_to_goal()
                else:
                    # Move toward goal
                    await self.move_toward_goal(step_size)
            
            elif self.state == "WALL_FOLLOW":
                # Follow the wall
                await self.follow_left_wall(step_size)
                
                # Check if we should switch back to go-to-goal mode
                current_distance = self.distance_to_goal()
                
                # Update closest distance to goal while wall following
                if current_distance < self.closest_distance_to_goal:
                    self.closest_distance_to_goal = current_distance
                
                # Check if we can head toward goal again
                if not self.lidar.check_front_obstacles():
                    # Check if heading toward goal would be better than current position
                    goal_angle = self.angle_to_goal()
                    angle_diff = abs(goal_angle - self.current_yaw)
                    
                    if angle_diff > 90 and current_distance < self.closest_distance_to_goal + 1.0:
                        print("Clear path to goal found! Switching back to goal mode.")
                        self.state = "GO_TO_GOAL"
                        self.wall_follow_start_x = None
                        self.wall_follow_start_y = None
                        self.closest_distance_to_goal = float('inf')
            
            await asyncio.sleep(0.2)  # Control loop frequency
        
        print("Goal reached!")

async def run():
    drone = System()
    lidar = SimpleLidarReader()
    await drone.connect(system_address="udp://:14540")

    # Drone connection
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Wait for global position estimate
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    # Start the LiDAR
    print("Starting LiDAR...")
    if not lidar.start_listening():
        print("Failed to start LiDAR, aborting mission.")
        return
    
    # Wait for initial LiDAR data
    print("Waiting for LiDAR data...")
    while lidar.latest_scan is None:
        await asyncio.sleep(0.1)
    print("LiDAR data received.")

    # Arm the drone
    print("Arming drone...")
    await drone.action.arm()

    # Set the initial setpoint at current position
    print("Setting initial position setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 90.0))

    # Start offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    # Take off
    print("Taking off to 5 meters...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 90))
    await asyncio.sleep(10)

    # Set goal position (10 meters forward, 5 meters right)
    goal_x = 0.0
    goal_y = 20.0
    
    print(f"Starting Bug algorithm to reach goal: ({goal_x}, {goal_y})")
    
    # Run Bug algorithm
    bug = BugAlgorithm(drone, lidar, goal_x, goal_y)
    await bug.run_bug_algorithm()

    # Hold position at goal
    print("Holding position at goal...")
    await asyncio.sleep(5)

    # Return to start and land
    print("Returning to start position...")
    await drone.offboard.set_position_ned(PositionNedYaw(goal_x, goal_y, -1.0, 90.0))
    await asyncio.sleep(10)

    print("Landing...")
    await drone.offboard.set_position_ned(PositionNedYaw(goal_x, goal_y, 0.0, 90.0))
    await asyncio.sleep(10)

    print("Stopping offboard mode...")
    await drone.offboard.stop()

    await drone.action.land()
    print("Mission complete!")

if __name__ == "__main__":
    asyncio.run(run())