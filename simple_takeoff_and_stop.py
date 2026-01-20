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

class ObstacleAvoidanceDrone:
    def __init__(self):
        self.drone = System()
        self.lidar = SimpleLidarReader()
        self.obstacle_detected = False
        self.min_safe_distance = 2.0  # meters
        
    def check_front_obstacle(self):
        """Check if there's an obstacle in front of the drone"""
        if self.lidar.latest_scan is None:
            return False
            
        msg = self.lidar.latest_scan
        angle_min = msg.angle_min
        angle_increment = msg.angle_step
        
        # Check front sector (±30 degrees)
        front_distances = []
        
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            angle = angle_min + (i * angle_increment)
            angle_deg = math.degrees(angle)
            
            # Check if this point is in front sector
            if abs(angle_deg) <= 30:  # ±30 degrees front
                front_distances.append(range_val)
        
        if front_distances:
            min_front_distance = min(front_distances)
            print(f"Minimum front distance: {min_front_distance:.2f}m")
            return min_front_distance < self.min_safe_distance
        
        return False

async def run():
    controller = ObstacleAvoidanceDrone()
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

    # Move forward until obstacle detected
    print("Starting forward movement...")
    forward_distance = 0.0
    step_size = 0.1  # Move 0.1 meter at a time
    
    while not controller.obstacle_detected:
        # Check for obstacles
        if controller.check_front_obstacle():
            print("OBSTACLE DETECTED! Stopping forward movement.")
            controller.obstacle_detected = True
            break
            
        # Move forward by step_size
        forward_distance += step_size
        print(f"Moving forward to {forward_distance}m...")
        await drone.offboard.set_position_ned(PositionNedYaw(forward_distance, 0.0, -3.0, 0.0))
        
        # Wait and check again
        await asyncio.sleep(0.1)
        
        # Safety check - don't go too far
        if forward_distance >= 50.0:
            print("Maximum distance reached, stopping...")
            break
    
    # Hold position briefly
    print("Holding position for safety...")
    await asyncio.sleep(3)

    # Land safely
    print("Landing safely...")
    await drone.offboard.set_position_ned(PositionNedYaw(forward_distance, 0.0, 0.0, 0.0))
    await asyncio.sleep(8)  # Give time to descend

    print("Stopping offboard mode...")
    await drone.offboard.stop()

    print("Final landing command...")
    await drone.action.land()

    print("Mission complete! Obstacle avoidance successful.")

if __name__ == "__main__":
    asyncio.run(run())