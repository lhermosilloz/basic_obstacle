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
        self.drone = System()
        self.lidar = SimpleLidarReader()
        self.obstacle_detected = False
        self.min_safe_distance = 0.5  # meters

    def check_where_obstacle(self):
        """Check where the obstacles are relative to the drone. Return a list of directions with close enough obstacles for drone to respond."""
        direction_distances = []
        if self.lidar.latest_scan is None:
            return direction_distances
        
        # Get message from LiDAR
        msg = self.lidar.latest_scan
        angle_min = msg.angle_min
        angle_increment = msg.angle_step

        # Note the closest obstacle distance angle

        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue

            angle = angle_min + (i * angle_increment)
            angle_deg = math.degrees(angle)

            direction_distances.append((angle_deg, range_val))
        
        # Move away from the closest obstacle, so return it
        min_distance = float('inf')
        closest_angle = None
        for angle_deg, distance in direction_distances:
            if distance < min_distance:
                min_distance = distance
                closest_angle = angle_deg

        return closest_angle, min_distance

async def run():
    controller = DynamicObstacleAvoidanceDrone()
    drone = controller.drone

    # Connect to drone
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate ok")
            break
    
    # Start LiDAR reader
    print("Starting LiDAR reader...")
    if not controller.lidar.start_listening():
        print("Failed to start LiDAR reader")
        return
    
    # Wait for initial LiDAR data
    print("Waiting for LiDAR data...")
    while controller.lidar.latest_scan is None:
        await asyncio.sleep(0.1)
    print("LiDAR data received.")

    # Arm the drone
    print("Arming drone...")
    await drone.action.arm()

    # Set initial position
    print("Setting initial position setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    # Start offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    # Takeoff to 2 meters
    print("Taking off to 2 meters...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(8)  # Give time to reach altitude

    # Hover X and Y
    hover_x = 0.0
    hover_y = 0.0

    while True:
        # Check for obstacles
        closest_angle, min_distance = controller.check_where_obstacle()
        if closest_angle is not None and min_distance < controller.min_safe_distance:
            print(f"Obstacle detected at angle {closest_angle:.2f}° with distance {min_distance:.2f}m")
            # Move away from obstacle
            if -45 <= closest_angle <= 45:
                # Obstacle in front, move backward
                print("Moving backward to avoid obstacle")
                hover_x -= 0.001  
            elif 45 < closest_angle <= 135:
                # Obstacle on right, move left
                print("Moving left to avoid obstacle")
                hover_y += 0.001
            elif -135 <= closest_angle < -45:
                # Obstacle on left, move right
                print("Moving right to avoid obstacle")
                hover_y -= 0.001
            else:
                # Obstacle behind, move forward
                print("Moving forward to avoid obstacle")
                hover_x += 0.001
            await drone.offboard.set_position_ned(PositionNedYaw(hover_x, hover_y, -2.0, 0.0))
        else:
            await drone.offboard.set_position_ned(PositionNedYaw(hover_x, hover_y, -2.0, 0.0))
        await asyncio.sleep(0.001)

if __name__ == "__main__":
    asyncio.run(run())