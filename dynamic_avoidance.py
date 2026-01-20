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
        self.move_step = 0.001  # meters to move per step

    def check_where_obstacle(self):
        """Check where the obstacles are relative to the drone. Return a list of directions with close enough obstacles for drone to respond."""
        direction_distances = []
        if self.lidar.latest_scan is None:
            return direction_distances
        
        # Get message from LiDAR
        msg = self.lidar.latest_scan
        angle_min = msg.angle_min
        angle_increment = msg.angle_step

        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue

            angle = angle_min + (i * angle_increment)
            angle_deg = math.degrees(angle)

            # Only consider obstacles within safe distance
            if range_val < self.min_safe_distance:
                direction_distances.append((angle_deg, range_val))
        
        # Sort by distance (closest first) and take the 2 closest
        direction_distances.sort(key=lambda x: x[1])
        return direction_distances[:2] if len(direction_distances) >= 2 else direction_distances

    def calculate_scissor_escape_direction(self, obstacles):
        """Calculate the escape direction using scissor effect"""
        if len(obstacles) < 2:
            if len(obstacles) == 1:
                # Single obstacle - move directly opposite
                obstacle_angle = obstacles[0][0]
                escape_angle = (obstacle_angle + 180) % 360
                if escape_angle > 180:
                    escape_angle -= 360
                return escape_angle
            else:
                # No obstacles - stay in place
                return None
        
        angle1, dist1 = obstacles[0]
        angle2, dist2 = obstacles[1]
        
        # Calculate the "blade" directions (opposite to obstacles)
        blade1_angle = (angle1 + 180) % 360
        blade2_angle = (angle2 + 180) % 360
        
        # Normalize to -180 to 180
        if blade1_angle > 180:
            blade1_angle -= 360
        if blade2_angle > 180:
            blade2_angle -= 360
        
        # Calculate the angle difference
        angle_diff = abs(blade1_angle - blade2_angle)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        # Calculate the midpoint (escape direction)
        if angle_diff <= 180:
            # Direct midpoint calculation
            escape_angle = (blade1_angle + blade2_angle) / 2
            
            # Handle wrap-around case
            if abs(blade1_angle - blade2_angle) > 180:
                escape_angle = (escape_angle + 180) % 360
                if escape_angle > 180:
                    escape_angle -= 360
        else:
            # This shouldn't happen with the above logic, but just in case
            escape_angle = (blade1_angle + blade2_angle) / 2
        
        return escape_angle

    def angle_to_movement(self, angle_deg):
        """Convert angle in degrees to x,y movement components"""
        if angle_deg is None:
            return 0.0, 0.0
            
        angle_rad = math.radians(angle_deg)
        # In NED coordinate system:
        # 0° = North (positive X)
        # 90° = East (positive Y)
        # 180° = South (negative X)
        # 270° = West (negative Y)
        dx = self.move_step * math.cos(angle_rad)
        dy = -self.move_step * math.sin(angle_rad)
        return dx, dy

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

    # Hover position
    hover_x = 0.0
    hover_y = 0.0

    print("Starting scissor-based obstacle avoidance...")
    
    while True:
        # Check for obstacles
        obstacles = controller.check_where_obstacle()
        
        if len(obstacles) >= 1:
            print(f"\nDetected {len(obstacles)} obstacle(s)")
            
            # Calculate escape direction using scissor effect
            escape_angle = controller.calculate_scissor_escape_direction(obstacles)
            
            if escape_angle is not None:
                # Convert angle to movement
                dx, dy = controller.angle_to_movement(escape_angle)
                
                # Update hover position
                hover_x += dx
                hover_y += dy
                
                # Command the movement
                await drone.offboard.set_position_ned(PositionNedYaw(hover_x, hover_y, -2.0, 0.0))
            else:
                print("No clear escape direction calculated")
        else:
            # No obstacles detected - maintain position
            await drone.offboard.set_position_ned(PositionNedYaw(hover_x, hover_y, -2.0, 0.0))
        
        await asyncio.sleep(0.001)  # 10Hz update rate

if __name__ == "__main__":
    asyncio.run(run())