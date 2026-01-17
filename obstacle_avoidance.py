#!/usr/bin/env python3

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        self.drone = System()
        self.current_position = None
        self.obstacles = []
        self.target_position = None
        self.avoidance_active = False
        
    async def connect(self):
        """Connect to PX4"""
        await self.drone.connect(system_address="udp://:14540")
        
        # Wait for connection
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break
                
    async def get_sensor_data(self):
        """Subscribe to sensor data streams"""
        
        # Subscribe to position
        async for position in self.drone.telemetry.position():
            self.current_position = position
            
        # Subscribe to lidar data (if available)
        # Note: You'll need to implement lidar data parsing
        
    async def simple_avoidance_algorithm(self):
        """Simple potential field-based obstacle avoidance"""
        
        if not self.current_position or not self.target_position:
            return None
            
        # Calculate attractive force toward target
        target_vector = np.array([
            self.target_position[0] - self.current_position.latitude_deg,
            self.target_position[1] - self.current_position.longitude_deg,
            self.target_position[2] - self.current_position.relative_altitude_m
        ])
        
        attractive_force = target_vector / np.linalg.norm(target_vector)
        
        # Calculate repulsive forces from obstacles
        repulsive_force = np.array([0.0, 0.0, 0.0])
        
        for obstacle in self.obstacles:
            obstacle_vector = np.array([
                self.current_position.latitude_deg - obstacle[0],
                self.current_position.longitude_deg - obstacle[1], 
                self.current_position.relative_altitude_m - obstacle[2]
            ])
            
            distance = np.linalg.norm(obstacle_vector)
            
            if distance < 5.0:  # 5 meter safety distance
                repulsive_force += (obstacle_vector / distance) * (1.0 / distance)
                
        # Combine forces
        total_force = attractive_force + repulsive_force * 2.0  # Weight repulsive higher
        
        return total_force / np.linalg.norm(total_force)
        
    async def send_velocity_command(self, direction_vector, speed=2.0):
        """Send velocity commands to drone"""
        
        velocity = VelocityBodyYawspeed(
            forward_m_s=direction_vector[0] * speed,
            right_m_s=direction_vector[1] * speed,
            down_m_s=-direction_vector[2] * speed,  # Negative for up
            yawspeed_deg_s=0.0
        )
        
        await self.drone.offboard.set_velocity_body(velocity)
        
    async def run_mission(self, target_lat, target_lon, target_alt):
        """Main mission loop"""
        
        self.target_position = (target_lat, target_lon, target_alt)
        
        # Start offboard mode
        await self.drone.offboard.start()
        
        print("Starting obstacle avoidance mission...")
        
        while True:
            # Get avoidance direction
            direction = await self.simple_avoidance_algorithm()
            
            if direction is not None:
                await self.send_velocity_command(direction)
                
            # Check if reached target
            if self.current_position:
                distance_to_target = math.sqrt(
                    (self.target_position[0] - self.current_position.latitude_deg)**2 +
                    (self.target_position[1] - self.current_position.longitude_deg)**2
                )
                
                if distance_to_target < 0.0001:  # Roughly 10 meters
                    print("Target reached!")
                    break
                    
            await asyncio.sleep(0.1)  # 10 Hz control loop

async def main():
    avoidance = ObstacleAvoidance()
    await avoidance.connect()
    
    # Example: fly to a target position
    await avoidance.run_mission(47.398, 8.546, 10.0)

if __name__ == "__main__":
    asyncio.run(main())