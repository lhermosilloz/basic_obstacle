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
        self.position_task = None
        
    async def connect(self):
        """Connect to PX4"""
        await self.drone.connect(system_address="udp://:14540")
        
        # Wait for connection
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Global position estimate OK")
                break

        print("Setting initial position setpoint before arming...")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))
        
        # Start position monitoring
        self.position_task = asyncio.create_task(self.get_sensor_data())
            
    async def get_sensor_data(self):
        """Subscribe to sensor data streams"""
        
        # Subscribe to position
        async for position in self.drone.telemetry.position():
            self.current_position = position
            
    async def simple_avoidance_algorithm(self):
        """Simple potential field-based obstacle avoidance using NED coordinates"""
        
        if not self.current_position or not self.target_position:
            return None
            
        # Use NED coordinates for calculations
        current_ned = np.array([0.0, 0.0, 0.0])  # Current position as origin
        target_ned = np.array(self.target_position)  # Target in NED
        
        # Calculate attractive force toward target
        target_vector = target_ned - current_ned
        target_distance = np.linalg.norm(target_vector)
        
        if target_distance < 0.1:  # Very close to target
            return np.array([0.0, 0.0, 0.0])
            
        attractive_force = target_vector / target_distance
        
        # Calculate repulsive forces from obstacles
        repulsive_force = np.array([0.0, 0.0, 0.0])
        
        for obstacle in self.obstacles:
            obstacle_vector = current_ned - np.array(obstacle)
            distance = np.linalg.norm(obstacle_vector)
            
            if distance < 5.0 and distance > 0.1:  # 5 meter safety distance
                repulsive_force += (obstacle_vector / distance) * (1.0 / distance)
                
        # Combine forces
        total_force = attractive_force + repulsive_force * 2.0  # Weight repulsive higher
        
        if np.linalg.norm(total_force) > 0:
            return total_force / np.linalg.norm(total_force)
        else:
            return np.array([0.0, 0.0, 0.0])
        
    async def send_velocity_command(self, direction_vector, speed=2.0):
        """Send velocity commands to drone"""
        
        velocity = VelocityBodyYawspeed(
            forward_m_s=direction_vector[0] * speed,
            right_m_s=direction_vector[1] * speed,
            down_m_s=direction_vector[2] * speed,  # Positive for down in NED
            yawspeed_deg_s=0.0
        )
        
        await self.drone.offboard.set_velocity_body(velocity)
        
    async def run_mission(self, target_north, target_east, target_down):
        """Main mission loop with NED coordinates"""
        
        self.target_position = (target_north, target_east, target_down)
        
        # Add some example obstacles (in NED coordinates)
        self.obstacles = [
            [5.0, 5.0, -10.0],   # Obstacle at 5m north, 5m east, 10m altitude
            [10.0, -3.0, -10.0]  # Another obstacle
        ]

        # Set initial velocity setpoint before starting offboard mode
        print("Setting initial velocity setpoint...")
        initial_velocity = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        await self.drone.offboard.set_velocity_body(initial_velocity)
    

        print("Starting offboard mode...")
        await self.drone.offboard.start()
        
        print("Arming drone...")
        await self.drone.action.arm()

        print("Taking off...")
        await self.drone.action.takeoff()
        await asyncio.sleep(5)  # Wait for takeoff to complete
        
        print("Starting obstacle avoidance mission...")
        
        mission_timeout = 60  # 60 second timeout
        start_time = asyncio.get_event_loop().time()
        
        while True:
            # Check timeout
            if asyncio.get_event_loop().time() - start_time > mission_timeout:
                print("Mission timeout!")
                break
                
            # Get avoidance direction
            direction = await self.simple_avoidance_algorithm()
            
            if direction is not None:
                await self.send_velocity_command(direction)
            else:
                # Send zero velocity if no direction calculated
                await self.send_velocity_command(np.array([0.0, 0.0, 0.0]))
                
            # Check if reached target (using NED coordinates)
            if self.target_position:
                distance_to_target = math.sqrt(
                    self.target_position[0]**2 + 
                    self.target_position[1]**2 +
                    self.target_position[2]**2
                )
                
                if distance_to_target < 2.0:  # 2 meter tolerance
                    print("Target reached!")
                    break
                    
            await asyncio.sleep(0.1)  # 10 Hz control loop
            
        # Stop offboard and land
        print("Stopping offboard mode...")
        await self.drone.offboard.stop()
        
        print("Landing...")
        await self.drone.action.land()

async def main():
    avoidance = ObstacleAvoidance()
    await avoidance.connect()
    
    # Wait a bit for position data to be available
    await asyncio.sleep(2)
    
    # Example: fly to target position (North, East, Down in meters)
    await avoidance.run_mission(-20.0, 20.0, -10.0)
    
    # Cancel position monitoring task
    if avoidance.position_task:
        avoidance.position_task.cancel()

if __name__ == "__main__":
    asyncio.run(main())