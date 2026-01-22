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

    # Arm the drone first
    print("Arming drone...")
    await drone.action.arm()

    # Set the initial setpoint at current position (0,0,0)
    print("Setting initial position setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 90.0))

    # Start offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    # Now command takeoff to 5 meters
    print("Taking off to 5 meters...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 90))
    await asyncio.sleep(10)  # Give time to reach altitude

    # Main wall following logic here. Abstracting to a class later

    # Move forward until obstacle detected
    print("Starting forward movement...")
    forward_distance = 0.0
    step_size = 0.1  # Move 1 meter at a time

    while not lidar.check_front_obstacles():
        # Move forward by step_size
        forward_distance += step_size
        print(f"Moving forward to {forward_distance:.1f} meters")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, forward_distance, -5.0, 90.0))
        await asyncio.sleep(0.1)

    # Begin wall following behavior
    print("OBSTACLE DETECTED! Stopping forward movement.")
    strafe_distance = 0.0
    strafe_size = 0.1  # Strafe 0.1 meter at a time

    while lidar.check_front_obstacles() and not lidar.check_left_obstacles():
        # Move left/right by strafe_size
        strafe_distance += strafe_size
        print(f"Strafing right to {strafe_distance:.1f} meters")
        await drone.offboard.set_position_ned(PositionNedYaw(strafe_distance, forward_distance, -5.0, 90.0))
        await asyncio.sleep(0.1)

    # Hold position for a few seconds
    print("Holding position...")
    await asyncio.sleep(3)

    # Command landing
    print("Commanding descent...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 90.0))
    await asyncio.sleep(10)  # Give time to descend

    print("Stopping offboard mode...")
    await drone.offboard.stop()

    print("Landing...")
    await drone.action.land()

    print("Mission complete!")

if __name__ == "__main__":
    asyncio.run(run())