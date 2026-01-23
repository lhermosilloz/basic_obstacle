#!/usr/bin/env python3
import os
# Set protobuf implementation to python before importing gz modules
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
import asyncio
import time
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from Bug.vertical_bug_algorithm import VerticalBugAlgorithm

async def run():
    # Set target coordinates (modify these as needed)
    TARGET_X = 0.0  # meters
    TARGET_Y = 40.0  # meters
    TARGET_ALTITUDE = -3.0  # meters (negative in NED)
    
    bug = VerticalBugAlgorithm(TARGET_X, TARGET_Y, TARGET_ALTITUDE)
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

    # Takeoff to target altitude
    print(f"Taking off to {TARGET_ALTITUDE}m...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, TARGET_ALTITUDE, 0.0))
    await asyncio.sleep(8)

    # Have a loop that allows user to send target positions
    print("Navigating to target using Vertical Bug Algorithm...")
    while True:
        user_input = input("Press Enter to start navigation to target or type 'exit' to quit: ")
        if user_input.lower() == 'exit':
            break
        # Navigate to target using Vertical Bug Algorithm
        await bug.navigate_to_target()
        # Get new target from user
        try:
            new_x = float(input("Enter new target X (meters): "))
            new_y = float(input("Enter new target Y (meters): "))
            bug.target_x = new_x
            bug.target_y = new_y
            print(f"New target set to X: {new_x}m, Y: {new_y}m")
        except ValueError:
            print("Invalid input. Please enter numeric values for X and Y.")
    
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