#!/usr/bin/env python3

"""
Example usage of the PID Controller for drone navigation.
This demonstrates how to use the PID controller to smoothly navigate to waypoints.
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from PID.pid_controller import PIDController

async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    # Initialize PID controller with custom gains for smooth movement
    pid_controller = PIDController(
        drone,
        kp_linear=1.0,      # Moderate proportional gain for smooth response
        ki_linear=0.05,     # Small integral gain to eliminate steady-state error
        kd_linear=0.5,      # Damping to reduce oscillations
        kp_angular=1.5,     # Yaw control
        ki_angular=0.02,
        kd_angular=0.8,
        max_velocity=1.5,   # Limit maximum velocity for safety
        position_tolerance=0.3,  # 30cm tolerance
        yaw_tolerance=10.0  # 10 degree yaw tolerance
    )

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    await drone.offboard.start()

    # Takeoff to 3 meters
    print("Taking off to 3 meters...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(8)  # Give time to reach altitude

    # Example 1: Single waypoint navigation
    print("\n=== Example 1: Single Waypoint Navigation ===")
    
    # Takeoff to 5 meters
    print("Taking off to 5 meters altitude...")
    success = await pid_controller.go_to_waypoint(0.0, 0.0, -5.0, 0.0, timeout=20.0)
    if success:
        print("Takeoff completed successfully!")
        await asyncio.sleep(2)  # Hover for 2 seconds
    else:
        print("Takeoff failed!")
        return

    # Navigate to a few waypoints with smooth transitions
    print("\nNavigating to waypoint (10, 0, -5)...")
    await pid_controller.go_to_waypoint(10.0, 0.0, -5.0, 90.0, timeout=30.0)
    await asyncio.sleep(2)

    print("\nNavigating to waypoint (10, 10, -5)...")
    await pid_controller.go_to_waypoint(10.0, 10.0, -5.0, 180.0, timeout=30.0)
    await asyncio.sleep(2)

    print("\nNavigating to waypoint (0, 10, -5)...")
    await pid_controller.go_to_waypoint(0.0, 10.0, -5.0, 270.0, timeout=30.0)
    await asyncio.sleep(2)

    print("\nReturning home...")
    await pid_controller.go_to_waypoint(0.0, 0.0, -5.0, 0.0, timeout=30.0)
    await asyncio.sleep(2)

    # Example 2: Waypoint sequence navigation
    print("\n=== Example 2: Waypoint Sequence Navigation ===")
    
    # Define a sequence of waypoints (x, y, z, yaw)
    waypoints = [
        (5.0, 5.0, -8.0, 45.0),    # Diagonal move, climb to 8m
        (-5.0, 5.0, -8.0, 135.0),  # Move west
        (-5.0, -5.0, -8.0, 225.0), # Move south  
        (5.0, -5.0, -8.0, 315.0),  # Move east
        (0.0, 0.0, -8.0, 0.0),     # Return to center
        (0.0, 0.0, -2.0, 0.0),     # Descend for landing
    ]
    
    print("Following waypoint sequence...")
    success = await pid_controller.follow_waypoint_sequence(waypoints, timeout_per_waypoint=25.0)
    
    if success:
        print("Waypoint sequence completed successfully!")
    else:
        print("Waypoint sequence failed!")

    # Land
    print("\n-- Landing")
    await drone.action.land()

    print("-- Mission completed!")

if __name__ == "__main__":
    asyncio.run(run())