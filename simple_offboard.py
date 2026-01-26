#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

async def run():
    drone = System()
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

    # Arm the drone first
    print("Arming drone...")
    await drone.action.arm()

    # Set the initial setpoint at current position (0,0,0)
    print("Setting initial position setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    # Start offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    # Now command takeoff to 10 meters
    print("Taking off to 10 meters...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(10)  # Give time to reach altitude

    # Hold position for a few seconds
    print("Holding position...")
    await asyncio.sleep(5)

    # Command landing
    print("Commanding descent...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(10)  # Give time to descend

    print("Stopping offboard mode...")
    await drone.offboard.stop()

    print("Landing...")
    await drone.action.land()

    print("Mission complete!")

if __name__ == "__main__":
    asyncio.run(run())