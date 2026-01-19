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

    # Set the initial setpoint before starting offboard
    print("Setting initial position setpoint before arming...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 0.0))

    # Start the offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    # Arm the drone
    print("Arming drone...")
    await drone.action.arm()

    # Takeoff
    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(5)  # Wait for takeoff to complete

    print("Stopping offboard mode...")
    await drone.offboard.stop()

    print("Landing...")
    await drone.action.land()

    # Wait for landing to complete
    print("Waiting for landing...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed!")
            break

    print("Mission complete!")

if __name__ == "__main__":
    asyncio.run(run())