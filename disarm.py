#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14551")

    # Drone connection
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Check current flight mode
    print("Getting current flight mode...")
    async for flight_mode in drone.telemetry.flight_mode():
        print(f"Current flight mode: {flight_mode}")
        break

    # Arm the drone
    print("Arming drone...")
    await drone.action.disarm()

    # Hold current position
    print("Holding current position...")
    await asyncio.sleep(5)

if __name__ == "__main__":
    asyncio.run(run())