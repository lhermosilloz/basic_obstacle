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

    # Get current position to use as initial setpoint
    print("Getting current position...")
    async for position in drone.telemetry.position_velocity_ned():
        current_north = position.position.north_m
        current_east = position.position.east_m
        current_down = position.position.down_m
        print(f"Current position: N={current_north:.2f}, E={current_east:.2f}, D={current_down:.2f}")
        break

    # Set the initial setpoint to current position (prevents takeoff)
    print("Setting initial position setpoint to current position...")
    await drone.offboard.set_position_ned(PositionNedYaw(current_north, current_east, current_down, 0.0))

    # Start offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()
    
    # Arm the drone
    print("Arming drone...")
    await drone.action.arm()

    # Hold current position
    print("Holding current position...")
    await asyncio.sleep(6)

    print("Landing...")
    await drone.action.land()
    
    # Wait for landing to complete by monitoring flight mode and armed state
    print("Waiting for landing to complete...")
    async for armed in drone.telemetry.armed():
        if not armed:
            print("Drone has landed and disarmed automatically!")
            break

    print("Stopping offboard mode...")
    await drone.offboard.stop()

    print("Disarm")
    await drone.action.disarm()

    print("Mission complete!")

if __name__ == "__main__":
    asyncio.run(run())