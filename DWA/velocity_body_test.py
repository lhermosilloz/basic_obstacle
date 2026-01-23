#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw, VelocityBodyYawspeed

async def run():
    drone = System()
    print("Connecting to drone...")
    await drone.connect(system_address="udp://:14540")  # Adjust for your connection

    # Wait until connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Wait for global position estimate
    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    # Arm the drone
    print("-- Arming")
    await drone.action.arm()

    # Set the initial setpoint at current position (0,0,0)
    print("Setting initial position setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    # Start offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    # Now command takeoff to 2 meter
    print("Taking off to 2 meter...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(10)  # Give time to reach altitude

    print("-- Flying forward at 1 m/s for 5 seconds")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_m_s=1.0, right_m_s=0.0, down_m_s=0.0, yawspeed_deg_s=0.0)
    )
    await asyncio.sleep(5)

    print("-- Moving right at 1 m/s for 5 seconds")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_m_s=0.0, right_m_s=1.0, down_m_s=0.0, yawspeed_deg_s=0.0)
    )
    await asyncio.sleep(5)

    print("-- Yawing at 30 deg/s for 2 seconds")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yawspeed_deg_s=30.0)
    )
    await asyncio.sleep(2)

    # Stop offboard mode
    print("-- Stopping offboard mode")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed: {error._result.result}")

    # Land
    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
