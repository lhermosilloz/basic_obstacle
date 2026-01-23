#!/usr/bin/env python3

"""
Debug script for PID controller - helps identify telemetry and control issues.
"""

import asyncio
import time
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from PID.pid_controller import PIDController

async def test_telemetry(drone):
    """Test all available telemetry sources."""
    print("=== TELEMETRY TEST ===")
    
    # Test position telemetry
    print("Testing position telemetry...")
    try:
        async for position_ned in drone.telemetry.position_velocity_ned():
            print(f"Position NED: North={position_ned.position.north_m:.2f}, "
                  f"East={position_ned.position.east_m:.2f}, "
                  f"Down={position_ned.position.down_m:.2f}")
            break
    except Exception as e:
        print(f"Position telemetry error: {e}")
    
    # Test attitude telemetry options
    print("Testing attitude telemetry sources...")
    
    # Try attitude_euler
    try:
        async for attitude_euler in drone.telemetry.attitude_euler():
            print(f"Attitude Euler: Roll={attitude_euler.roll_deg:.1f}°, "
                  f"Pitch={attitude_euler.pitch_deg:.1f}°, "
                  f"Yaw={attitude_euler.yaw_deg:.1f}°")
            break
    except Exception as e:
        print(f"Attitude Euler error: {e}")
    
    # Try attitude_quaternion
    try:
        async for attitude_quat in drone.telemetry.attitude_quaternion():
            # Convert to Euler for display
            w, x, y, z = attitude_quat.w, attitude_quat.x, attitude_quat.y, attitude_quat.z
            yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            yaw_deg = math.degrees(yaw_rad)
            print(f"Attitude Quaternion: w={w:.3f}, x={x:.3f}, y={y:.3f}, z={z:.3f}")
            print(f"  Converted Yaw: {yaw_deg:.1f}°")
            break
    except Exception as e:
        print(f"Attitude Quaternion error: {e}")
    
    # Try heading
    try:
        async for heading in drone.telemetry.heading():
            print(f"Heading: {heading.heading_deg:.1f}°")
            break
    except Exception as e:
        print(f"Heading error: {e}")
    
    print("Telemetry test completed.\n")

async def test_basic_movement(drone):
    """Test basic position commands without PID."""
    print("=== BASIC MOVEMENT TEST ===")
    
    print("Testing basic position command...")
    
    # Get initial position
    try:
        async for position_ned in drone.telemetry.position_velocity_ned():
            start_x = position_ned.position.north_m
            start_y = position_ned.position.east_m
            start_z = position_ned.position.down_m
            print(f"Starting position: ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})")
            break
    except Exception as e:
        print(f"Could not get starting position: {e}")
        start_x, start_y, start_z = 0.0, 0.0, 0.0
    
    # Command a small movement
    target_x = start_x + 1.0
    target_y = start_y + 1.0
    target_z = start_z - 1.0  # Move up 1 meter
    
    print(f"Commanding move to: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
    
    await drone.offboard.set_position_ned(PositionNedYaw(target_x, target_y, target_z, 0.0))
    
    # Monitor movement for 10 seconds
    for i in range(10):
        await asyncio.sleep(1)
        try:
            async for position_ned in drone.telemetry.position_velocity_ned():
                current_x = position_ned.position.north_m
                current_y = position_ned.position.east_m
                current_z = position_ned.position.down_m
                
                distance = math.sqrt((target_x - current_x)**2 + 
                                   (target_y - current_y)**2 + 
                                   (target_z - current_z)**2)
                
                print(f"T+{i+1}s: Pos=({current_x:.2f}, {current_y:.2f}, {current_z:.2f}), "
                      f"Distance to target={distance:.2f}m")
                break
        except Exception as e:
            print(f"Position update error: {e}")
    
    print("Basic movement test completed.\n")

async def test_pid_step_by_step(drone):
    """Test PID controller with detailed debugging."""
    print("=== PID STEP-BY-STEP TEST ===")
    
    # Create PID controller with conservative settings
    pid = PIDController(
        drone,
        kp_linear=0.5,      # Very conservative
        ki_linear=0.01,     # Minimal integral
        kd_linear=0.3,      # Light damping
        kp_angular=1.0,
        ki_angular=0.01,
        kd_angular=0.5,
        max_velocity=0.8,   # Slow for safety
        position_tolerance=0.3
    )
    
    print("PID Controller created with conservative settings")
    
    # Test telemetry update
    print("Testing telemetry update...")
    await pid.update_current_state()
    print(f"Current state: Pos=({pid.current_x:.2f}, {pid.current_y:.2f}, {pid.current_z:.2f}), "
          f"Yaw={pid.current_yaw:.1f}°")
    
    # Set a nearby target
    target_x = pid.current_x + 2.0
    target_y = pid.current_y + 2.0
    target_z = pid.current_z - 1.0
    target_yaw = 45.0
    
    print(f"Setting target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), Yaw={target_yaw:.1f}°")
    pid.set_waypoint(target_x, target_y, target_z, target_yaw)
    
    # Run a few PID iterations manually
    for i in range(5):
        print(f"\n--- PID Iteration {i+1} ---")
        
        # Update state
        await pid.update_current_state()
        print(f"Current: ({pid.current_x:.2f}, {pid.current_y:.2f}, {pid.current_z:.2f}), "
              f"Yaw={pid.current_yaw:.1f}°")
        
        # Compute PID
        vel_x, vel_y, vel_z, yaw_rate = pid.compute_control_commands()
        print(f"PID Output: Vel=({vel_x:.2f}, {vel_y:.2f}, {vel_z:.2f}), "
              f"YawRate={yaw_rate:.2f}°/s")
        
        # Check if at waypoint
        if pid.at_waypoint():
            print("Waypoint reached!")
            break
        
        distance = pid.get_distance_to_waypoint()
        print(f"Distance to waypoint: {distance:.2f}m")
        
        # Apply control (simplified)
        new_x = pid.current_x + vel_x * 0.1
        new_y = pid.current_y + vel_y * 0.1
        new_z = pid.current_z + vel_z * 0.1
        new_yaw = (pid.current_yaw + yaw_rate * 0.1) % 360
        
        print(f"Commanding: ({new_x:.2f}, {new_y:.2f}, {new_z:.2f}), "
              f"Yaw={new_yaw:.1f}°")
        
        await drone.offboard.set_position_ned(PositionNedYaw(new_x, new_y, new_z, new_yaw))
        await asyncio.sleep(1)
    
    print("PID test completed.\n")

async def run_debug_session():
    """Run complete debug session."""
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

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    await drone.offboard.start()
    
    # Wait a moment for offboard to stabilize
    await asyncio.sleep(2)
    
    # Run tests
    await test_telemetry(drone)
    await test_basic_movement(drone)
    await test_pid_step_by_step(drone)
    
    print("=== DEBUG SESSION COMPLETE ===")
    print("If you see telemetry errors above, that's the root cause.")
    print("If basic movement works but PID doesn't, check PID parameters.")
    
    # Land
    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run_debug_session())