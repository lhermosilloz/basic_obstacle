#!/usr/bin/env python3

"""
PID Controller Tuning Guide and Test Script

This script helps you tune the PID controller parameters for your specific drone
and test different configurations to achieve optimal performance.
"""

import asyncio
import time
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from PID.pid_controller import PIDController

class PIDTuner:
    """Helper class for testing and tuning PID parameters."""
    
    def __init__(self, drone):
        self.drone = drone
        self.test_results = []
    
    async def test_pid_configuration(self, config_name, pid_params, test_waypoint=(5.0, 0.0, -3.0, 0.0)):
        """
        Test a specific PID configuration and record performance metrics.
        
        Args:
            config_name: Name for this configuration
            pid_params: Dictionary with PID parameters
            test_waypoint: Waypoint to test navigation to
        """
        print(f"\n=== Testing Configuration: {config_name} ===")
        print(f"Parameters: {pid_params}")
        
        # Create PID controller with test parameters
        controller = PIDController(self.drone, **pid_params)
        
        # Record start time
        start_time = time.time()
        
        # Navigate to test waypoint
        success = await controller.go_to_waypoint(*test_waypoint, timeout=30.0)
        
        # Record end time
        end_time = time.time()
        completion_time = end_time - start_time
        
        # Calculate final position error
        await controller.update_current_state()
        final_error = controller.get_distance_to_waypoint()
        
        # Record results
        result = {
            'config_name': config_name,
            'success': success,
            'completion_time': completion_time,
            'final_error': final_error,
            'params': pid_params.copy()
        }
        
        self.test_results.append(result)
        
        print(f"Result: {'SUCCESS' if success else 'FAILED'}")
        print(f"Time: {completion_time:.2f}s")
        print(f"Final error: {final_error:.3f}m")
        
        # Return to start position
        await controller.go_to_waypoint(0.0, 0.0, -3.0, 0.0, timeout=15.0)
        await asyncio.sleep(2)
        
        return result
    
    def print_results_summary(self):
        """Print a summary of all test results."""
        print("\n" + "="*60)
        print("PID TUNING RESULTS SUMMARY")
        print("="*60)
        
        # Sort by completion time for successful runs
        successful_results = [r for r in self.test_results if r['success']]
        successful_results.sort(key=lambda x: x['completion_time'])
        
        print("\nSuccessful configurations (sorted by time):")
        print(f"{'Config Name':<20} {'Time (s)':<10} {'Error (m)':<12} {'Kp':<6} {'Ki':<6} {'Kd':<6}")
        print("-" * 60)
        
        for result in successful_results:
            params = result['params']
            print(f"{result['config_name']:<20} {result['completion_time']:<10.2f} "
                  f"{result['final_error']:<12.3f} {params['kp_linear']:<6.1f} "
                  f"{params['ki_linear']:<6.2f} {params['kd_linear']:<6.1f}")
        
        print(f"\nFailed configurations: {len(self.test_results) - len(successful_results)}")
        
        if successful_results:
            best = successful_results[0]
            print(f"\nBest configuration: {best['config_name']}")
            print(f"Parameters: {best['params']}")

async def run_tuning_session():
    """Run a comprehensive PID tuning session."""
    
    # Connect to drone
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
    
    # Takeoff to test altitude
    print("-- Taking off to test altitude")
    initial_controller = PIDController(drone)
    await initial_controller.go_to_waypoint(0.0, 0.0, -3.0, 0.0, timeout=20.0)
    await asyncio.sleep(2)

    # Create tuner
    tuner = PIDTuner(drone)
    
    # Define test configurations
    test_configs = [
        # Conservative (slow but stable)
        {
            'name': 'Conservative',
            'params': {
                'kp_linear': 0.8, 'ki_linear': 0.02, 'kd_linear': 0.4,
                'kp_angular': 1.0, 'ki_angular': 0.01, 'kd_angular': 0.5,
                'max_velocity': 1.0, 'position_tolerance': 0.2
            }
        },
        
        # Balanced (good starting point)
        {
            'name': 'Balanced',
            'params': {
                'kp_linear': 1.2, 'ki_linear': 0.05, 'kd_linear': 0.6,
                'kp_angular': 1.5, 'ki_angular': 0.02, 'kd_angular': 0.8,
                'max_velocity': 1.5, 'position_tolerance': 0.2
            }
        },
        
        # Aggressive (fast but may oscillate)
        {
            'name': 'Aggressive',
            'params': {
                'kp_linear': 2.0, 'ki_linear': 0.1, 'kd_linear': 1.0,
                'kp_angular': 2.5, 'ki_angular': 0.05, 'kd_angular': 1.2,
                'max_velocity': 2.5, 'position_tolerance': 0.2
            }
        },
        
        # High damping (reduces overshoot)
        {
            'name': 'High_Damping',
            'params': {
                'kp_linear': 1.0, 'ki_linear': 0.03, 'kd_linear': 1.5,
                'kp_angular': 1.2, 'ki_angular': 0.01, 'kd_angular': 1.8,
                'max_velocity': 1.2, 'position_tolerance': 0.2
            }
        },
        
        # No integral (prevents windup)
        {
            'name': 'No_Integral',
            'params': {
                'kp_linear': 1.5, 'ki_linear': 0.0, 'kd_linear': 0.8,
                'kp_angular': 2.0, 'ki_angular': 0.0, 'kd_angular': 1.0,
                'max_velocity': 1.8, 'position_tolerance': 0.2
            }
        }
    ]
    
    # Run tests
    print(f"\nStarting PID tuning with {len(test_configs)} configurations...")
    
    for config in test_configs:
        try:
            await tuner.test_pid_configuration(config['name'], config['params'])
        except Exception as e:
            print(f"Error testing {config['name']}: {e}")
            continue
    
    # Print summary
    tuner.print_results_summary()
    
    # Land
    print("\n-- Landing")
    await drone.action.land()

# PID Tuning Guidelines
TUNING_GUIDELINES = """
PID TUNING GUIDELINES FOR DRONE CONTROL
======================================

1. START WITH CONSERVATIVE VALUES:
   - Kp_linear: 0.5-1.0 (too high causes oscillation)
   - Ki_linear: 0.01-0.05 (too high causes overshoot and instability)
   - Kd_linear: 0.3-0.8 (adds damping, reduces overshoot)

2. TUNING PROCESS:
   a) Start with Kp only (Ki=0, Kd=0)
   b) Increase Kp until steady oscillation occurs
   c) Reduce Kp by 50%
   d) Add Kd to reduce overshoot and settling time
   e) Add small Ki to eliminate steady-state error

3. SYMPTOMS AND FIXES:
   - Oscillation: Reduce Kp or increase Kd
   - Slow response: Increase Kp
   - Overshoot: Increase Kd or reduce Kp
   - Steady-state error: Increase Ki (carefully)
   - Instability: Reduce all gains, especially Ki

4. SAFETY LIMITS:
   - Always set max_velocity appropriately for your environment
   - Use reasonable position_tolerance (0.1-0.5m)
   - Monitor for windup with integral terms

5. ENVIRONMENT FACTORS:
   - Wind: Increase Kd, may need higher Ki
   - Heavy drone: May need higher Kp
   - Light drone: May need lower gains overall
   - Simulator vs real: Real world often needs more damping (higher Kd)
"""

if __name__ == "__main__":
    print(TUNING_GUIDELINES)
    print("\nRunning automated tuning session...")
    asyncio.run(run_tuning_session())