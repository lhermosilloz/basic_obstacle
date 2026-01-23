#!/usr/bin/env python3

import math
import time
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

class PIDController:
    """
    PID Controller for smooth drone navigation to waypoints.
    Handles position control in X, Y, Z and yaw control separately.
    """
    
    def __init__(self, drone, 
                 kp_linear=1.5, ki_linear=0.1, kd_linear=0.8,
                 kp_angular=2.0, ki_angular=0.05, kd_angular=1.0,
                 max_velocity=2.0, max_angular_velocity=45.0,
                 position_tolerance=0.2, yaw_tolerance=5.0):
        """
        Initialize PID Controller with tunable gains and limits.
        
        Args:
            drone: MAVSDK System object
            kp_linear: Proportional gain for position control
            ki_linear: Integral gain for position control  
            kd_linear: Derivative gain for position control
            kp_angular: Proportional gain for yaw control
            ki_angular: Integral gain for yaw control
            kd_angular: Derivative gain for yaw control
            max_velocity: Maximum velocity in m/s
            max_angular_velocity: Maximum angular velocity in deg/s
            position_tolerance: Tolerance for reaching waypoint (meters)
            yaw_tolerance: Tolerance for reaching target yaw (degrees)
        """
        self.drone = drone
        
        # PID gains for linear motion (X, Y, Z)
        self.kp_linear = kp_linear
        self.ki_linear = ki_linear
        self.kd_linear = kd_linear
        
        # PID gains for angular motion (yaw)
        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular
        
        # Control limits
        self.max_velocity = max_velocity
        self.max_angular_velocity = max_angular_velocity
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        
        # Target state
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -1.0  # Default 1m altitude
        self.target_yaw = 0.0
        
        # PID state variables
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_error_yaw = 0.0
        
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.integral_yaw = 0.0
        
        self.prev_time = time.time()
        
        # Anti-windup limits for integral terms
        self.integral_limit = 10.0
        
    async def update_current_state(self):
        """Update current position and orientation from telemetry."""
        try:
            # Get position in NED coordinates
            async for position_ned in self.drone.telemetry.position_velocity_ned():
                self.current_x = position_ned.position.north_m
                self.current_y = position_ned.position.east_m
                self.current_z = position_ned.position.down_m
                break
                
            # Get current attitude (yaw) - try multiple telemetry sources
            yaw_obtained = False
            
            # Try attitude_euler first
            try:
                async for attitude_euler in self.drone.telemetry.attitude_euler():
                    self.current_yaw = attitude_euler.yaw_deg
                    if self.current_yaw < 0:
                        self.current_yaw += 360  # Normalize to [0, 360]
                    yaw_obtained = True
                    break
            except:
                pass
            
            # If attitude_euler failed, try attitude_quaternion
            if not yaw_obtained:
                try:
                    async for attitude_quat in self.drone.telemetry.attitude_quaternion():
                        # Convert quaternion to yaw
                        w, x, y, z = attitude_quat.w, attitude_quat.x, attitude_quat.y, attitude_quat.z
                        yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
                        self.current_yaw = math.degrees(yaw_rad)
                        if self.current_yaw < 0:
                            self.current_yaw += 360  # Normalize to [0, 360]
                        yaw_obtained = True
                        break
                except:
                    pass
            
            # If both failed, try heading
            if not yaw_obtained:
                try:
                    async for heading in self.drone.telemetry.heading():
                        self.current_yaw = heading.heading_deg
                        yaw_obtained = True
                        break
                except:
                    pass
            
            # If all telemetry failed, keep previous yaw value
            if not yaw_obtained:
                print("Warning: Could not get yaw from telemetry, using previous value")
                
        except Exception as e:
            print(f"Telemetry error: {e}")
            print("Using estimated position (telemetry unavailable)")
    
    def set_waypoint(self, x, y, z=None, yaw=None):
        """
        Set target waypoint.
        
        Args:
            x: Target North position (meters)
            y: Target East position (meters) 
            z: Target Down position (meters, negative for altitude)
            yaw: Target yaw angle (degrees)
        """
        self.target_x = x
        self.target_y = y
        if z is not None:
            self.target_z = z
        if yaw is not None:
            self.target_yaw = yaw
    
    def _clamp(self, value, min_val, max_val):
        """Clamp value between min and max."""
        return max(min_val, min(value, max_val))
    
    def _normalize_angle_error(self, error):
        """Normalize angle error to [-180, 180] degrees."""
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error
    
    def _calculate_pid_output(self, error, prev_error, integral, dt, kp, ki, kd, max_output=None):
        """Calculate PID output for a single axis."""
        # Proportional term
        proportional = kp * error
        
        # Integral term with anti-windup
        integral += error * dt
        integral = self._clamp(integral, -self.integral_limit, self.integral_limit)
        integral_term = ki * integral
        
        # Derivative term
        derivative = kd * (error - prev_error) / dt if dt > 0 else 0
        
        # Combined output
        output = proportional + integral_term + derivative
        
        # Apply output limits
        if max_output is not None:
            output = self._clamp(output, -max_output, max_output)
        
        return output, integral
    
    def compute_control_commands(self):
        """
        Compute PID control commands for position and yaw.
        
        Returns:
            tuple: (velocity_x, velocity_y, velocity_z, yaw_rate)
        """
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            dt = 0.01  # Prevent division by zero
        
        # Position errors
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_z = self.target_z - self.current_z
        
        # Yaw error (normalize to [-180, 180])
        error_yaw = self._normalize_angle_error(self.target_yaw - self.current_yaw)
        
        # Calculate PID outputs
        velocity_x, self.integral_x = self._calculate_pid_output(
            error_x, self.prev_error_x, self.integral_x, dt,
            self.kp_linear, self.ki_linear, self.kd_linear, self.max_velocity
        )
        
        velocity_y, self.integral_y = self._calculate_pid_output(
            error_y, self.prev_error_y, self.integral_y, dt,
            self.kp_linear, self.ki_linear, self.kd_linear, self.max_velocity
        )
        
        velocity_z, self.integral_z = self._calculate_pid_output(
            error_z, self.prev_error_z, self.integral_z, dt,
            self.kp_linear, self.ki_linear, self.kd_linear, self.max_velocity
        )
        
        yaw_rate, self.integral_yaw = self._calculate_pid_output(
            error_yaw, self.prev_error_yaw, self.integral_yaw, dt,
            self.kp_angular, self.ki_angular, self.kd_angular, self.max_angular_velocity
        )
        
        # Store current errors for next iteration
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z
        self.prev_error_yaw = error_yaw
        self.prev_time = current_time
        
        return velocity_x, velocity_y, velocity_z, yaw_rate
    
    def at_waypoint(self):
        """Check if drone has reached the target waypoint."""
        position_error = math.sqrt(
            (self.target_x - self.current_x)**2 +
            (self.target_y - self.current_y)**2 +
            (self.target_z - self.current_z)**2
        )
        
        yaw_error = abs(self._normalize_angle_error(self.target_yaw - self.current_yaw))
        
        return (position_error <= self.position_tolerance and 
                yaw_error <= self.yaw_tolerance)
    
    def get_distance_to_waypoint(self):
        """Get current distance to target waypoint."""
        return math.sqrt(
            (self.target_x - self.current_x)**2 +
            (self.target_y - self.current_y)**2 +
            (self.target_z - self.current_z)**2
        )
    
    async def go_to_waypoint(self, x, y, z=None, yaw=None, timeout=30.0):
        """
        Navigate to waypoint using PID control.
        
        Args:
            x: Target North position (meters)
            y: Target East position (meters)
            z: Target Down position (meters, negative for altitude)
            yaw: Target yaw angle (degrees)
            timeout: Maximum time to reach waypoint (seconds)
            
        Returns:
            bool: True if waypoint reached, False if timeout
        """
        self.set_waypoint(x, y, z, yaw)
        
        start_time = time.time()
        control_dt = 0.1  # Control loop period
        
        print(f"Navigating to waypoint: ({x:.2f}, {y:.2f}, {z:.2f}) with yaw {yaw:.1f}°")
        
        # Reset integral terms for new waypoint
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.integral_yaw = 0.0
        
        while not self.at_waypoint():
            loop_start = time.time()
            
            if time.time() - start_time > timeout:
                print(f"Timeout reached after {timeout}s")
                return False
            
            # Update current state from telemetry
            await self.update_current_state()
            
            # Compute control commands
            vel_x, vel_y, vel_z, yaw_rate = self.compute_control_commands()
            
            # Instead of simple integration, use position commands directly
            # This is more compatible with how MAVSDK offboard control works
            
            # Calculate desired position based on current position + velocity step
            desired_x = self.current_x + vel_x * control_dt
            desired_y = self.current_y + vel_y * control_dt
            desired_z = self.current_z + vel_z * control_dt
            desired_yaw = self.current_yaw + yaw_rate * control_dt
            
            # Clamp to reasonable bounds relative to target
            max_step = self.max_velocity * control_dt
            
            # Limit step size to prevent large jumps
            step_x = desired_x - self.current_x
            step_y = desired_y - self.current_y
            step_z = desired_z - self.current_z
            
            if abs(step_x) > max_step:
                step_x = max_step if step_x > 0 else -max_step
            if abs(step_y) > max_step:
                step_y = max_step if step_y > 0 else -max_step
            if abs(step_z) > max_step:
                step_z = max_step if step_z > 0 else -max_step
            
            final_x = self.current_x + step_x
            final_y = self.current_y + step_y
            final_z = self.current_z + step_z
            
            # Normalize yaw
            desired_yaw = desired_yaw % 360
            
            # Send position command
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(final_x, final_y, final_z, desired_yaw)
            )
            
            # Debug output (reduce frequency to avoid spam)
            if int(time.time() - start_time) % 2 == 0:  # Every 2 seconds
                distance = self.get_distance_to_waypoint()
                print(f"Distance: {distance:.2f}m | Pos: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) | "
                      f"Target: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}) | "
                      f"Vel: ({vel_x:.2f}, {vel_y:.2f}, {vel_z:.2f})")
            
            # Maintain control loop timing
            loop_time = time.time() - loop_start
            sleep_time = max(0, control_dt - loop_time)
            await asyncio.sleep(sleep_time)
        
        print("Waypoint reached!")
        return True
    
    async def follow_waypoint_sequence(self, waypoints, timeout_per_waypoint=30.0):
        """
        Follow a sequence of waypoints.
        
        Args:
            waypoints: List of tuples [(x, y, z, yaw), ...] or [(x, y), ...]
            timeout_per_waypoint: Timeout for each individual waypoint
            
        Returns:
            bool: True if all waypoints reached, False if any timeout
        """
        for i, waypoint in enumerate(waypoints):
            print(f"\nWaypoint {i+1}/{len(waypoints)}")
            
            if len(waypoint) == 2:
                x, y = waypoint
                z, yaw = None, None
            elif len(waypoint) == 3:
                x, y, z = waypoint
                yaw = None
            elif len(waypoint) == 4:
                x, y, z, yaw = waypoint
            else:
                print(f"Invalid waypoint format: {waypoint}")
                continue
            
            success = await self.go_to_waypoint(x, y, z, yaw, timeout_per_waypoint)
            if not success:
                print(f"Failed to reach waypoint {i+1}")
                return False
        
        print("All waypoints completed!")
        return True