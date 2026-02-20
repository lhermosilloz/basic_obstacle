#!/usr/bin/env python3

# DWA Planner stuff
import os
import asyncio
import numpy as np
import time
import math
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

# Hardware reading
import threading
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pymavlink import mavutil
from PyQt5.QtWidgets import QApplication

# Keyboard control
import keyboard

class StarlingDWA:
    def __init__(self, sys_url: str = "udpin://0.0.0.0:14550", pml_url: str = "udpin:0.0.0.0:14551", dist: float = 1.0, vel: float = 1.0, obs: float = 1.0):
        # MAVSDK System
        self.drone = System()
        self.drone_sys_url = sys_url
        
        # Manual control
        self.is_flying = False
        self.max_velocity = 2.0  # m/s
        self.max_yaw_rate = 90.0  # deg/s
        self.override = False

        # PyMAVLINK
        self.drone_pml_url = pml_url
        self.master = None

        # Starling VIO output through PyMAVLINK (OBSTACLE_DISTANCE)
        self.latest_obs = None
        self.running_obs = False
        self.thread_obs = None

        # PyMAVLINK GCS Heartbeat needed
        self.heartbeat_thread = None
        self.heartbeat_running = False

        # Starling DWA weights and parameters
        self.w_dist = dist
        self.w_vel = vel
        self.w_obs = obs

    # Used by both DWA and Hardware reader
    async def connect_and_setup_drone(self):
        """
        Docstring for connect_and_setup_drone
        
        :return: Description
        :rtype: Any
        """
        # In order to receive OBSTACLE_DISTANCE messages, 
        # we need to connect using PyMAVLINK as a GCS
        # Hence, there might be a port conflict.
        
        # The SDK version on Starling lets us use 2 different ports for Mavlink
        # Can use 14550 for MAVSDK (DWA) and 14551 for PyMAVLINK (Keyboard + OBSTACLE_DISTANCE + PyQt info), or vice versa
        
        # Connect to drone using MAVSDK for DWA control, manual override and telemetry
        print("Connecting to drone system...")
        await self.drone.connect(system_address=self.drone_sys_url)

        # Connect to drone using PyMAVLINK for hardware reading
        print("Connecting to drone via PyMAVLINK for hardware reading...")
        self.master = mavutil.mavlink_connection(self.drone_pml_url)

        # Wait for connection
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

        # Set initial velocity to zero
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        # Start offboard mode
        print("Starting offboard mode...")
        await self.drone.offboard.start()

        # Arm the drone
        print("Arming drone...")
        await self.drone.action.arm()

        print("Drone ready for keyboard control and DWA!")
        self.is_flying = True

    # DWA related functions
    async def run_dwa_loop(self, goal, dt: float = 0.1, stop_distance: float = 1.5):
        """
        Main DWA loop to be called periodically
        Inputs:
        - Goal position
        - Planning frequency
        - Stop conditions
        Outputs:
        - No return value
        """
        
        iteration = 0

        while True:
            iteration += 1
            loop_start = time.time()

            # # 1. Get current state (position, yaw, etc)
            # step_start = time.time()
            # state = await self.get_current_state()
            # get_state_time = time.time() - step_start

            # if state is None:
            #     print("Failed to get state, retrying...")
            #     await asyncio.sleep(dt)
            #     continue

            # # 2. Get latest obstacles from LiDAR
            # step_start = time.time()
            # obstacles = self.get_obstacles((state[0], state[1], state[3]))
            # get_obstacles_time = time.time() - step_start

            # # 3. Sample velocities
            # step_start = time.time()
            # candidates = self.sample_velocities(current_forward_vel=state[4], current_yaw_rate=state[7], dt=dt)
            # get_sample_vel_time = time.time() - step_start

            # # 4. Predict trajectories
            # step_start = time.time()
            # trajectories = self.trajectory_prediction(current_state=state[:4], candidates=candidates, time_horizon=3.0, dt=dt)
            # get_trajectory_time = time.time() - step_start

            # # 5. Collision checking
            # step_start = time.time()
            # collision_mask = self.collision_check(trajectories, obstacles, safety_distance=0.28)
            # get_collision_time = time.time() - step_start

            # # 6. Trajectory scoring
            # step_start = time.time()
            # scores = self.trajectory_scoring(goal, collision_mask, trajectories, candidates, obstacles)
            # get_scoring_time = time.time() - step_start

            # # 7. Choose best trajectory
            # step_start = time.time()
            # best_idx = np.argmin(scores)
            # best_candidate = candidates[best_idx]
            # get_best_trajectory_time = time.time() - step_start

            # # 8. Stop if all in collision
            # step_start = time.time()
            # # item[0] for item in collision_mask (If forget)
            # if all(item[0] for item in collision_mask):
            #     print("All trajectories in collision, stopping.")
            #     await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            #     return False

            # # 9. Send velocity command
            # await self.drone.offboard.set_velocity_body(
            #     VelocityBodyYawspeed(
            #         best_candidate['forward_m_s'],
            #         best_candidate['right_m_s'],
            #         best_candidate['down_m_s'],
            #         best_candidate['yawspeed_deg_s']
            #     )
            # )

            if keyboard.is_pressed('esc'):
                print("ESC pressed - Emergency stop!")
                self.override = True
                await self.emergency_stop()
                
            if keyboard.is_pressed('l'):
                print("L pressed - Landing...")
                self.override = True
                await self.emergency_stop()

            forward, right, down, yaw_rate = self.get_velocity_from_keys()
            
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward,
                    right,
                    down,
                    yaw_rate
                )
            )
            # get_command_time = time.time() - step_start

            total_loop_time = time.time() - loop_start
            
            # # Print timing every 10 iterations
            # if iteration % 10 == 0:
            #     print(f"\n=== DWA TIMING ITERATION {iteration} ===")
            #     print(f"Get State:      {get_state_time*1000:6.1f}ms")
            #     print(f"Get Obstacles:  {get_obstacles_time*1000:6.1f}ms")
            #     print(f"Sample Vels:    {get_sample_vel_time*1000:6.1f}ms")
            #     print(f"Predict Trajs:  {get_trajectory_time*1000:6.1f}ms")
            #     print(f"Collision Chk:  {get_collision_time*1000:6.1f}ms")
            #     print(f"Scoring:        {get_scoring_time*1000:6.1f}ms")
            #     print(f"Selection:      {get_best_trajectory_time*1000:6.1f}ms")
            #     print(f"Send Command:   {get_command_time*1000:6.1f}ms")
            #     print(f"TOTAL LOOP:     {total_loop_time*1000:6.1f}ms")
            #     print(f"Candidates:     {len(candidates)}")
            #     print(f"Obstacles:      {len(obstacles)}")
            #     print(f"=====================================")

            # # 10. Check if goal reached
            # if np.linalg.norm(np.array([state[0] - goal[0], state[1] - goal[1]])) < stop_distance:
            #     print("Goal reached, stopping DWA loop.")
            #     await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            #     return True

            # Compensate for the 100 ms loop time with the rest it should wait
            await asyncio.sleep(dt - total_loop_time)

    async def get_current_state(self):
        """
        Docstring for get_current_state
        :return: Description
        """
        try:
            # Get position and velocities
            async for position in self.drone.telemetry.odometry():
                # 1. Get position & raw velocities (world frame / NED)
                x = position.position_body.x_m
                y = position.position_body.y_m
                z = position.position_body.z_m

                # 2. Get velocities
                x_vel = position.velocity_body.x_m_s    # North
                y_vel = position.velocity_body.y_m_s    # East
                z_vel = position.velocity_body.z_m_s    # Down
                yaw_rate = position.angular_velocity_body.yaw_rad_s * (180.0 / math.pi)  # Convert to deg/s
                
                # Extract yaw from quaternion
                w = position.q.w  # [w, x, y, z]
                xq = position.q.x
                yq = position.q.y
                zq = position.q.z

                # Get heading (yaw) in degrees
                siny_cosp = 2 * (w * zq + xq * yq)
                cosy_cosp = 1 - 2 * (yq * yq + zq * zq)
                yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

                # Get forward velocity in body frame
                forward_vel = (x_vel * math.cos(math.radians(yaw))) + (y_vel * math.sin(math.radians(yaw)))

                # Get right velocity in body frame
                right_vel = (-x_vel * math.sin(math.radians(yaw))) + (y_vel * math.cos(math.radians(yaw)))
                break

            return [x, y, z, yaw, forward_vel, right_vel, z_vel, yaw_rate]
        except Exception as e:
            print(f"Failed to get current state: {e}")
            return None

    def get_obstacles(self):
        pass

    def sample_velocities(self, current_forward_vel, current_yaw_rate, dt=0.1):
        """
        Docstring for sample_velocities
        :param current_forward_vel: Description
        :param current_yaw_rate: Description
        :param dt: Description
        """
        # Only the achievable velocities are sampled
        candidates = []

        # The x500 max accelerations are found in px4 parameters
        max_hor_accel = 5.0  # m/s^2 (MPC_ACC_HOR_MAX was 5)
        max_yaw_accel = 60.0 # deg/s^2 (MPC_YAWRAUTO_ACC was 20)

        # Calculate reachable velocity ranges (How much it can change in 0.1s)
        max_forward_change = max_hor_accel * dt
        max_yaw_change = max_yaw_accel * dt

        # Reachable forward velocity range
        min_forward = max(0.0, current_forward_vel - max_forward_change)
        max_forward = min(12.0, current_forward_vel + max_forward_change)    # MPC_XY_VEL_MAX / MPC_XY_CRUISE

        # Reachable yaw rate range
        min_yaw = max(-60.0, current_yaw_rate - max_yaw_change) # MPC_YAWRAUTO_MAX (was 60)
        max_yaw = min(60.0, current_yaw_rate + max_yaw_change)
        
        # Sample forward velocities (np.arange for float steps, linspace for fixed number of samples)
        for forward_vel in np.linspace(min_forward, max_forward, 8):
            # Sample yaw rates
            for yaw_rate in np.linspace(min_yaw, max_yaw, 10):
                candidates.append({
                    'forward_m_s': forward_vel,
                    'right_m_s': 0.0,
                    'down_m_s': 0.0,
                    'yawspeed_deg_s': yaw_rate
                })

        return candidates

    def trajectory_prediction(self, current_state, candidates, time_horizon=2.0, dt=0.1):
        """
        Docstring for trajectory_prediction
        :param current_state: Description
        :param candidates: Description
        :param time_horizon: Description
        :param dt: Description
        """
        num_steps = int(time_horizon / dt)
        num_candidates = len(candidates)
        
        # Convert candidates to numpy arrays
        forward_vels = np.array([cmd['forward_m_s'] for cmd in candidates])
        right_vels = np.array([cmd['right_m_s'] for cmd in candidates])
        down_vels = np.array([cmd['down_m_s'] for cmd in candidates])
        yaw_rates = np.array([cmd['yawspeed_deg_s'] for cmd in candidates])
        
        # Initialize state arrays
        x = np.full(num_candidates, current_state[0], dtype=np.float32)
        y = np.full(num_candidates, current_state[1], dtype=np.float32)
        z = np.full(num_candidates, current_state[2], dtype=np.float32)
        yaw = np.full(num_candidates, current_state[3], dtype=np.float32)
        
        # Store all trajectories
        trajectories = []
        for i in range(num_candidates):
            trajectories.append([])
        
        # Simulate all trajectories simultaneously
        for step in range(num_steps):
            # Convert yaw to radians for all candidates at once
            yaw_rad = np.radians(yaw)
            
            # Calculate global frame velocities for all candidates
            vx = forward_vels * np.cos(yaw_rad) - right_vels * np.sin(yaw_rad)
            vy = forward_vels * np.sin(yaw_rad) + right_vels * np.cos(yaw_rad)
            
            # Update positions for all candidates
            x += vx * dt
            y += vy * dt
            z += down_vels * dt
            yaw += yaw_rates * dt
            
            # Store current step for all trajectories
            for i in range(num_candidates):
                trajectories[i].append((x[i], y[i], z[i], yaw[i]))
        
        return trajectories

    def collision_check(self, trajectories, obstacles, safety_distance=0.25):
        """
        Docstring for collision_check
        :param trajectories: Description
        :param obstacles: Description
        :param safety_distance: Description
        """
        collision_mask = []
        
        # Convert obstacles to numpy array for vectorized operations
        if not obstacles:
            # return [False] * len(trajectories)
            # UNCOMMENT LATER:
            return [[False, 1000000.0] ] * len(trajectories)
        
        obs_array = np.array(obstacles)  # Shape: (N_obstacles, 2)
        safety_sq = safety_distance ** 2  # Avoid sqrt in inner loop
        
        for traj in trajectories:
            in_collision = False

            # UNCOMMENT LATER:
            min_obs_dist = 1000000.0

            # Check fewer points along trajectory (every 3rd point)
            # check_points = traj[::3]
            
            # UNCOMMENT LATER:
            check_points = np.array(traj[::3])  # Check every 3rd point instead of all
            
            # UNCOMMENT LATER:
            # # Shape: (Num_Path_Points, Num_Obstacles)
            dx = check_points[:, 0][:, np.newaxis] - obs_array[:, 0]
            dy = check_points[:, 1][:, np.newaxis] - obs_array[:, 1]
            dists = np.sqrt(dx**2 + dy**2)
            
            # UNCOMMENT LATER:
            # # The single closest encounter on this entire path
            min_obs_dist = np.min(dists)

            for point in check_points:
                px, py = point[0], point[1]
                
                # Vectorized distance calculation to all obstacles
                distances_sq = (obs_array[:, 0] - px) ** 2 + (obs_array[:, 1] - py) ** 2
                
                if np.any(distances_sq < safety_distance):
                    in_collision = True
                    break  # Early termination
                    
            # collision_mask.append(in_collision)

            # UNCOMMENT LATER:
            collision_mask.append([in_collision, min_obs_dist])
        
        return collision_mask

    def trajectory_scoring(self, goal, collision_mask, trajectories, candidates, obstacles):
        """
        Docstring for trajectory_scoring
        :param goal: Description
        :param collision_mask: Description
        :param trajectories: Description
        :param candidates: Description
        :param obstacles: Description
        """
        scores = []
        
        # Pre-compute obstacle array
        if obstacles:
            obs_array = np.array(obstacles)
        else:
            obs_array = np.empty((0, 2))
        
        for i, traj in enumerate(trajectories):
            # UNCOMMENT LATER:
            if collision_mask[i][0]:
            # if collision_mask[i]:
                scores.append(float('inf'))
                continue

            # Only check end point for obstacle distance (not entire trajectory)
            end_point = traj[-1]
            ex, ey = end_point[0], end_point[1]
            
            # Vectorized distance to all obstacles from end point only
            if len(obs_array) > 0:
                distances = np.sqrt((obs_array[:, 0] - ex) ** 2 + (obs_array[:, 1] - ey) ** 2)
                min_obs_dist = np.min(distances)
            else:
                min_obs_dist = 1000000.0 # Won't detect anything near, but it brings the cost to zero which is what we want

            # UNCOMMENT LATER:
            min_obs_dist = collision_mask[i][1]
            
            # Rest of scoring (simplified)
            dist_score = math.sqrt((ex - goal[0]) ** 2 + (ey - goal[1]) ** 2)
            forward_vel = candidates[i]['forward_m_s']
            # Normalized Yaw Rate (0.0 to 1.0)
            norm_yaw_rate = abs(candidates[i]['yawspeed_deg_s']) / 60.0
            
            score = (self.w_dist * dist_score) + (self.w_vel * forward_vel) + (self.w_obs * (6 / min_obs_dist)) # + (0.05 * norm_yaw_rate)
            scores.append(score)
            
        return scores

    # Hardware reading related functions
    def start_hardware_listening(self):
        """
        Docstring for start_hardware_listening

        :return: Description
        :rtype: Any
        """

        # Start the obstacle distance listening thread
        self.running_obs = True
        self.thread_obs = threading.Thread(target=self._listen_obs, daemon=True)
        self.thread_obs.start()

        # Start the heartbeat thread
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(target=self._send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def stop_hardware_listening(self):
        """
        Docstring for stop_hardware_listening

        :return: Description
        :rtype: Any
        """
        # Stop obstacle distance listening thread
        self.running_obs = False
        if self.thread_obs:
            self.thread_obs.join()

        # Stop heartbeat thread
        self.heartbeat_running = False
        if self.heartbeat_thread:
            self.heartbeat_thread.join()

    def _send_heartbeat(self):
        """
        Docstring for _send_heartbeat
        
        :param self: Description
        """
        while self.heartbeat_running:
            if self.master:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            time.sleep(1)
        
    def _listen_obs(self):
        # NOTE: Needs supervision to ensure angles are in correct reference frame and units
        """
        Docstring for _listen_obs
        
        :param self: Description
        """
        while self.running_obs:
            msg = self.master.recv_match(type='OBSTACLE_DISTANCE', blocking=True, timeout=1)
            if msg:
                self.latest_obs = msg
                # If you want to add decaying for visualization, add here

    def get_distances(self):
        """
        Docstring for get_distances
        """
        if self.latest_obs:
            return [d / 100.0 for d in self.latest_obs.distances]
        return None
    
    # Keyboard control functions
    def get_velocity_from_keys(self):
        """
        Docstring for get_velocity_from_keys
        :return: Description
        :rtype: Any
        """
        forward = 0.0
        right = 0.0
        down = 0.0
        yaw_rate = 0.0
        
        # Forward/Backward (Up/Down arrow keys)
        if keyboard.is_pressed('up'):
            forward = self.max_velocity
            self.override = True
        elif keyboard.is_pressed('down'):
            forward = -self.max_velocity
            self.override = True
            
        # Left/Right Roll (Left/Right arrow keys)  
        if keyboard.is_pressed('left'):
            right = -self.max_velocity
            self.override = True
        elif keyboard.is_pressed('right'):
            right = self.max_velocity
            self.override = True
            
        # Up/Down (Space/Shift keys)
        if keyboard.is_pressed('space'):
            down = -self.max_velocity  # Negative for up in NED frame
            self.override = True
        elif keyboard.is_pressed('shift'):
            down = self.max_velocity   # Positive for down in NED frame
            self.override = True
            
        # Yaw Left/Right (A/D keys)
        if keyboard.is_pressed('a'):
            yaw_rate = -self.max_yaw_rate  # Negative for left
            self.override = True
        elif keyboard.is_pressed('d'):
            yaw_rate = self.max_yaw_rate   # Positive for right
            self.override = True
            
        return forward, right, down, yaw_rate

    async def emergency_stop(self):
        """Stop drone and land safely"""
        print("Stopping drone...")
        
        # Stop all movement
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1)
        
        # Now land at current position
        print("Landing")
        await self.drone.action.land()
        
        # Wait for disarming
        print("Waiting for disarm...")
        async for armed in self.drone.telemetry.armed():
            if not armed:
                print("Drone has disarmed successfully!")
                break
        
        # Stop offboard mode
        print("Stopping offboard mode...")
        await self.drone.offboard.stop()

        print("Control session ended safely.")
        self.is_flying = False


async def main(starling: StarlingDWA):
    available_waypoints = [(2, -2), (2, 0), (2, 2), (0, -2), (0, 0), (0, 2), (-2, -2), (-2, 0), (-2, 2)]
    waypoints = [(2, 0), (0, 0), (2, -2), (0, 0), (0, -2), (0, 0), (-2, -2), (0, 0), (-2, 0), (0, 0), (-2, 2), (0, 0), (0, 2), (0, 0), (2, 2), (0, 0)]
    await starling.connect_and_setup_drone()
    for waypoint in waypoints:
        print(f"Navigating to waypoint: {waypoint}")
        while True:
            reached = await starling.run_dwa_loop(goal=waypoint, dt=0.1, stop_distance=1)
            if reached:
                print(f"Reached waypoint: {waypoint}")
                break

if __name__ == "__main__":
    print("Starling DWA with Keyboard Override")

    starling = StarlingDWA(sys_url = "udpin://0.0.0.0:14550", pml_url = "udpin:0.0.0.0:14551", dist=1.0, vel=1.0, obs=1.0)
    asyncio.run(main(starling))