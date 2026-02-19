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
    def __init__(self, sys_url: str = "udpin://0.0.0.0:14550", pml_url: str = "udpin://0.0.0.0:14551", dist: float = 1.0, vel: float = 1.0, obs: float = 1.0):
        # MAVSDK System
        self.drone = System()
        self.drone_sys_url = sys_url
        
        # Manual control
        self.is_flying = False
        self.max_velocity = 2.0  # m/s
        self.max_yaw_rate = 90.0  # deg/s

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
    async def connect_drone(self):
        """
        Docstring for connect_drone
        
        :return: Description
        :rtype: Any
        """
        # In order to receive OBSTACLE_DISTANCE messages, 
        # we need to connect using PyMAVLINK as a GCS
        # Hence, there might be a port conflict.
        
        # The SDK version on Starling lets us use 2 different ports for Mavlink
        # Can use 14550 for MAVSDK (DWA) and 14551 for PyMAVLINK (Keyboard + OBSTACLE_DISTANCE + PyQt info), or vice versa
        
        # Connect to drone using MAVSDK for control
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

            # 1. Get current state (position, yaw, etc)
            step_start = time.time()
            state = await self.get_current_state()
            get_state_time = time.time() - step_start

            if state is None:
                print("Failed to get state, retrying...")
                await asyncio.sleep(dt)
                continue

            # 2. Get latest obstacles from LiDAR
            step_start = time.time()
            obstacles = self.get_obstacles((state[0], state[1], state[3]))
            get_obstacles_time = time.time() - step_start

            # 3. Sample velocities
            step_start = time.time()
            candidates = self.sample_velocities(current_forward_vel=state[4], current_yaw_rate=state[7], dt=dt)
            get_sample_vel_time = time.time() - step_start

            # 4. Predict trajectories
            step_start = time.time()
            trajectories = self.trajectory_prediction(current_state=state[:4], candidates=candidates, time_horizon=3.0, dt=dt)
            get_trajectory_time = time.time() - step_start

            # 5. Collision checking
            step_start = time.time()
            collision_mask = self.collision_check(trajectories, obstacles, safety_distance=0.28)
            get_collision_time = time.time() - step_start

            # 6. Trajectory scoring
            step_start = time.time()
            scores = self.trajectory_scoring(goal, collision_mask, trajectories, candidates, obstacles)
            get_scoring_time = time.time() - step_start

            # 7. Choose best trajectory
            step_start = time.time()
            best_idx = np.argmin(scores)
            best_candidate = candidates[best_idx]
            get_best_trajectory_time = time.time() - step_start

            # 8. Stop if all in collision
            step_start = time.time()
            # item[0] for item in collision_mask (If forget)
            if all(item[0] for item in collision_mask):
                print("All trajectories in collision, stopping.")
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                return False

            # 9. Send velocity command
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    best_candidate['forward_m_s'],
                    best_candidate['right_m_s'],
                    best_candidate['down_m_s'],
                    best_candidate['yawspeed_deg_s']
                )
            )
            get_command_time = time.time() - step_start

            total_loop_time = time.time() - loop_start
            
            # Print timing every 10 iterations
            if iteration % 10 == 0:
                print(f"\n=== DWA TIMING ITERATION {iteration} ===")
                print(f"Get State:      {get_state_time*1000:6.1f}ms")
                print(f"Get Obstacles:  {get_obstacles_time*1000:6.1f}ms")
                print(f"Sample Vels:    {get_sample_vel_time*1000:6.1f}ms")
                print(f"Predict Trajs:  {get_trajectory_time*1000:6.1f}ms")
                print(f"Collision Chk:  {get_collision_time*1000:6.1f}ms")
                print(f"Scoring:        {get_scoring_time*1000:6.1f}ms")
                print(f"Selection:      {get_best_trajectory_time*1000:6.1f}ms")
                print(f"Send Command:   {get_command_time*1000:6.1f}ms")
                print(f"TOTAL LOOP:     {total_loop_time*1000:6.1f}ms")
                print(f"Candidates:     {len(candidates)}")
                print(f"Obstacles:      {len(obstacles)}")
                print(f"=====================================")

            # 10. Check if goal reached
            if np.linalg.norm(np.array([state[0] - goal[0], state[1] - goal[1]])) < stop_distance:
                print("Goal reached, stopping DWA loop.")
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                return True

            # Compensate for the 100 ms loop time with the rest it should wait
            await asyncio.sleep(dt - total_loop_time)

    async def get_current_state(self):
        pass

    def get_obstacles(self):
        pass

    def sample_velocities(self):
        pass

    def trajectory_prediction(self):
        pass

    def collision_check(self):
        pass

    def trajectory_scoring(self):
        pass

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