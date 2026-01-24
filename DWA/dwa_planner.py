#!/usr/bin/env python3
import os
# Set protobuf implementation to python before importing gz modules
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
import asyncio
import numpy as np
import time
import math
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed
try:
    import gz.transport13 as gz_transport
except ImportError:
    print("Error: gz-transport Python bindings not found")
    print("Install with: sudo apt install python3-gz-transport13")
    exit(1)

class DynamicWindowApproachPlanner:
    def __init__(self):
        # -- MAVSDK Stuff --
        self.drone = System()
        asyncio.run(self.connect_drone())

        # -- LiDAR Stuff --
        self.node = gz_transport.Node() # LiDAR subscriber node
        self.latest_scan = None         # Store latest LiDAR scan
        self.start_listening()          # Start LiDAR subscription

    def sample_velocities(self, current_forward_vel, current_yaw_rate, dt=0.1):
        """Sample only forward + yaw like if it were a ground robot."""
        # Only the achievable velocities are sampled
        candidates = []

        # The x500 max accelerations are found in px4 parameters
        max_hor_accel = 5.0  # m/s^2 (MPC_ACC_HOR_MAX)
        max_yaw_accel = 20.0 # deg/s^2 (MPC_YAWRAUTO_ACC)

        # Calculate reachable velocity ranges (How much it can change in 0.1s)
        max_forward_change = max_hor_accel * dt
        max_yaw_change = max_yaw_accel * dt

        # Reachable forward velocity range
        min_forward = max(0.0, current_forward_vel - max_forward_change)
        max_forward = min(12.0, current_forward_vel + max_forward_change)    # MPC_XY_VEL_MAX

        # Reachable yaw rate range
        min_yaw = max(-60.0, current_yaw_rate - max_yaw_change) # MPC_YAWRAUTO_MAX 
        max_yaw = min(60.0, current_yaw_rate + max_yaw_change)
        # print(f"Sampling velocities: Forward [{min_forward:.1f}, {max_forward:.1f}] m/s, Yaw [{min_yaw:.1f}, {max_yaw:.1f}] deg/s")

        # Sample forward velocities (np.arange for float steps, linspace for fixed number of samples)
        for forward_vel in np.linspace(min_forward, max_forward, 7):
            # Sample yaw rates
            for yaw_rate in np.linspace(min_yaw, max_yaw, 7):
                candidates.append({
                    'forward_m_s': forward_vel,
                    'right_m_s': 0.0,
                    'down_m_s': 0.0,
                    'yawspeed_deg_s': yaw_rate
                })
        return candidates

    def sample_full_4d_velocities(self):
        # In addition to the parameters already being used above,
        # the extra dimension for up/down, will be checked from:
        # - MPC_Z_VEL_MAX_UP
        # - MPC_Z_VEL_MAX_DN
        pass

    def trajectory_prediction(self, current_state, candidates, time_horizon=2.0, dt=0.1):
        """
        For each candidate velocity, predict the trajectory over the time horizon.
        - current_state: [x, y, z, yaw] Obtain from PX4 Odometry, GZ Sim Ground Truth (Maybe not z if only 2D)
        - candidates: list of velocity commands
        - time_horizon: total time to simulate
        - dt: time step for simulation
        """
        trajectories = []
        num_steps = int(time_horizon / dt)

        for cmd in candidates:
            traj = []
            x, y, z, yaw = current_state

            for step in range(num_steps):
                # Update position based on current velocities
                forward_vel = cmd['forward_m_s']
                right_vel = cmd['right_m_s']
                down_vel = cmd['down_m_s']
                yaw_rate = cmd['yawspeed_deg_s']

                # Convert yaw to radians for calculation
                yaw_rad = np.radians(yaw)

                # Calculate global frame velocities
                vx = forward_vel * np.cos(yaw_rad) - right_vel * np.sin(yaw_rad)
                vy = forward_vel * np.sin(yaw_rad) + right_vel * np.cos(yaw_rad)

                # Update positions
                x += vx * dt
                y += vy * dt
                z += down_vel * dt
                yaw += yaw_rate * dt

                traj.append((x, y, z, yaw))

            trajectories.append(traj)
        return trajectories
    
    def lidar_callback(self, msg):
        """Simple callback to store latest scan data"""
        self.latest_scan = msg

    def start_listening(self):
        """Start listening to LiDAR topic"""
        topic = "/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"
        
        # Use the correct gz.msgs10 import
        try:
            # Import the LaserScan message type from msgs10
            from gz.msgs10.laserscan_pb2 import LaserScan
            
            # Correct order: message_type, topic, callback
            success = self.node.subscribe(LaserScan, topic, self.lidar_callback)
            
            if success:
                print(f"Successfully subscribed to: {topic}")
                return True
            else:
                print(f"Failed to subscribe to: {topic}")
                return False
                
        except ImportError as e:
            print(f"Failed to import LaserScan: {e}")
            return False
        except Exception as e:
            print(f"Subscription failed: {e}")
            return False

    def get_obstacles(self) -> list:
        """Return list of detected obstacles in Cartesian coordinates"""
        obstacles = []
        if self.latest_scan is None:
            return obstacles
            
        msg = self.latest_scan
        angle_min = msg.angle_min
        angle_increment = msg.angle_step
        
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            angle = angle_min + (i * angle_increment)
            obstacles.append((angle, range_val))
        
        # Convert into Cartesian coordinates
        cartesian_obstacles = []
        for angle, distance in obstacles:
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            cartesian_obstacles.append((x, y))

        return cartesian_obstacles

    def get_latest_scan(self):
        if self.latest_scan is None:
            return None
        else:
            return self.latest_scan

    def collision_checking(self, trajectories, obstacles, safety_distance=1.0) -> list:
        """For each predicted trajectory, check if any point along the trajectory comes within
        a safety distance of any detected obstacle. Mark trajectories as 'in-collision' or 'safe'
        Inputs:
        - trajectories: A list of predicted trajectories. [(x, y, z, yaw), ...]
        - obstacles: A list of obstacle points from the LiDAR [(x, y)]
        - safety_distance: A float specifying the minimum safe distance from obstacles
        Outputs:
        - A list of booleans indicating whether each trajectory is in collision (True) or safe (False)
        """
        collision_mask = []

        for traj in trajectories:
            in_collision = False
            for point in traj:
                px, py, pz, _ = point
                for obs in obstacles:
                    ox, oy = obs
                    distance = math.sqrt((px - ox) ** 2 + (py - oy) ** 2)
                    if distance < safety_distance:
                        in_collision = True
                        break
                if in_collision:
                    break
            collision_mask.append(in_collision)

        return collision_mask

    def trajectory_scoring(self, goal, collision_mask, trajectories):
        """For each safe trajectory, compute a score based on criteria like:
        - Proximity to goal
        - Obstacle proximity
        - Speed
        - Smoothness
        Inputs:
        - goal: (x, y) coordinates of the goal position
        - collision_mask: List of booleans indicating if trajectory is in collision
        - trajectories: List of predicted trajectories
        Outputs:
        - Return trajectories with their scores.
        """
        scores = []
        for i, traj in enumerate(trajectories):
            if collision_mask[i]:
                scores.append(float('inf'))  # High cost for collision trajectories (Already considers obstacles)
                continue

            # Distance to goal at the end of trajectory
            end_point = traj[-1]
            ex, ey, ez, _ = end_point
            goal_distance = math.sqrt((ex - goal[0]) ** 2 + (ey - goal[1]) ** 2)

            # Optional: Speed


            # Optional: Smoothness
            beta = 1 # Weight for smoothness (Increase for smoothness, decrease for sharper turns)
            smoothness_score = 0
            for i in range(1, len(traj)):
                prev_yaw = traj[i-1][3]
                curr_yaw = traj[i][3]
                smoothness_score += abs(curr_yaw - prev_yaw)
            smoothness_score *= beta
            
            score = goal_distance + smoothness_score  # + other criteria
            scores.append(score)
        return scores
    
    def choose_best_trajectory(self, scores, trajectories):
        """Choose the trajectory with the lowest score"""
        min_index = np.argmin(scores)
        return trajectories[min_index]
    
    def run_dwa_loop(self):
        """Main DWA loop to be called periodically"""
        pass

    async def connect_drone(self):
        """Connect to the drone using MAVSDK"""
        
        # Connect to drone
        print("Connecting to drone...")
        await self.drone.connect(system_address="udp://:14540")

        # Wait for connection
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

        # Get global position estimate
        print("Waiting for global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Global position estimate OK")
                break