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
import matplotlib.pyplot as plt
from dwa_visualizer import DWAVisualizer
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
        max_yaw_accel = 20.0 # deg/s^2 (MPC_YAWRAUTO_ACC was 20)

        # Calculate reachable velocity ranges (How much it can change in 0.1s)
        max_forward_change = max_hor_accel * dt
        max_yaw_change = max_yaw_accel * dt

        # Reachable forward velocity range
        min_forward = max(0.0, current_forward_vel - max_forward_change)
        max_forward = min(12.0, current_forward_vel + max_forward_change)    # MPC_XY_VEL_MAX

        # Reachable yaw rate range
        min_yaw = max(-60.0, current_yaw_rate - max_yaw_change) # MPC_YAWRAUTO_MAX (was 60)
        max_yaw = min(60.0, current_yaw_rate + max_yaw_change)
        # print(f"Sampling velocities: Forward [{min_forward:.1f}, {max_forward:.1f}] m/s, Yaw [{min_yaw:.1f}, {max_yaw:.1f}] deg/s")

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

        # for rot in [-45, -30, -15, 15, 30, 45]:
        #     candidates.append({
        #         'forward_m_s': 0.0,
        #         'right_m_s': 0.0,
        #         'down_m_s': 0.0,
        #         'yawspeed_deg_s': rot
        #     })

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
    
    def get_obstacles_world(self, drone_pose):
        """
        Convert local LiDAR points to world frame.
        drone_pose: (x, y, yaw) in world frame
        """
        x_drone, y_drone, yaw_drone = drone_pose
        obstacles_local = self.get_obstacles()
        obstacles_world = []
        for x_local, y_local in obstacles_local:
            x_world = x_drone + x_local * math.cos(math.radians(yaw_drone)) - y_local * math.sin(math.radians(yaw_drone))
            y_world = y_drone + x_local * math.sin(math.radians(yaw_drone)) + y_local * math.cos(math.radians(yaw_drone))
            obstacles_world.append((x_world, y_world))
        return obstacles_world

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

    def trajectory_scoring(self, goal, collision_mask, trajectories, candidates, obstacles):
        """
        Ground up scoring
        """
        scores = []

        w_heading = 0.5     # Maintain course
        w_dist    = 0.75    # Go to goal
        w_vel     = 0.5     # Reward speed
        w_yaw     = 2.0     # Small penalty to prevent oscillation
        w_obs     = 10.0    # Avoid obstacles seriously

        for i, traj in enumerate(trajectories):
            # Hard collision check
            if collision_mask[i]:
                scores.append(float('inf'))
                continue

            # Minimum distance
            min_obs_dist = float('inf')

            for point in traj:
                px, py, pz, _ = point
                for obs in obstacles:
                    ox, oy = obs
                    distance = math.sqrt((px - ox) ** 2 + (py - oy) ** 2)
                    if distance < min_obs_dist:
                        min_obs_dist = distance

            # State at end of trajectory
            end_point = traj[-1]
            ex, ey, _, end_yaw = end_point
            
            # --- 1. Calculate Raw Metrics ---
            
            # Distance to goal
            dist_score = math.sqrt((ex - goal[0]) ** 2 + (ey - goal[1]) ** 2)

            # Heading Error (Angle difference)
            goal_heading = math.degrees(math.atan2(goal[1] - ey, goal[0] - ex))
            norm_heading_error = (abs((end_yaw - goal_heading + 180) % 360 - 180)) / 180 # [0 - 1]
            
            # Velocity and Yaw Rate from candidate
            forward_vel = candidates[i]['forward_m_s']
            norm_yaw_rate = abs(candidates[i]['yawspeed_deg_s']) / 60 # [0 - 1] (Divided by max rate)
            # (w_heading * norm_heading_error) + \
            # (w_yaw * yaw_rate) + \
            score = (w_dist * dist_score) + \
                    (w_vel * (12.0 - forward_vel)) + \
                    (w_obs * min_obs_dist**-1) + \
                    (w_yaw * norm_yaw_rate) + \
                    (w_heading * norm_heading_error)
            
            scores.append(score)
            
        return scores
    
    def choose_best_trajectory(self, scores, trajectories):
        """Choose the trajectory with the lowest score"""
        min_index = np.argmin(scores)
        return trajectories[min_index]
    
    async def run_dwa_loop(self, goal, dt=0.1, stop_distance=1.5):
        """Main DWA loop to be called periodically
        Inputs:
        - Goal position
        - Planning frequency
        - Stop conditions
        Outputs:
        - No return value
        """

        # viz = DWAVisualizer() # Initialize Visualizer
        # frame = 0

        while True:
            # 1. Get current state (position, yaw, etc)
            state = await self.get_current_state()

            if state is None:
                print("Failed to get state, retrying...")
                await asyncio.sleep(dt)
                continue

            print(f"Current State: x={state[0]:.2f}, y={state[1]:.2f}, z={state[2]:.2f}, yaw={state[3]:.2f}, x_vel={state[4]:.2f}, y_vel={state[5]:.2f}, z_vel={state[6]:.2f}, yaw_rate={state[7]:.2f}")

            # 2. Get latest obstacles from LiDAR
            #obstacles = self.get_obstacles()
            obstacles = self.get_obstacles_world((state[0], state[1], state[3]))

            # 3. Sample velocities
            candidates = self.sample_velocities(current_forward_vel=state[4], current_yaw_rate=state[7], dt=dt)

            # 4. Predict trajectories
            trajectories = self.trajectory_prediction(current_state=state[:4], candidates=candidates, time_horizon=4.0, dt=dt)

            # 5. Collision checking
            collision_mask = self.collision_checking(trajectories, obstacles, safety_distance=0.25)

            # 6. Trajectory scoring
            scores = self.trajectory_scoring(goal, collision_mask, trajectories, candidates, obstacles)

            # 7. Choose best trajectory
            best_idx = np.argmin(scores)
            best_candidate = candidates[best_idx]

            # frame += 1
            # if frame % 5 == 0:
            #     viz.render(state, goal, obstacles, trajectories, collision_mask, best_idx)

            # 8. Stop if all in collision
            if all(collision_mask):
                print("All trajectories in collision, stopping.")
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                break

            # 9. Send velocity command
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    best_candidate['forward_m_s'],
                    best_candidate['right_m_s'],
                    best_candidate['down_m_s'],
                    best_candidate['yawspeed_deg_s']
                )
            )

            # 10. Check if goal reached
            if np.linalg.norm(np.array([state[0] - goal[0], state[1] - goal[1]])) < stop_distance:
                print("Goal reached, stopping DWA loop.")
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                break

            await asyncio.sleep(dt)

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

    async def get_current_state(self):
        """Get current state from drone telemetry"""

        """
        MAVSDK API:
        - mavsdk.telemetry.PositionBody(x_m, y_m, z_m)
        - mavsdk.telemetry.Heading(heading_deg)
        And?
        - mavsdk.telemetry.VelocityBody(x_m_s, y_m_s, z_m_s)
        """

        try:
            # Get position and velocities
            async for position in self.drone.telemetry.odometry():
                x = position.position_body.x_m
                y = position.position_body.y_m
                z = position.position_body.z_m
                x_vel = position.velocity_body.x_m_s
                y_vel = position.velocity_body.y_m_s
                z_vel = position.velocity_body.z_m_s
                yaw_rate = position.angular_velocity_body.yaw_rad_s * (180.0 / math.pi)  # Convert to deg/s
                # Extract yaw from quaternion
                w = position.q.w  # [w, x, y, z]
                xq = position.q.x
                yq = position.q.y
                zq = position.q.z
                siny_cosp = 2 * (w * zq + xq * yq)
                cosy_cosp = 1 - 2 * (yq * yq + zq * zq)
                yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)
                break
            # Get heading
            # async for heading in self.drone.telemetry.heading():
            #     yaw = heading.heading_deg
            #     break

            return [x, y, z, yaw, x_vel, y_vel, z_vel, yaw_rate]
        except Exception as e:
            print(f"Failed to get current state: {e}")
            return None