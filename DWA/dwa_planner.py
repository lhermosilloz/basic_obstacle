#!/usr/bin/env python3
import os
# Set protobuf implementation to python before importing gz modules
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
import asyncio
import numpy as np
import time
import math
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
import matplotlib.pyplot as plt
from dwa_visualizer import DWAVisualizer
try:
    import gz.transport13 as gz_transport
except ImportError:
    print("Error: gz-transport Python bindings not found")
    print("Install with: sudo apt install python3-gz-transport13")
    exit(1)

class DynamicWindowApproachPlanner:
    def __init__(self, dist, vel, obs):
        # -- MAVSDK Stuff --
        self.drone = System()

        # -- LiDAR Stuff --
        self.node = gz_transport.Node() # LiDAR subscriber node
        self.latest_scan = None         # Store latest LiDAR scan
        self.start_listening()          # Start LiDAR subscription

        # Weights
        self.w_dist = dist
        self.w_vel = vel
        self.w_obs = obs

    def sample_velocities(self, current_forward_vel, current_yaw_rate, dt=0.1):
        """Sample only forward + yaw like if it were a ground robot."""
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
            for yaw_rate in np.linspace(min_yaw, max_yaw, 8):
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
    
    def trajectory_prediction_vectorized(self, current_state, candidates, time_horizon=2.0, dt=0.1):
        """Vectorized trajectory prediction using numpy for 10-100x speedup"""
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

    def collision_checking_optimized(self, trajectories, obstacles, safety_distance=0.25):
        """Optimized collision checking using numpy and early termination"""
        collision_mask = []
        
        # Convert obstacles to numpy array for vectorized operations
        if not obstacles:
            return [False] * len(trajectories)
        
        obs_array = np.array(obstacles)  # Shape: (N_obstacles, 2)
        safety_sq = safety_distance ** 2  # Avoid sqrt in inner loop
        
        for traj in trajectories:
            in_collision = False

            # UNCOMMENT LATER:
            # min_obs_dist = float('inf')

            # Check fewer points along trajectory (every 3rd point)
            check_points = traj[::3]
            
            # UNCOMMENT LATER:
            # check_points = np.array(traj[::3])  # Check every 3rd point instead of all
            
            # UNCOMMENT LATER:
            # # Shape: (Num_Path_Points, Num_Obstacles)
            # dx = check_points[:, 0][:, np.newaxis] - obs_array[:, 0]
            # dy = check_points[:, 1][:, np.newaxis] - obs_array[:, 1]
            # dists = np.sqrt(dx**2 + dy**2)
            
            # UNCOMMENT LATER:
            # # The single closest encounter on this entire path
            # min_obs_dist = np.min(dists)

            for point in check_points:
                px, py = point[0], point[1]
                
                # Vectorized distance calculation to all obstacles
                distances_sq = (obs_array[:, 0] - px) ** 2 + (obs_array[:, 1] - py) ** 2
                
                if np.any(distances_sq < safety_sq):
                    in_collision = True
                    break  # Early termination
                    
            collision_mask.append(in_collision)

            # UNCOMMENT LATER:
            # collision_mask.append([in_collision, min_obs_dist])
        
        return collision_mask

    def collision_checking_ultra_optimized(self, trajectories, obstacles, safety_distance=0.25):
        """Ultra-optimized: Check even fewer points and use squared distances"""
        collision_mask = []
        
        if not obstacles:
            return [False] * len(trajectories)
        
        obs_array = np.array(obstacles)
        safety_sq = safety_distance ** 2
        
        for traj in trajectories:
            # Check only START, MIDDLE, and END points (3 points total instead of ~13)
            check_indices = [0, len(traj)//2, -1]
            check_points = [traj[i] for i in check_indices]
            
            in_collision = False
            for point in check_points:
                px, py = point[0], point[1]
                
                # Vectorized squared distance (no sqrt needed)
                distances_sq = (obs_array[:, 0] - px) ** 2 + (obs_array[:, 1] - py) ** 2
                
                if np.any(distances_sq < safety_sq):
                    in_collision = True
                    break
                    
            collision_mask.append(in_collision)
        
        return collision_mask

    def trajectory_scoring(self, goal, collision_mask, trajectories, candidates, obstacles):
        """
        Ground up scoring
        """
        scores = []

        w_heading = 0.5     # Maintain course
        w_dist    = 0.25    # Go to goal
        w_vel     = 0.5     # Reward speed
        w_yaw     = 2.0     # Small penalty to prevent oscillation
        w_obs     = 2.0    # Avoid obstacles seriously

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
            
            score = (w_dist * dist_score) + \
                    (-w_vel * forward_vel) + \
                    (w_obs * self.smooth_obstacle_cost(min_obs_dist)) + \
                    (w_yaw * norm_yaw_rate) + \
                    (w_heading * norm_heading_error)
            
            scores.append(score)
            
        return scores
    
    def trajectory_scoring_optimized(self, goal, collision_mask, trajectories, candidates, obstacles):
        """Optimized scoring with reduced obstacle distance calculations"""
        scores = []
        
        # Pre-compute obstacle array
        if obstacles:
            obs_array = np.array(obstacles)
        else:
            obs_array = np.empty((0, 2))
        
        for i, traj in enumerate(trajectories):
            # UNCOMMENT LATER:
            # if collision_mask[i][0]:
            if collision_mask[i]:
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
                min_obs_dist = float('inf')

            # UNCOMMENT LATER:
            # min_obs_dist = collision_mask[i][1]
            
            # Rest of scoring (simplified)
            dist_score = math.sqrt((ex - goal[0]) ** 2 + (ey - goal[1]) ** 2)
            forward_vel = candidates[i]['forward_m_s']
            # Normalized Yaw Rate (0.0 to 1.0)
            norm_yaw_rate = abs(candidates[i]['yawspeed_deg_s']) / 60.0
            
            score = (self.w_dist * dist_score) + (self.w_vel * forward_vel) + (self.w_obs * (6 / min_obs_dist)) # + (0.05 * norm_yaw_rate)
            scores.append(score)
            
        return scores

    def trajectory_scoring_ultra_fast(self, goal, collision_mask, trajectories, candidates, obstacles):
        """Ultra-fast scoring: minimal calculations, vectorized where possible"""
        scores = []
        
        # Pre-calculate goal position once
        goal_x, goal_y = goal[0], goal[1]
        
        for i, traj in enumerate(trajectories):
            if collision_mask[i]:
                scores.append(float('inf'))
                continue

            # Only use END point for all calculations
            end_point = traj[-1]
            ex, ey = end_point[0], end_point[1]
            
            # Simple distance to goal (no sqrt needed for comparison)
            dist_score_sq = (ex - goal_x) ** 2 + (ey - goal_y) ** 2
            
            # Simple speed reward (no obstacle distance calculation)
            forward_vel = candidates[i]['forward_m_s']
            
            # Ultra-simple scoring (remove obstacle cost for speed)
            score = dist_score_sq - (forward_vel * 2.0)  # Heavily favor speed (No work because neglects obstacles)
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
        iteration = 0

        # viz = DWAVisualizer() # Initialize Visualizer
        # frame = 0

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
            obstacles = self.get_obstacles_world((state[0], state[1], state[3]))
            get_obstacles_time = time.time() - step_start

            # 3. Sample velocities
            step_start = time.time()
            candidates = self.sample_velocities(current_forward_vel=state[4], current_yaw_rate=state[7], dt=dt)
            get_sample_vel_time = time.time() - step_start

            # 4. Predict trajectories
            step_start = time.time()
            trajectories = self.trajectory_prediction_vectorized(current_state=state[:4], candidates=candidates, time_horizon=3.0, dt=dt)
            get_trajectory_time = time.time() - step_start

            # 5. Collision checking
            step_start = time.time()
            collision_mask = self.collision_checking_optimized(trajectories, obstacles, safety_distance=0.25)
            get_collision_time = time.time() - step_start

            # 6. Trajectory scoring
            step_start = time.time()
            scores = self.trajectory_scoring_optimized(goal, collision_mask, trajectories, candidates, obstacles)
            get_scoring_time = time.time() - step_start

            # 7. Choose best trajectory
            step_start = time.time()
            best_idx = np.argmin(scores)
            best_candidate = candidates[best_idx]
            get_best_trajectory_time = time.time() - step_start

            # frame += 1
            # if frame % 5 == 0:
            #     viz.render(state, goal, obstacles, trajectories, collision_mask, best_idx)

            # 8. Stop if all in collision
            step_start = time.time()
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
            get_command_time = time.time() - step_start

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

            # 10. Check if goal reached
            if np.linalg.norm(np.array([state[0] - goal[0], state[1] - goal[1]])) < stop_distance:
                print("Goal reached, stopping DWA loop.")
                await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                break

            # Pring the trajectory chosen:
            print(f"Chose forward Vel: {best_candidate['forward_m_s']:.2f} m/s, Yaw Rate: {best_candidate['yawspeed_deg_s']:.1f} deg/s")
            # Print current state
            print(f"Forward Vel: {state[4]:.2f} m/s, Yaw Rate: {state[7]:.1f} deg/s")

            # Compensate for the 100 ms loop time with the rest it should wait
            await asyncio.sleep(dt - total_loop_time)

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

    def smooth_obstacle_cost(self, min_obs_dist):
        """Smooth cost function for obstacle distance"""
        lethal_dist = 1.0      # Hard collision distance
        inflation_radius = 3.0  # Start penalizing from this distance
        max_cost = 3.25 # 1.6

        if min_obs_dist <= lethal_dist:
            return max_cost
        elif min_obs_dist >= inflation_radius:
            return 0.0
        else:
            # Linear interpolation between lethal_dist and inflation_radius
            return max_cost * ((inflation_radius - min_obs_dist) / (inflation_radius - lethal_dist))
        
    async def test_gazebo_state(self):
        """This function helped me figure out that calling the get_current_state() from Gazebo roughly takes like 30 ms"""
        last_time = time.time()
        iteration = 0
        
        while True:
            start_time = time.time()
            
            state = await self.get_current_state()

            if state is None:
                print("Failed to get state, retrying...")
                await asyncio.sleep(0.1)
                continue

            current_time = time.time()
            
            # Calculate timing metrics
            loop_time = current_time - start_time
            time_since_last = current_time - last_time
            expected_interval = 1  # Your sleep time
            
            iteration += 1
            
            print(f"Iteration {iteration:3d}: "
                f"State={state[0]:.3f}, "
                f"Loop time={loop_time*1000:.1f}ms, "
                f"Actual interval={time_since_last*1000:.1f}ms, "
                f"Expected={expected_interval*1000:.1f}ms")
            
            last_time = current_time
            await asyncio.sleep(expected_interval)

    async def speed_test(self):
        """Test how fast drone can change directions and speeds"""
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(2)

        test_commands = [
            VelocityBodyYawspeed(0.0, 0.0, 0.0, -15.0),
            VelocityBodyYawspeed(3.0, 0.0, 0.0, 0.0),
            VelocityBodyYawspeed(5.0, 0.0, 0.0, 0.0),
            VelocityBodyYawspeed(0.0, 5.0, 0.0, 0.0),
            VelocityBodyYawspeed(-5.0, 0.0, 0.0, 0.0),
            VelocityBodyYawspeed(0.0, -5.0, 0.0, 0.0),
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 90.0),
            VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0),
        ]

        for cmd in test_commands:
            print(f"Sending command: Forward={cmd.forward_m_s}, Right={cmd.right_m_s}, Down={cmd.down_m_s}, YawRate={cmd.yawspeed_deg_s}")
            for i in range(4):
                state = await self.get_current_state()
                print(f"  Time {i}: Forward Vel={state[4]:.2f} m/s, Right Vel={state[5]:.2f} m/s, Yaw Rate={state[7]:.1f} deg/s")
                await self.drone.offboard.set_velocity_body(cmd)
                await asyncio.sleep(1)

        # Stop the drone
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        print("Speed test completed.")