#!/usr/bin/env python3
import asyncio
import matplotlib.pyplot as plt
from dwa_planner import DynamicWindowApproachPlanner
import math

def test_velocity_sampler():
    planner = DynamicWindowApproachPlanner()
    
    print("=== Testing Velocity Sampler ===")
    
    # Test Case 1: Drone at rest
    print("\n1. Drone at rest:")
    candidates = planner.sample_velocities(current_forward_vel=0.0, current_yaw_rate=0.0)
    print(f"Number of candidates: {len(candidates)}")
    print("First few candidates:")
    for i, candidate in enumerate(candidates[:5]):
        print(f"  {i+1}: {candidate}")
    
    # Test Case 2: Drone moving forward
    print("\n2. Drone moving forward at 1.0 m/s:")
    candidates = planner.sample_velocities(current_forward_vel=1.0, current_yaw_rate=0.0)
    print(f"Number of candidates: {len(candidates)}")
    print("Forward velocity range:")
    forward_vels = [c['forward_m_s'] for c in candidates]
    print(f"  Min: {min(forward_vels):.1f}, Max: {max(forward_vels):.1f}")
    
    # Test Case 3: Drone turning
    print("\n3. Drone turning at 20 deg/s:")
    candidates = planner.sample_velocities(current_forward_vel=0.5, current_yaw_rate=20.0)
    print(f"Number of candidates: {len(candidates)}")
    print("Yaw rate range:")
    yaw_rates = [c['yawspeed_deg_s'] for c in candidates]
    print(f"  Min: {min(yaw_rates):.1f}, Max: {max(yaw_rates):.1f}")
    
    # Test Case 4: Edge case - maximum velocity
    print("\n4. Drone at max forward velocity:")
    candidates = planner.sample_velocities(current_forward_vel=2.0, current_yaw_rate=0.0)
    print(f"Number of candidates: {len(candidates)}")
    if candidates:
        print("Can only maintain or slow down:")
        forward_vels = [c['forward_m_s'] for c in candidates]
        print(f"  Max forward: {max(forward_vels):.1f}")

def visualize_velocity_space():
    planner = DynamicWindowApproachPlanner()
    
    # Test different current states
    test_states = [
        (0.0, 0.0),    # At rest
        (2.0, 0.0),    # Moving forward
        (0.5, 20.0),   # Moving + turning
    ]
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    for i, (curr_vel, curr_yaw) in enumerate(test_states):
        candidates = planner.sample_velocities(curr_vel, curr_yaw)
        
        forward_vels = [c['forward_m_s'] for c in candidates]
        yaw_rates = [c['yawspeed_deg_s'] for c in candidates]
        
        axes[i].scatter(forward_vels, yaw_rates, alpha=0.6)
        axes[i].scatter([curr_vel], [curr_yaw], color='red', s=100, label='Current')
        axes[i].set_xlabel('Forward Velocity (m/s)')
        axes[i].set_ylabel('Yaw Rate (deg/s)')
        axes[i].set_title(f'Current: ({curr_vel}, {curr_yaw})')
        axes[i].legend()
        axes[i].grid(True)
    
    plt.tight_layout()
    plt.show()

def test_trajectory_prediction():
    planner = DynamicWindowApproachPlanner()
    
    # Example initial state: [x, y, z, yaw]
    current_state = [0.0, 0.0, 0.0, 0.0]  # Start at origin, facing east (yaw=0)
    candidates = planner.sample_velocities(current_forward_vel=0.0, current_yaw_rate=0.0)
    
    # Predict trajectories
    trajectories = planner.trajectory_prediction(current_state, candidates, time_horizon=1.0, dt=0.1)
    
    print(f"Predicted {len(trajectories)} trajectories.")
    print("First trajectory (first 5 points):")
    for pt in trajectories[0][:5]:
        print(f"  {pt}")
    
    # Visualization
    plt.figure(figsize=(8, 8))
    for traj in trajectories:
        xs = [pt[0] for pt in traj]
        ys = [pt[1] for pt in traj]
        plt.plot(xs, ys, alpha=0.5)
    
    plt.scatter(current_state[0], current_state[1], color='red', label='Start')
    plt.title("Predicted Trajectories from DWA Candidates")
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

def test_scanner():
    planner = DynamicWindowApproachPlanner()
    # Run get_latest_scan until it returns data
    async def wait_for_scan():
        while True:
            scan = planner.get_latest_scan()
            if scan is not None:
                cartesian_points = planner.get_obstacles()
                print(f"Received LiDAR scan with {len(cartesian_points)} valid points.")
                
                # Visualization
                if cartesian_points:
                    xs = [pt[0] for pt in cartesian_points]
                    ys = [pt[1] for pt in cartesian_points]
                    plt.figure(figsize=(6, 6))
                    plt.scatter(xs, ys, c='b', s=10, label='Obstacles')
                    plt.scatter([0], [0], c='r', s=50, label='Drone (Origin)')
                    plt.xlabel("X (meters)")
                    plt.ylabel("Y (meters)")
                    plt.title("LiDAR Obstacle Points (Drone Frame)")
                    plt.axis('equal')
                    plt.grid(True)
                    plt.legend()
                    plt.show()
                break
            else:
                print("Waiting for LiDAR data...")
                await asyncio.sleep(1.0)
    asyncio.run(wait_for_scan())

def collision_check_trajectories():
    planner = DynamicWindowApproachPlanner()
    
    # Example initial state: [x, y, z, yaw]
    current_state = [0.0, 0.0, 0.0, 0.0]  # Start at origin, facing east (yaw=0)
    candidates = planner.sample_velocities(current_forward_vel=0.0, current_yaw_rate=0.0)
    
    # Predict trajectories
    trajectories = planner.trajectory_prediction(current_state, candidates, time_horizon=1.0, dt=0.1)
    
    # Get obstacles (in drone frame)
    obstacles = []
    async def wait_for_scan():
        while True:
            scan = planner.get_latest_scan()
            if scan is not None:
                print("LiDAR data received for collision checking.")
                return planner.get_obstacles()
            else:
                print("Waiting for LiDAR data...")
                await asyncio.sleep(1.0)
    obstacles = asyncio.run(wait_for_scan())
    
    collision_mask = planner.collision_checking(trajectories, obstacles, safety_distance=0.5)
    
    # Matplotlib visualization
    plt.figure(figsize=(8, 8))
    # Plot each trajectory: green if safe, red if in collision
    for traj, in_collision in zip(trajectories, collision_mask):
        xs = [pt[0] for pt in traj]
        ys = [pt[1] for pt in traj]
        color = 'red' if in_collision else 'green'
        plt.plot(xs, ys, color=color, alpha=0.7)
    
    # Plot obstacles
    if obstacles:
        obs_xs = [o[0] for o in obstacles]
        obs_ys = [o[1] for o in obstacles]
        plt.scatter(obs_xs, obs_ys, c='blue', s=30, label='Obstacles')
    
    # Plot drone start position
    plt.scatter([current_state[0]], [current_state[1]], c='magenta', s=80, label='Start')
    plt.title("DWA Trajectories: Green=Safe, Red=Collision")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()
    
if __name__ == "__main__":
    # test_velocity_sampler()
    # test_trajectory_prediction()
    # visualize_velocity_space()
    # test_scanner()
    # collision_check_trajectories()
    