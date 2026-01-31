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

def collision_check_trajectories(horizon=3.0, w_dist=2.0, w_vel=-0.5, w_obs=6.0):
    planner = DynamicWindowApproachPlanner(w_dist, w_vel, w_obs)
    
    # Obstacles remain the same
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

    current_state = [0.0, 0.0, 0.0, 0.0]  # Start at origin, facing east (yaw=0)

    speed_candidates = []
    for i in range(1, 13):
        candidates = planner.sample_velocities(current_forward_vel=i, current_yaw_rate=0)
        speed_candidates.append(candidates)
    
    # Predict trajectories
    speed_trajectories = []
    for candidates in speed_candidates:
        trajectories = planner.trajectory_prediction_vectorized(current_state, candidates, time_horizon=horizon, dt=0.1)
        speed_trajectories.append(trajectories)
    
    # Get the collision mask
    speed_collision_masks = []
    for trajectories in speed_trajectories:
        collision_mask = planner.collision_checking_optimized(trajectories, obstacles, safety_distance=0.5)
        speed_collision_masks.append(collision_mask)

    # Score the trajectories
    speed_scores = []
    for trajectories, collision_mask, candidates in zip(speed_trajectories, speed_collision_masks, speed_candidates):
        scores = planner.trajectory_scoring_optimized((0,10), collision_mask, trajectories, candidates, obstacles)
        speed_scores.append(scores)
    
    # Filter out infinite scores and get valid scores for color mapping
    valid_scores = [score for score in scores if score != float('inf')]
    finite_scores = [score for score in scores if score != float('inf')]
    
    if not finite_scores:
        print("All trajectories have infinite scores!")
        return
    
    # Normalize scores for color mapping (0=best/green, 1=worst/red)
    min_score = min(finite_scores)
    max_score = max(finite_scores)
    score_range = max_score - min_score
    
    print(f"Score range: {min_score:.3f} to {max_score:.3f}")
    print(f"Valid trajectories: {len(finite_scores)}/{len(scores)}")
    
    # Matplotlib visualization with colormap
    plt.figure(figsize=(10, 8))
    
    # Create colormap (green = low score/good, red = high score/bad)
    import matplotlib.cm as cm
    import numpy as np
    
    # Plot each trajectory with color based on score
    for i, (traj, score) in enumerate(zip(trajectories, scores)):
        xs = [pt[0] for pt in traj]
        ys = [pt[1] for pt in traj]
        
        if score == float('inf'):
            # Plot collision trajectories in black
            color = 'black'
            alpha = 0.3
            linewidth = 0.5
        else:
            # Normalize score to 0-1 range
            if score_range > 0:
                normalized_score = (score - min_score) / score_range
            else:
                normalized_score = 0.0
            
            # Use RdYlGn_r colormap (reversed so green=good, red=bad)
            color = cm.RdYlGn_r(normalized_score)
            alpha = 0.8
            linewidth = 1.0
            
        plt.plot(xs, ys, color=color, alpha=alpha, linewidth=linewidth)
    
    # Plot obstacles
    if obstacles:
        obs_xs = [o[0] for o in obstacles]
        obs_ys = [o[1] for o in obstacles]
        plt.scatter(obs_xs, obs_ys, c='blue', s=30, label='Obstacles', zorder=5)
    
    # Plot drone start position
    plt.scatter([current_state[0]], [current_state[1]], c='magenta', s=100, 
                marker='o', label='Start', zorder=5)
    
    # Plot goal
    plt.scatter([10], [0], c='gold', s=100, marker='*', label='Goal', zorder=5)
    
    # Find and highlight best trajectory
    if finite_scores:
        best_idx = scores.index(min(finite_scores))
        best_traj = trajectories[best_idx]
        xs = [pt[0] for pt in best_traj]
        ys = [pt[1] for pt in best_traj]
        plt.plot(xs, ys, color='lime', linewidth=3, alpha=1.0, label='Best Trajectory')
    
    plt.title("DWA Trajectories: Color-coded by Score\n(Green=Low Score/Good, Red=High Score/Bad, Black=Collision)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Add colorbar to show score mapping
    if finite_scores and score_range > 0:
        sm = plt.cm.ScalarMappable(cmap=cm.RdYlGn_r, 
                                   norm=plt.Normalize(vmin=min_score, vmax=max_score))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=plt.gca())
        cbar.set_label('Trajectory Score')
    
    plt.tight_layout()
    plt.show()
    
    # Print some statistics
    print(f"\nBest score: {min(finite_scores):.3f}")
    print(f"Worst finite score: {max(finite_scores):.3f}")
    if finite_scores:
        print(f"Best trajectory candidate: {candidates[scores.index(min(finite_scores))]}")
    
def test_current_state():
    planner = DynamicWindowApproachPlanner()
    async def print_state():
        while True:
            state = await planner.get_current_state()
            if state is not None:
                # [x, y, z, yaw, x_vel, y_vel, z_vel, yaw_rate]
                print(f"Current State: x={state[0]:.2f}, y={state[1]:.2f}, z={state[2]:.2f}, yaw={state[3]:.2f}, x_vel={state[4]:.2f}, y_vel={state[5]:.2f}, z_vel={state[6]:.2f}, yaw_rate={state[7]:.2f}")
                break
            else:
                print("Waiting for current state...")
            await asyncio.sleep(2.0)
    asyncio.run(print_state())

async def latency_check(planner):
    await planner.connect_drone()
    await planner.test_gazebo_state()

async def speed_check(planner):
    await planner.connect_drone()
    await planner.speed_test()

async def main(planner):
    await planner.connect_drone()
    await planner.run_dwa_loop(goal=(10, 0), dt=0.1, stop_distance=1)

def collision_check_diff_speed_trajectories(horizon=3.0, w_dist=2.0, w_vel=-0.5, w_obs=6.0):
    planner = DynamicWindowApproachPlanner(w_dist, w_vel, w_obs)
    
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

    current_state = [0.0, 0.0, 0.0, 0.0]  # Start at origin, facing east (yaw=0)

    # Generate candidates and trajectories for different starting speeds
    speed_candidates = []
    speed_trajectories = []
    speed_collision_masks = []
    speed_scores = []
    
    starting_speeds = range(1, 13)  # 1 m/s to 12 m/s
    
    for speed in starting_speeds:
        print(f"Processing starting speed: {speed} m/s")
        
        # Sample velocities for this starting speed
        candidates = planner.sample_velocities(current_forward_vel=speed, current_yaw_rate=0)
        speed_candidates.append(candidates)
        
        # Predict trajectories
        trajectories = planner.trajectory_prediction_vectorized(current_state, candidates, time_horizon=horizon, dt=0.1)
        speed_trajectories.append(trajectories)
        
        # Check collisions
        collision_mask = planner.collision_checking_optimized(trajectories, obstacles, safety_distance=0.25)
        speed_collision_masks.append(collision_mask)
        
        # Score trajectories
        scores = planner.trajectory_scoring_optimized((10, 0), collision_mask, trajectories, candidates, obstacles)
        speed_scores.append(scores)
    
    # Create subplots for each starting speed
    n_speeds = len(starting_speeds)
    cols = 4  # 4 plots per row
    rows = (n_speeds + cols - 1) // cols  # Calculate needed rows
    
    fig, axes = plt.subplots(rows, cols, figsize=(20, 5 * rows))
    axes = axes.flatten() if rows > 1 else [axes] if n_speeds == 1 else axes
    
    import matplotlib.cm as cm
    import numpy as np
    
    for idx, (speed, trajectories, collision_mask, scores, candidates) in enumerate(
        zip(starting_speeds, speed_trajectories, speed_collision_masks, speed_scores, speed_candidates)):
        
        ax = axes[idx]
        
        # Filter out infinite scores for color mapping
        finite_scores = [score for score in scores if score != float('inf')]
        
        if finite_scores:
            min_score = min(finite_scores)
            max_score = max(finite_scores)
            score_range = max_score - min_score
            
            # Plot each trajectory with color based on score
            for i, (traj, score, collision) in enumerate(zip(trajectories, scores, collision_mask)):
                xs = [pt[0] for pt in traj]
                ys = [pt[1] for pt in traj]
                
                if collision or score == float('inf'):
                    # Plot collision trajectories in black
                    color = 'black'
                    alpha = 0.3
                    linewidth = 0.5
                else:
                    # Normalize score to 0-1 range for color mapping
                    if score_range > 0:
                        normalized_score = (score - min_score) / score_range
                    else:
                        normalized_score = 0.0
                    
                    # Use RdYlGn_r colormap (green=good, red=bad)
                    color = cm.RdYlGn_r(normalized_score)
                    alpha = 0.8
                    linewidth = 1.0
                
                ax.plot(xs, ys, color=color, alpha=alpha, linewidth=linewidth)
            
            # Highlight best trajectory
            if finite_scores:
                best_idx = scores.index(min(finite_scores))
                best_traj = trajectories[best_idx]
                xs = [pt[0] for pt in best_traj]
                ys = [pt[1] for pt in best_traj]
                ax.plot(xs, ys, color='lime', linewidth=3, alpha=1.0, label='Best')
        
        # Plot obstacles
        if obstacles:
            obs_xs = [o[0] for o in obstacles]
            obs_ys = [o[1] for o in obstacles]
            ax.scatter(obs_xs, obs_ys, c='blue', s=20, alpha=0.6)
        
        # Plot drone start position
        ax.scatter([current_state[0]], [current_state[1]], c='magenta', s=100, 
                  marker='o', zorder=5)
        
        # Plot goal
        ax.scatter([10], [0], c='gold', s=100, marker='*', zorder=5)
        
        # Formatting
        ax.set_title(f'Speed {speed} m/s\n({len([s for s in scores if s != float("inf")])}/{len(scores)} safe)')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(-5, 15)
        ax.set_ylim(-8, 8)
        
        # Add statistics
        safe_count = len([s for s in scores if s != float('inf')])
        collision_count = len(scores) - safe_count
        ax.text(0.02, 0.98, f'Safe: {safe_count}\nCollision: {collision_count}', 
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Hide extra subplots if any
    for idx in range(n_speeds, len(axes)):
        axes[idx].set_visible(False)
    
    plt.tight_layout()
    plt.suptitle(f'DWA Trajectories by Starting Speed\nHorizon: {horizon}s, Weights: dist={w_dist}, vel={w_vel}, obs={w_obs}', 
                 fontsize=16, y=1.02)
    plt.show()
    
    # Print summary statistics
    print(f"\n=== SUMMARY STATISTICS ===")
    for idx, (speed, scores) in enumerate(zip(starting_speeds, speed_scores)):
        safe_count = len([s for s in scores if s != float('inf')])
        total_count = len(scores)
        safe_percentage = (safe_count / total_count) * 100
        print(f"Speed {speed:2d} m/s: {safe_count:2d}/{total_count} safe ({safe_percentage:5.1f}%)")

if __name__ == "__main__":
    # test_velocity_sampler()
    # test_trajectory_prediction()
    # visualize_velocity_space()
    # test_scanner()
    # collision_check_trajectories()
    # collision_check_diff_speed_trajectories(3.0, 2.0, -0.5, 6.0)
    # test_current_state()
    planner = DynamicWindowApproachPlanner(2.0, -0.5, 6.0)
    asyncio.run(main(planner))
    # asyncio.run(latency_check(planner))
    # asyncio.run(speed_check(planner))