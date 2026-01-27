import pygame
import numpy as np
import math

class DWAVisualizer:
    def __init__(self, width=800, height=800, scale=40):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("DWA Planner Visualizer")
        self.clock = pygame.time.Clock()
        self.scale = scale  # Pixels per meter
        self.offset = np.array([width // 2, height // 2])
        self.font = pygame.font.SysFont("Arial", 18)

    def world_to_screen(self, pos):
        # Flips Y because Pygame Y grows downwards
        screen_x = int(self.offset[0] + pos[0] * self.scale)
        screen_y = int(self.offset[1] - pos[1] * self.scale)
        return (screen_x, screen_y)

    def render(self, drone_state, goal, obstacles, trajectories, collision_mask, best_idx):
        self.screen.fill((30, 30, 30))  # Dark Background

        # 1. Draw Obstacles (LiDAR points)
        for obs in obstacles:
            pygame.draw.circle(self.screen, (255, 50, 50), self.world_to_screen(obs), 3)

        # 2. Draw Trajectories
        for i, traj in enumerate(trajectories):
            if not traj: continue
            points = [self.world_to_screen((p[0], p[1])) for p in traj]
            
            color = (200, 0, 0) if collision_mask[i] else (0, 200, 0)
            width = 3 if i == best_idx else 1
            if i == best_idx: color = (0, 255, 255) # Cyan for best
            
            if len(points) > 1:
                pygame.draw.lines(self.screen, color, False, points, width)

        # 3. Draw Goal
        pygame.draw.circle(self.screen, (255, 215, 0), self.world_to_screen(goal), 10)

        # 4. Draw Drone
        drone_pos = self.world_to_screen((drone_state[0], drone_state[1]))
        pygame.draw.circle(self.screen, (255, 255, 255), drone_pos, 8)
        
        # Draw Heading Line
        yaw_rad = math.radians(drone_state[3])
        head_x = drone_state[0] + math.cos(yaw_rad) * 0.5
        head_y = drone_state[1] + math.sin(yaw_rad) * 0.5
        pygame.draw.line(self.screen, (255, 255, 255), drone_pos, self.world_to_screen((head_x, head_y)), 2)

        # 5. UI Text
        vel_text = self.font.render(f"Vel: {drone_state[4]:.2f} m/s", True, (255, 255, 255))
        self.screen.blit(vel_text, (20, 20))

        pygame.display.flip()
        self.clock.tick(60)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()