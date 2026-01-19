#!/usr/bin/env python3

import asyncio
import math
import numpy as np
try:
    import gz.transport13 as gz_transport
except ImportError:
    print("Error: gz-transport Python bindings not found")
    print("Install with: sudo apt install python3-gz-transport13")
    exit(1)

class LidarReader:
    def __init__(self):
        # Initialize Gazebo transport node
        self.node = gz_transport.Node()
        self.lidar_data = None
        self.obstacles = []
        
        # LiDAR parameters (adjust based on your sensor configuration)
        self.max_range = 30.0  # meters
        self.min_range = 0.1   # meters
        self.obstacle_threshold = 10.0  # consider anything closer than this as obstacle
        
    def lidar_callback(self, msg):
        """Callback function for LiDAR data"""
        try:
            # Parse LaserScan message
            ranges = msg.ranges
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            
            self.lidar_data = {
                'ranges': ranges,
                'angle_min': angle_min,
                'angle_increment': angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max
            }
            
            # Process LiDAR data to extract obstacles
            self.process_lidar_data()
            
        except Exception as e:
            print(f"Error processing LiDAR data: {e}")
    
    def process_lidar_data(self):
        """Convert LiDAR scan to obstacle positions in NED coordinates"""
        if not self.lidar_data:
            return
            
        obstacles = []
        ranges = self.lidar_data['ranges']
        angle_min = self.lidar_data['angle_min']
        angle_increment = self.lidar_data['angle_increment']
        
        for i, range_val in enumerate(ranges):
            # Skip invalid readings
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            # Skip readings beyond our threshold
            if range_val > self.obstacle_threshold or range_val < self.min_range:
                continue
                
            # Calculate angle for this reading
            angle = angle_min + i * angle_increment
            
            # Convert polar to Cartesian coordinates (NED frame)
            # Assuming LiDAR X-axis points forward (North in NED)
            north = range_val * math.cos(angle)  # X in sensor frame = North in NED
            east = range_val * math.sin(angle)   # Y in sensor frame = East in NED
            down = 0.0  # Assuming 2D LiDAR at same altitude
            
            obstacles.append([north, east, down])
        
        self.obstacles = obstacles
        
    def subscribe_to_lidar(self):
        """Subscribe to the Gazebo LiDAR topic"""
        topic = "/world/default/model/x500_lidar_2d_0/link/lidar_sensor_link/sensor/lidar/scan"
        
        # Try different message type imports based on Gazebo version
        try:
            # For newer Gazebo versions
            try:
                from gz.msgs10.laserscan_pb2 import LaserScan
            except ImportError:
                try:
                    from gz.msgs9.laserscan_pb2 import LaserScan
                except ImportError:
                    from gz.msgs.laserscan_pb2 import LaserScan
            
            # Subscribe with message type as first argument
            success = self.node.subscribe(LaserScan, topic, self.lidar_callback)
            
            if success:
                print(f"Successfully subscribed to {topic}")
                return True
            else:
                print(f"Failed to subscribe to {topic}")
                return False
                
        except ImportError as e:
            print(f"Error importing LaserScan message type: {e}")
            print("Available packages to try:")
            print("  sudo apt install python3-gz-msgs10")
            print("  sudo apt install python3-gz-msgs9") 
            print("  sudo apt install python3-gz-msgs")
            return False
        except Exception as e:
            print(f"Error subscribing to topic: {e}")
            return False
    
    def get_obstacles(self):
        """Return current obstacles in NED coordinates"""
        return self.obstacles.copy()
    
    def print_lidar_info(self):
        """Print current LiDAR information"""
        if self.lidar_data:
            print(f"LiDAR ranges: {len(self.lidar_data['ranges'])} points")
            print(f"Obstacles detected: {len(self.obstacles)}")
            if self.obstacles:
                print("Closest obstacles (North, East, Down):")
                for i, obs in enumerate(self.obstacles[:5]):  # Show first 5
                    distance = math.sqrt(obs[0]**2 + obs[1]**2)
                    print(f"  {i+1}: ({obs[0]:.2f}, {obs[1]:.2f}, {obs[2]:.2f}) - {distance:.2f}m")
        else:
            print("No LiDAR data received yet")

async def main():
    """Main function to test LiDAR reading"""
    lidar_reader = LidarReader()
    
    # Subscribe to LiDAR topic
    if not lidar_reader.subscribe_to_lidar():
        print("Failed to subscribe to LiDAR topic. Is Gazebo running?")
        return
    
    print("Reading LiDAR data... Press Ctrl+C to stop")
    
    try:
        while True:
            lidar_reader.print_lidar_info()
            await asyncio.sleep(1.0)  # Print info every second
            
    except KeyboardInterrupt:
        print("\nStopping LiDAR reader...")

if __name__ == "__main__":
    asyncio.run(main())