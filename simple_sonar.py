#!/usr/bin/env python3

import os
# Set protobuf implementation to python before importing gz modules
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import time
import math
try:
    import gz.transport13 as gz_transport
except ImportError:
    print("Error: gz-transport Python bindings not found")
    print("Install with: sudo apt install python3-gz-transport13")
    exit(1)

class SimpleLidarReader:
    def __init__(self):
        self.node = gz_transport.Node()
        self.latest_scan = None
        
    def lidar_callback(self, msg):
        """Simple callback to store latest scan data"""
        self.latest_scan = msg
        print(f"Received scan with {len(msg.ranges)} points")
        
        # Calculate angles for each point
        angle_min = msg.angle_min  # Starting angle (usually negative)
        angle_max = msg.angle_max  # Ending angle (usually positive)
        angle_increment = msg.angle_step  # Angular resolution
        
        # Print some basic info
        valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r))]
        if valid_ranges:
            print(f"  Min distance: {min(valid_ranges):.2f}m")
            print(f"  Max distance: {max(valid_ranges):.2f}m")
            print(f"  Valid points: {len(valid_ranges)}")
            print(f"  Angle range: {math.degrees(angle_min):.1f}° to {math.degrees(angle_max):.1f}°")
            
            # Example: Find obstacles in front (around 0 degrees)
            self.analyze_directions(msg)

    def analyze_directions(self, msg):
        """Analyze obstacles by direction"""
        angle_min = msg.angle_min
        angle_increment = msg.angle_step
        
        for i, range_val in enumerate(msg.ranges):
            # Calculate angle for this point
            angle = angle_min + (i * angle_increment)
            angle_deg = math.degrees(angle)
            
            # Skip invalid readings
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            # Check for obstacles in specific directions
            if abs(angle_deg) < 45 and range_val < 1.0:  # Front (±10°)
                print(f"Obstacle ahead: {range_val:.2f}m at {angle_deg:.1f}°")
            elif -90 < angle_deg < -80 and range_val < 1.0:  # Right side
                print(f"Obstacle on right: {range_val:.2f}m at {angle_deg:.1f}°")
            elif 80 < angle_deg < 90 and range_val < 1.0:  # Left side
                print(f"Obstacle on left: {range_val:.2f}m at {angle_deg:.1f}°")

    def get_direction_sectors(self, msg, num_sectors=8):
        """Divide scan into directional sectors"""
        angle_min = msg.angle_min
        angle_increment = msg.angle_step
        
        sectors = {}
        sector_size = (2 * math.pi) / num_sectors
        
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            angle = angle_min + (i * angle_increment)
            # Normalize angle to 0-2π
            normalized_angle = (angle + 2*math.pi) % (2*math.pi)
            
            sector = int(normalized_angle / sector_size)
            direction_name = ["Front", "Front-Left", "Left", "Back-Left", 
                            "Back", "Back-Right", "Right", "Front-Right"][sector]
            
            if direction_name not in sectors:
                sectors[direction_name] = []
            sectors[direction_name].append(range_val)
        
        return sectors

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

def main():
    """Main function"""
    reader = SimpleLidarReader()
    
    if not reader.start_listening():
        print("Failed to start LiDAR reader")
        return
    
    print("Listening for LiDAR data... Press Ctrl+C to stop")
    
    try:
        while True:
            if reader.latest_scan is None:
                print("Waiting for LiDAR data...")
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == "__main__":
    main()