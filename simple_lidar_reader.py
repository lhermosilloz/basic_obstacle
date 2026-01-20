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
        # print(f"Received scan with {len(msg.ranges)} points")
        
        # Print some basic info
        valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r))]
        # if valid_ranges:
        #     print(f"  Min distance: {min(valid_ranges):.2f}m")
        #     print(f"  Max distance: {max(valid_ranges):.2f}m")
        #     print(f"  Valid points: {len(valid_ranges)}")
        
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