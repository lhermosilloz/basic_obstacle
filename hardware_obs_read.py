#!/usr/bin/env python3

import os
# Set protobuf implementation to python before importing gz modules
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import time
import math
import threading
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None
    print("Warning: pymavlink not installed. Hardware LiDAR reading will not work.")

# --- Hardware LiDAR Reader using MAVLink ---
class HardwareLidarReader:
    def __init__(self, connection_str="udpin:0.0.0.0:14551"):
        if mavutil is None:
            raise ImportError("pymavlink is required for hardware LiDAR reading.")
        self.connection_str = connection_str
        self.master = mavutil.mavlink_connection(self.connection_str)
        self.latest_distance = None
        self.running = False
        self.thread = None
        self.heartbeat_thread = None
        self.heartbeat_running = False

        self.latest_obs = None
        self.running_obs = False
        self.thread_obs = None

    def _listen(self):
        while self.running:
            msg = self.master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)
            if msg:
                self.latest_distance = msg

    def _listen_obs(self):
        while self.running_obs:
            msg = self.master.recv_match(type='OBSTACLE_DISTANCE', blocking=True, timeout=1)
            if msg:
                self.latest_obs = msg

    def _send_heartbeat(self):
        while self.heartbeat_running:
            # Send a heartbeat as a GCS (MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID)
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(1)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()

        self.running_obs = True
        self.thread_obs = threading.Thread(target=self._listen_obs, daemon=True)
        self.thread_obs.start()

        # Start heartbeat thread
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(target=self._send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

        self.running_obs = False
        if self.thread_obs:
            self.thread_obs.join()

        self.heartbeat_running = False
        if self.heartbeat_thread:
            self.heartbeat_thread.join()

    def get_obstacle_distances(self):
        """Return the latest obstacle distances in meters, or None if not available."""
        if self.latest_obs:
            return [d / 100.0 for d in self.latest_obs.distances]
        return None
    
    def print_obstacle_distances(self):
        obs = self.get_obstacle_distances()
        if obs is not None:
            print(f"[MAVLink] Latest obstacle distances: {[f'{d:.2f}m' for d in obs]}")
        else:
            print("[MAVLink] No obstacle distance received yet.")

    def get_distance(self):
        """Return the latest distance in meters, or None if not available."""
        if self.latest_distance:
            # The distance is in cm, convert to meters
            return self.latest_distance.current_distance / 100.0
        return None

    def print_distance(self):
        dist = self.get_distance()
        if dist is not None:
            print(f"[MAVLink] Latest LiDAR distance: {dist:.2f} m")
        else:
            print("[MAVLink] No LiDAR distance received yet.")
        
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

def main():
    print("Hardware (MAVLink udpin:0.0.0.0:14551)")

    if mavutil is None:
        print("pymavlink not installed. Please install with: pip install pymavlink")
        return
    reader = HardwareLidarReader()
    reader.start()
    print("Listening for hardware LiDAR data via MAVLink... Press Ctrl+C to stop")
    try:
        while True:
            reader.print_obstacle_distances()
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.stop()

if __name__ == "__main__":
    main()