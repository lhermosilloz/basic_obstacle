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
    print("Warning: pymavlink not installed. Hardware 2D LiDAR reading will not work.")

# --- Hardware 2D LiDAR Reader using MAVLink ---
class Hardware2DLidarReader:
    def __init__(self, connection_str="udpin:0.0.0.0:14550"):
        if mavutil is None:
            raise ImportError("pymavlink is required for hardware 2D LiDAR reading.")
        self.connection_str = connection_str
        self.master = mavutil.mavlink_connection(self.connection_str)
        self.latest_msg = None
        self.running = False
        self.thread = None
        self.heartbeat_thread = None
        self.heartbeat_running = False

    def _listen(self):
        while self.running:
            msg = self.master.recv_match(type='OBSTACLE_DISTANCE', blocking=True, timeout=1)
            if msg:
                self.latest_msg = msg

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

        # Start heartbeat thread
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(target=self._send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

        self.heartbeat_running = False
        if self.heartbeat_thread:
            self.heartbeat_thread.join()

    def get_distances(self):
        """Return the latest distance in meters, or None if not available."""
        if self.latest_msg:
            # The distance is in cm, convert to meters
            return [d / 100.0 for d in self.latest_msg.distances]
        return None

    def print_distances(self):
        dist = self.get_distances()
        if dist is not None:
            print(f"[MAVLink] Latest 2D LiDAR distances: {dist}")
        else:
            print("[MAVLink] No 2D LiDAR distance received yet.")

def main():
    print("Hardware (MAVLink udpin:0.0.0.0:14550)")

    if mavutil is None:
        print("pymavlink not installed. Please install with: pip install pymavlink")
        return
    reader = Hardware2DLidarReader()
    reader.start()
    print("Listening for hardware 2D LiDAR data via MAVLink... Press Ctrl+C to stop")
    try:
        while True:
            reader.print_distances()
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.stop()

if __name__ == "__main__":
    main()