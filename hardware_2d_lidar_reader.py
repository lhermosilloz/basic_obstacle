#!/usr/bin/env python3

import os
# Set protobuf implementation to python before importing gz modules
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import time
import math
import threading
import numpy as np
# PyQtGraph and Qt imports for real-time visualization
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None
    print("Warning: pymavlink not installed. Hardware 2D LiDAR reading will not work.")

from PyQt5.QtWidgets import QApplication
# --- Hardware 2D LiDAR Reader using MAVLink ---
class Hardware2DLidarReader:
    def __init__(self, connection_str="udpin:0.0.0.0:14550"):
        if mavutil is None:
            raise ImportError("pymavlink is required for hardware 2D LiDAR reading.")
        self.connection_str = connection_str
        self.master = mavutil.mavlink_connection(self.connection_str)
        self.latest_msg = None
        # For decay visualization
        self.decay_points = []  # List of (x, y, timestamp)
        self.decay_time = 2  # seconds
        self.running = False
        self.thread = None
        self.heartbeat_thread = None
        self.heartbeat_running = False

    def _listen(self):
        while self.running:
            msg = self.master.recv_match(type='OBSTACLE_DISTANCE', blocking=True, timeout=1)
            if msg:
                self.latest_msg = msg
                # Update decay points
                distances = [d / 100.0 for d in msg.distances]
                n = len(distances)
                # Angle calculation (same as in update)
                if hasattr(msg, 'increment') and msg.increment > 0:
                    angles = np.arange(msg.angle_offset, msg.angle_offset + n * msg.increment, msg.increment)
                    angles = np.radians(angles / 100.0)
                else:
                    angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
                xs = np.array(distances) * np.cos(angles)
                ys = np.array(distances) * np.sin(angles)
                now = time.time()
                # Add new points with timestamp
                for x, y in zip(xs, ys):
                    self.decay_points.append((x, y, now))
                # Remove old points
                self.decay_points = [(x, y, t) for (x, y, t) in self.decay_points if now - t < self.decay_time]

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
            return [d / 100.0 for d in self.latest_msg.distances]
        return None

    def get_decay_points(self):
        """Return list of (x, y, age_frac) for points within decay window, where age_frac is 0 (new) to 1 (old)."""
        now = time.time()
        points = []
        for x, y, t in self.decay_points:
            age = now - t
            if age < self.decay_time:
                age_frac = age / self.decay_time
                points.append((x, y, age_frac))
        return points

    def print_distances(self):
        dist = self.get_distances()
        if dist is not None:
            print(f"[MAVLink] Latest 2D LiDAR distances (size: {len(dist)}): {dist}")
        else:
            print("[MAVLink] No 2D LiDAR distance received yet.")

def main():
    # Create QGuiApplication before any Qt widgets
    app = QApplication([])

    print("Hardware (MAVLink udpin:0.0.0.0:14550)")

    if mavutil is None:
        print("pymavlink not installed. Please install with: pip install pymavlink")
        return
    reader = Hardware2DLidarReader()
    reader.start()
    print("Listening for hardware 2D LiDAR data via MAVLink... Press Ctrl+C to stop")

    # --- PyQtGraph Visualization Setup ---
    win = pg.GraphicsLayoutWidget(show=True, title="2D LiDAR Real-Time Visualization")
    win.resize(600, 600)
    plot = win.addPlot(title="LiDAR Scan (Top-Down View)")
    plot.setAspectLocked(True)
    plot.setXRange(-10, 10)
    plot.setYRange(-10, 10)
    scatter = pg.ScatterPlotItem(size=8, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 120))
    plot.addItem(scatter)

    # For OBSTACLE_DISTANCE, angles are usually evenly spaced over a field of view
    def update():
        # Get points with decay
        points = reader.get_decay_points()
        if points:
            spots = []
            for x, y, age_frac in points:
                # Fade alpha from 255 (new) to 0 (old)
                alpha = int(255 * (1 - age_frac))
                spots.append({'pos': (x, y), 'brush': pg.mkBrush(0, 255, 0, alpha)})
            scatter.setData(spots)
        else:
            scatter.setData([])

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(30)  # Update at ~33 Hz

    # Start Qt event loop
    try:
        app.exec_()
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.stop()

if __name__ == "__main__":
    main()