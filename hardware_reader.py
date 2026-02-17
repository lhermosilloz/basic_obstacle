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
        self.latest_height = None
        # For decay visualization
        self.decay_points = []  # List of (x, y, timestamp)
        self.decay_time = 2  # seconds
        self.running_obs = False
        self.running_height = False
        self.thread_obs = None
        self.thread_height = None
        self.heartbeat_thread = None
        self.heartbeat_running = False

    def _listen_obs(self):
        while self.running_obs:
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

    def _listen_height(self):
        while self.running_height:
            msg = self.master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)
            if msg:
                self.latest_height = msg

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
        self.running_obs = True
        self.thread_obs = threading.Thread(target=self._listen_obs, daemon=True)
        self.thread_obs.start()

        self.running_height = True
        self.thread_height = threading.Thread(target=self._listen_height, daemon=True)
        self.thread_height.start()

        # Start heartbeat thread
        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(target=self._send_heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def stop(self):
        self.running_obs = False
        if self.thread_obs:
            self.thread_obs.join()

        self.running_height = False
        if self.thread_height:
            self.thread_height.join()

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
        
    def get_height(self):
        """Return the latest height in meters, or None if not available."""
        if self.latest_height:
            # The distance is in cm, convert to meters
            return self.latest_height.current_distance / 100.0
        return None
    
    def print_height(self):
        height = self.get_height()
        if height is not None:
            print(f"[MAVLink] Latest height: {height:.2f} m")
        else:
            print("[MAVLink] No height data received yet.")

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
    # Flip axes: heading is upwards (y positive is forward)
    plot.setXRange(-10, 10)
    plot.setYRange(0, 10)
    scatter = pg.ScatterPlotItem(size=8, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 120))
    plot.addItem(scatter)

    # Add a legend label for closest object
    legend_label = pg.TextItem(text="Closest: -- m", color=(255,255,255), anchor=(0,1))
    plot.addItem(legend_label)
    legend_label.setPos(-10, 10)  # Top-left corner

    # Add a height bar to the right side
    height_bar_plot = win.addPlot(title="Height", row=0, col=1)
    height_bar_plot.setXRange(0, 1)
    height_bar_plot.setYRange(0, 10)
    height_bar_plot.setAspectLocked(False)
    height_bar_plot.hideAxis('bottom')
    height_bar_plot.setFixedWidth(100)
    # BarGraphItem for height
    bar = pg.BarGraphItem(x=[0.5], height=[0], width=0.8, brush=pg.mkBrush(255, 0, 0, 150))
    height_bar_plot.addItem(bar)
    # Height label
    height_label = pg.TextItem(text="Height: -- m", color=(255,255,255), anchor=(0.5,0))
    height_bar_plot.addItem(height_label)
    height_label.setPos(0.5, 10)  # Top of the bar

    # For OBSTACLE_DISTANCE, angles are usually evenly spaced over a field of view
    def update():
        # Get points with decay
        points = reader.get_decay_points()
        # Flip axes: heading is upwards (y positive is forward)
        flipped_points = [(y, x, age_frac) for (x, y, age_frac) in points]
        if flipped_points:
            spots = []
            for x, y, age_frac in flipped_points:
                alpha = int(255 * (1 - age_frac))
                spots.append({'pos': (x, y), 'brush': pg.mkBrush(0, 255, 0, alpha)})
            scatter.setData(spots)
        else:
            scatter.setData([])

        # Show closest object distance in legend
        dists = reader.get_distances()
        if dists:
            valid = [d for d in dists if d > 0 and not np.isinf(d) and not np.isnan(d)]
            if valid:
                closest = min(valid)
                legend_label.setText(f"Closest: {closest:.2f} m")
            else:
                legend_label.setText("Closest: -- m")
        else:
            legend_label.setText("Closest: -- m")

        # Update height bar
        height = reader.get_height()
        if height is not None:
            bar.setOpts(height=[height])
            height_label.setText(f"Height: {height:.2f} m")
            height_label.setPos(0.5, height)  # Move label to top of bar
        else:
            bar.setOpts(height=[0])
            height_label.setText("Height: -- m")
            height_label.setPos(0.5, 0)  # Move label to bottom of bar

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