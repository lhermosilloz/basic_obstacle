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

from PyQt5.QtWidgets import QApplication, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QFrame, QGraphicsPathItem
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPalette, QColor, QPainterPath
# --- Hardware 2D LiDAR Reader using MAVLink ---
class Hardware2DLidarReader:
    def __init__(self, connection_str="udpin:0.0.0.0:14550"):
        if mavutil is None:
            raise ImportError("pymavlink is required for hardware 2D LiDAR reading.")
        self.connection_str = connection_str
        self.master = None
        self.connection_error = None
        self.latest_msg = None
        self.latest_height = None
        # For decay visualization
        self.decay_points = []  # List of (x, y, timestamp)
        self.decay_time = 10  # seconds
        self.running_obs = False
        self.running_height = False
        self.thread_obs = None
        self.thread_height = None
        self.heartbeat_thread = None
        self.heartbeat_running = False
        # Connection status
        self.connected = False
        self.last_message_time = 0
        self.connection_timeout = 5  # seconds
        
        # Try to establish connection with error handling
        self._connect()
    
    def _connect(self):
        """Try to connect with fallback options."""
        connection_options = [
            self.connection_str
        ]
        
        for conn_str in connection_options:
            try:
                print(f"Trying connection: {conn_str}")
                self.master = mavutil.mavlink_connection(conn_str)
                self.connection_str = conn_str
                self.connection_error = None
                print(f"Successfully connected to: {conn_str}")
                break
            except Exception as e:
                print(f"Failed to connect to {conn_str}: {str(e)}")
                self.connection_error = str(e)
                continue
        
        if self.master is None:
            print("Could not establish any MAVLink connection.")
            print("   This is normal if no drone is connected.")
            print("   The application will run in demo mode showing simulated data.")

    def _listen_obs(self):
        while self.running_obs:
            if self.master is None:
                time.sleep(1)
                continue
            try:
                msg = self.master.recv_match(type='OBSTACLE_DISTANCE', blocking=True, timeout=1)
                if msg:
                    self.latest_msg = msg
                    self.last_message_time = time.time()
                    self.connected = True
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
            except Exception as e:
                print(f"Error in _listen_obs: {e}")
                time.sleep(1)

    def _listen_height(self):
        while self.running_height:
            if self.master is None:
                time.sleep(1)
                continue
            try:
                msg = self.master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)
                if msg:
                    self.latest_height = msg
                    self.last_message_time = time.time()
                    self.connected = True
            except Exception as e:
                print(f"Error in _listen_height: {e}")
                time.sleep(1)

    def _send_heartbeat(self):
        while self.heartbeat_running:
            try:
                if self.master:
                    # Send a heartbeat as a GCS (MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID)
                    self.master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
            except Exception as e:
                print(f"Error sending heartbeat: {e}")
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
    
    def is_connected(self):
        """Check if we're receiving data from the drone."""
        if self.last_message_time == 0:
            return False
        return (time.time() - self.last_message_time) < self.connection_timeout
    
    def get_demo_data(self):
        """Generate demo LiDAR data for visualization when no drone is connected."""
        if self.master is not None:
            return []  # Only show demo when not connected
            
        # Generate some realistic obstacle patterns
        now = time.time()
        demo_obstacles = []
        
        # Add some walls at different distances
        for angle in np.linspace(0, 2*np.pi, 72):  # 5-degree increments
            # Create walls at different distances with some noise
            distance = 5 + 2 * np.sin(angle * 3) + 0.5 * np.random.random()
            if distance > 2:  # Only obstacles further than 2m
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                demo_obstacles.append((x, y, np.random.random() * 0.3))  # Random age up to 30%
        
        return demo_obstacles
    
    def get_demo_height(self):
        """Generate demo height data."""
        if self.master is not None:
            return None
        # Simulate varying height between 2-8 meters
        return 4 + 2 * np.sin(time.time() * 0.5) + 0.5 * np.random.random()



def create_status_widget():
    """Create a status widget with connection and data info."""
    status_widget = QWidget()
    status_layout = QVBoxLayout()
    
    # Title
    title_label = QLabel("Drone Hardware Reader")
    title_label.setStyleSheet("""
        QLabel { 
            font-size: 18px; 
            font-weight: bold; 
            color: #2E86AB;
            padding: 10px;
            background-color: #F8F9FA;
            border-radius: 5px;
            margin-bottom: 10px;
        }
    """)
    title_label.setAlignment(Qt.AlignCenter)
    
    # Connection status
    conn_label = QLabel("Connection: Connecting...")
    conn_label.setStyleSheet("""
        QLabel { 
            font-size: 12px; 
            color: #FF6B35;
            padding: 5px;
            background-color: #FFF;
            border: 1px solid #DDD;
            border-radius: 3px;
        }
    """)
    
    # Data status
    data_label = QLabel("LiDAR: No data | Height: No data")
    data_label.setStyleSheet("""
        QLabel { 
            font-size: 12px; 
            color: #666;
            padding: 5px;
            background-color: #FFF;
            border: 1px solid #DDD;
            border-radius: 3px;
        }
    """)
    
    status_layout.addWidget(title_label)
    status_layout.addWidget(conn_label)
    status_layout.addWidget(data_label)
    status_layout.addStretch()
    
    status_widget.setLayout(status_layout)
    status_widget.setFixedWidth(250)
    status_widget.setStyleSheet("QWidget { background-color: #F5F5F5; }")
    
    return status_widget, conn_label, data_label

def main():
    # Create QApplication with better styling
    app = QApplication([])
    app.setStyle('Fusion')  # Use Fusion style for better appearance
    
    # Set application palette for dark theme
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)

    print("Enhanced Drone Hardware Reader (MAVLink)")
    print("   Supports multiple connection types and demo mode")

    if mavutil is None:
        print("pymavlink not installed. Please install with: pip install pymavlink")
        return
        
    reader = Hardware2DLidarReader()
    reader.start()
    
    if reader.master:
        print(f"Connected successfully! Listening for hardware data...")
    else:
        print("Running in demo mode - showing simulated data")
        print("   Connect your drone to see real sensor data")
    print("   Press Ctrl+C to stop")

    # Create main window with layout
    main_widget = QWidget()
    main_widget.setWindowTitle("Enhanced Drone Hardware Reader")
    main_widget.resize(1200, 800)
    main_layout = QHBoxLayout()
    
    # Create status panel
    status_widget, conn_label, data_label = create_status_widget()
    main_layout.addWidget(status_widget)
    
    # --- Enhanced PyQtGraph Visualization Setup ---
    graphics_widget = QWidget()
    graphics_layout = QVBoxLayout()
    
    # Create graphics layout
    win = pg.GraphicsLayoutWidget()
    graphics_layout.addWidget(win)
    graphics_widget.setLayout(graphics_layout)
    main_layout.addWidget(graphics_widget, stretch=1)
    
    main_widget.setLayout(main_layout)
    main_widget.show()
    
    # Main plot for LiDAR with improved styling
    lidar_plot = win.addPlot(title="LiDAR Scan (Top-Down View)", row=0, col=0)
    lidar_plot.setAspectLocked(True)
    lidar_plot.setLabel('left', 'Forward (m)', **{'color': '#FFF', 'font-size': '12pt'})
    lidar_plot.setLabel('bottom', 'Right (m)', **{'color': '#FFF', 'font-size': '12pt'})
    lidar_plot.setXRange(-10, 10)
    lidar_plot.setYRange(0, 15)
    lidar_plot.showGrid(x=True, y=True, alpha=0.3)
    
    # Add range circles for better orientation
    for radius in [2, 5, 8]:
        circle = QPainterPath()
        circle.addEllipse(-radius, -radius, 2*radius, 2*radius)
        circle_item = QGraphicsPathItem(circle)
        circle_item.setPen(pg.mkPen(color='gray', width=1, style=Qt.DashLine))
        lidar_plot.addItem(circle_item)
    
    # LiDAR points scatter plot with improved styling
    lidar_scatter = pg.ScatterPlotItem(size=6, pen=pg.mkPen(None))
    lidar_plot.addItem(lidar_scatter)
    
    # Info displays with better styling
    closest_label = pg.TextItem(text="Closest: -- m", color='#00FF00', anchor=(0,1))
    closest_label.setFont(QFont("Arial", 12, QFont.Bold))
    lidar_plot.addItem(closest_label)
    closest_label.setPos(-9.5, 14.5)
    
    scan_info_label = pg.TextItem(text="Scan: -- points", color='#FFAA00', anchor=(1,1))
    scan_info_label.setFont(QFont("Arial", 10))
    lidar_plot.addItem(scan_info_label)
    scan_info_label.setPos(9.5, 14.5)
    
    # Enhanced height visualization
    height_plot = win.addPlot(title="Altitude", row=0, col=1)
    height_plot.setXRange(0, 1)
    height_plot.setYRange(0, 15)
    height_plot.setLabel('left', 'Height (m)', **{'color': '#FFF', 'font-size': '12pt'})
    height_plot.hideAxis('bottom')
    height_plot.setFixedWidth(120)
    height_plot.showGrid(y=True, alpha=0.3)
    
    # Height bar with gradient effect
    height_bar = pg.BarGraphItem(x=[0.5], height=[0], width=0.6, 
                                brush=pg.mkBrush('#FF4444'))
    height_plot.addItem(height_bar)
    
    # Height text
    height_text = pg.TextItem(text="0.0 m", color='#FFFFFF', anchor=(0.5, 0.5))
    height_text.setFont(QFont("Arial", 12, QFont.Bold))
    height_plot.addItem(height_text)
    
    def update_visualization():
        # Update connection status
        is_connected = reader.is_connected()
        if reader.master is None:
            conn_label.setText("Demo Mode: No MAVLink connection")
            conn_label.setStyleSheet("""
                QLabel { 
                    font-size: 12px; 
                    color: #FF8C00;
                    padding: 5px;
                    background-color: #FFF3CD;
                    border: 1px solid #FF8C00;
                    border-radius: 3px;
                }
            """)
        elif reader.connection_error:
            conn_label.setText(f"Connection Error: {reader.connection_str}")
            conn_label.setStyleSheet("""
                QLabel { 
                    font-size: 12px; 
                    color: #DC3545;
                    padding: 5px;
                    background-color: #F8D7DA;
                    border: 1px solid #DC3545;
                    border-radius: 3px;
                }
            """)
        elif is_connected:
            conn_label.setText(f"Connected: {reader.connection_str}")
            conn_label.setStyleSheet("""
                QLabel { 
                    font-size: 12px; 
                    color: #28A745;
                    padding: 5px;
                    background-color: #D4F6D4;
                    border: 1px solid #28A745;
                    border-radius: 3px;
                }
            """)
        else:
            conn_label.setText(f"Waiting for data: {reader.connection_str}")
            conn_label.setStyleSheet("""
                QLabel { 
                    font-size: 12px; 
                    color: #FF8C00;
                    padding: 5px;
                    background-color: #FFF3CD;
                    border: 1px solid #FF8C00;
                    border-radius: 3px;
                }
            """)
        
        # Update LiDAR visualization with decay effect
        points = reader.get_decay_points()
        demo_points = reader.get_demo_data()
        
        # Combine real and demo data
        all_points = points if points else demo_points
        flipped_points = [(y, x, age_frac) for (x, y, age_frac) in all_points]
        
        if flipped_points:
            spots = []
            for x, y, age_frac in flipped_points:
                # Create color gradient from bright green (new) to dark red (old)
                intensity = 1 - age_frac
                if demo_points and not points:  # Demo mode
                    red = int(100 + 100 * age_frac)
                    green = int(100 + 100 * intensity)
                    blue = int(200 + 55 * intensity)
                    alpha = int(150 * intensity)
                else:  # Real data
                    red = int(255 * (0.2 + 0.8 * age_frac))
                    green = int(255 * intensity)
                    blue = int(50 * intensity)
                    alpha = int(200 * intensity)
                
                spots.append({'pos': (x, y), 'brush': pg.mkBrush(red, green, blue, alpha), 
                             'size': 8 - 3 * age_frac})
            lidar_scatter.setData(spots)
        else:
            lidar_scatter.setData([])

        # Update closest distance and scan info
        distances = reader.get_distances()
        lidar_data_available = False
        
        if distances:
            valid_distances = [d for d in distances if d > 0 and not np.isinf(d) and not np.isnan(d)]
            if valid_distances:
                closest = min(valid_distances)
                closest_label.setText(f"Closest: {closest:.2f} m")
                scan_info_label.setText(f"Scan: {len(valid_distances)} pts")
                lidar_data_available = True
            else:
                closest_label.setText("Closest: No valid data")
                scan_info_label.setText("Scan: 0 pts")
        elif demo_points:  # Show demo data info
            demo_distances = [np.sqrt(x**2 + y**2) for x, y, _ in demo_points]
            if demo_distances:
                closest = min(demo_distances)
                closest_label.setText(f"Demo: {closest:.2f} m")
                scan_info_label.setText(f"Demo: {len(demo_distances)} pts")
                lidar_data_available = True
        else:
            closest_label.setText("Closest: -- m")
            scan_info_label.setText("Scan: -- pts")

        # Update height with enhanced visualization
        height = reader.get_height()
        demo_height = reader.get_demo_height()
        actual_height = height if height is not None else demo_height
        
        height_data_available = False
        if actual_height is not None:
            # Color coding for height bar
            if actual_height < 2:
                color = '#FF4444'  # Red for low altitude
            elif actual_height < 5:
                color = '#FFAA44'  # Orange for medium altitude
            else:
                color = '#44AA44'  # Green for high altitude
                
            height_bar.setOpts(height=[actual_height], brush=pg.mkBrush(color))
            
            if height is not None:
                height_text.setText(f"{actual_height:.1f} m")
            else:
                height_text.setText(f"{actual_height:.1f} m (demo)")
            
            height_text.setPos(0.5, actual_height + 0.5)
            height_data_available = True
        else:
            height_bar.setOpts(height=[0])
            height_text.setText("-- m")
            height_text.setPos(0.5, 0.5)
        
        # Update data status
        if reader.master is None:
            lidar_status = "Demo mode"
            height_status = "Demo mode"
        else:
            lidar_status = "Active" if lidar_data_available else "No data"
            height_status = "Active" if height_data_available else "No data"
        
        data_label.setText(f"LiDAR: {lidar_status} | Height: {height_status}")

    # Update timer with faster refresh for smoother animations
    timer = QtCore.QTimer()
    timer.timeout.connect(update_visualization)
    timer.start(50)  # Update at 20 Hz for smoother experience

    # Start Qt event loop
    try:
        app.exec_()
    except KeyboardInterrupt:
        print("\nStopping...")
        reader.stop()

if __name__ == "__main__":
    main()