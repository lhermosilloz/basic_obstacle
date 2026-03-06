#!/usr/bin/env python3
"""
DWA Parameter Control Dashboard
Real-time parameter tuning interface for DWA planner
"""
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import asyncio
import json
import os
from dwa_planner import DynamicWindowApproachPlanner

class DWAControlDashboard:
    def __init__(self, planner=None):
        self.planner = planner
        self.mission_thread = None
        self.mission_active = False
        self.root = tk.Tk()
        self.root.title("DWA Parameter Control Dashboard")
        self.root.geometry("800x1000")
        
        # Parameter definitions with [min, max, default, step]
        self.parameters = {
            # Primary weights
            'w_dist': [0.0, 10.0, 4.0, 0.1],
            'w_vel': [-5.0, 5.0, -0.5, 0.1], 
            'w_obs': [0.0, 10.0, 1.0, 0.1],
            
            # Physical limits
            'max_hor_accel': [1.0, 15.0, 5.0, 0.5],
            'max_yaw_accel': [5.0, 100.0, 20.0, 5.0],
            'max_forward_vel': [0.5, 15.0, 3.0, 0.1],
            'max_yaw_rate': [10.0, 120.0, 60.0, 5.0],
            
            # Sampling resolution
            'forward_vel_samples': [3, 15, 8, 1],
            'yaw_rate_samples': [3, 20, 10, 1],
            'slow_turn_samples': [3, 20, 10, 1],
            'slow_turn_range': [0.1, 5.0, 0.25, 0.05],
            
            # Trajectory prediction
            'time_horizon': [0.5, 10.0, 3.0, 0.1],
            
            # Collision detection
            'safety_distance': [0.1, 2.0, 0.28, 0.01],
            'trajectory_check_spacing': [1, 10, 3, 1],
            
            # Obstacle cost function
            'lethal_dist': [0.1, 3.0, 1.0, 0.1],
            'inflation_radius': [1.0, 10.0, 3.0, 0.1],
            'max_obstacle_cost': [0.5, 10.0, 3.25, 0.25],
        }
        
        self.sliders = {}
        self.labels = {}
        self.status_label = None  # Initialize this first
        
        self.setup_gui()
        
    def setup_gui(self):
        # Main frame with scrollbar
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Canvas and scrollbar for scrollable content
        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Title
        title_label = ttk.Label(scrollable_frame, text="DWA Parameter Control Dashboard", 
                               font=("Arial", 16, "bold"))
        title_label.pack(pady=(0, 20))
        
        # Status label (create this first)
        self.status_label = ttk.Label(scrollable_frame, text="Status: Dashboard ready", foreground="green")
        self.status_label.pack(pady=10)
        
        # Create parameter groups
        groups = {
            "Trajectory Scoring Weights": ['w_dist', 'w_vel', 'w_obs'],
            "Physical Limits": ['max_hor_accel', 'max_yaw_accel', 'max_forward_vel', 'max_yaw_rate'],
            "Sampling Resolution": ['forward_vel_samples', 'yaw_rate_samples', 'slow_turn_samples', 'slow_turn_range'],
            "Trajectory Prediction": ['time_horizon'],
            "Collision Detection": ['safety_distance', 'trajectory_check_spacing'],
            "Obstacle Cost Function": ['lethal_dist', 'inflation_radius', 'max_obstacle_cost']
        }
        
        for group_name, param_names in groups.items():
            self.create_parameter_group(scrollable_frame, group_name, param_names)
        
        # Control buttons
        button_frame = ttk.Frame(scrollable_frame)
        button_frame.pack(pady=20)
        
        ttk.Button(button_frame, text="Save Preset", command=self.save_preset).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Load Preset", command=self.load_preset).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Reset Defaults", command=self.reset_defaults).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Start Mission", command=self.start_mission).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="End Mission", command=self.end_mission).pack(side=tk.LEFT, padx=5)
        
    def create_parameter_group(self, parent, group_name, param_names):
        # Group frame
        group_frame = ttk.LabelFrame(parent, text=group_name, padding=10)
        group_frame.pack(fill=tk.X, pady=5)
        
        for param_name in param_names:
            if param_name in self.parameters:
                self.create_parameter_slider(group_frame, param_name)
    
    def create_parameter_slider(self, parent, param_name):
        min_val, max_val, default_val, step = self.parameters[param_name]
        
        param_frame = ttk.Frame(parent)
        param_frame.pack(fill=tk.X, pady=2)
        
        # Label
        label_text = param_name.replace('_', ' ').title()
        ttk.Label(param_frame, text=label_text, width=20).pack(side=tk.LEFT)
        
        # Value label
        value_label = ttk.Label(param_frame, text=f"{default_val}", width=8)
        value_label.pack(side=tk.RIGHT)
        self.labels[param_name] = value_label
        
        # Slider
        slider = ttk.Scale(param_frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                          command=lambda val, name=param_name: self.on_parameter_change(name, val))
        slider.set(default_val)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        self.sliders[param_name] = slider
        
    def on_parameter_change(self, param_name, value):
        # Round value to appropriate step
        min_val, max_val, default_val, step = self.parameters[param_name]
        
        # Convert to appropriate type (int for samples, float for others)
        if 'samples' in param_name or 'spacing' in param_name:
            rounded_value = int(float(value))
        else:
            rounded_value = round(float(value) / step) * step
        
        # Update label
        self.labels[param_name].config(text=f"{rounded_value}")
        
        # Update planner if connected
        if self.planner and hasattr(self.planner, 'update_parameters'):
            try:
                self.planner.update_parameters(**{param_name: rounded_value})
                if self.status_label:  # Check if status_label exists
                    self.status_label.config(text=f"Updated {param_name} = {rounded_value}", foreground="green")
            except Exception as e:
                if self.status_label:
                    self.status_label.config(text=f"Error updating {param_name}: {str(e)}", foreground="red")
        else:
            if self.status_label:
                self.status_label.config(text="No planner connected", foreground="orange")
    
    def connect_planner(self, planner):
        """Connect the dashboard to a DWA planner instance"""
        self.planner = planner
        if self.status_label:
            self.status_label.config(text="Planner connected!", foreground="green")
        
        # Update sliders to match planner's current values
        for param_name in self.parameters:
            if hasattr(planner, param_name):
                current_value = getattr(planner, param_name)
                self.sliders[param_name].set(current_value)
                self.labels[param_name].config(text=f"{current_value}")
    
    def save_preset(self):
        """Save current parameter values to a JSON file"""
        preset = {}
        for param_name, slider in self.sliders.items():
            preset[param_name] = slider.get()
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(preset, f, indent=2)
                self.status_label.config(text=f"Preset saved to {filename}", foreground="green")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save preset: {str(e)}")
    
    def load_preset(self):
        """Load parameter values from a JSON file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    preset = json.load(f)
                
                for param_name, value in preset.items():
                    if param_name in self.sliders:
                        self.sliders[param_name].set(value)
                        self.on_parameter_change(param_name, value)
                
                self.status_label.config(text=f"Preset loaded from {filename}", foreground="green")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load preset: {str(e)}")
    
    def reset_defaults(self):
        """Reset all parameters to their default values"""
        for param_name, (min_val, max_val, default_val, step) in self.parameters.items():
            if param_name in self.sliders:
                self.sliders[param_name].set(default_val)
                self.on_parameter_change(param_name, default_val)
        
        self.status_label.config(text="Reset to defaults", foreground="blue")
    
    def start_mission(self):
        """Start the drone mission in background thread"""
        if not self.planner:
            if self.status_label:
                self.status_label.config(text="No planner connected!", foreground="red")
            return
            
        if hasattr(self, 'mission_thread') and self.mission_thread and self.mission_thread.is_alive():
            if self.status_label:
                self.status_label.config(text="Mission already running!", foreground="orange")
            return
        
        def run_mission():
            try:
                # Import here to avoid circular imports
                from mavsdk.offboard import VelocityBodyYawspeed
                
                async def mission():
                    waypoints = [(10, 0), (0, 0), (10, -10)]
                    
                    await self.planner.connect_drone()
                    await self.planner.arm()
                    await self.planner.takeoff()
                    await asyncio.sleep(10.0)
                    
                    # Start offboard mode
                    print("Setting initial velocity setpoint...")
                    await self.planner.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    print("Starting offboard mode...")
                    await self.planner.drone.offboard.start()
                    
                    self.mission_active = True
                    
                    for waypoint in waypoints:
                        if not self.mission_active:  # Check if mission was ended
                            break
                        print(f"Navigating to waypoint: {waypoint}")
                        while self.mission_active:
                            reached = await self.planner.run_dwa_loop(goal=waypoint, dt=0.1, stop_distance=1)
                            if reached:
                                print(f"Reached waypoint: {waypoint}")
                                break
                    
                    # Land and disarm
                    await self.planner.land()
                    await self.planner.disarm()
                    self.mission_active = False
                
                asyncio.run(mission())
                
            except Exception as e:
                print(f"Mission failed: {e}")
                self.mission_active = False
        
        self.mission_thread = threading.Thread(target=run_mission, daemon=True)
        self.mission_thread.start()
        
        if self.status_label:
            self.status_label.config(text="Mission started! Adjust parameters in real-time", foreground="blue")
    
    def end_mission(self):
        """End the current mission and land/disarm the drone"""
        if not self.planner:
            if self.status_label:
                self.status_label.config(text="No planner connected!", foreground="red")
            return
            
        if not self.mission_active:
            if self.status_label:
                self.status_label.config(text="No mission running!", foreground="orange")
            return
        
        def emergency_land():
            try:
                async def land_and_disarm():
                    print("Emergency landing initiated...")
                    # Stop the mission loop
                    self.mission_active = False
                    
                    # Stop current velocity commands
                    from mavsdk.offboard import VelocityBodyYawspeed
                    await self.planner.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    
                    # Land and disarm
                    await self.planner.land()
                    print("Landing...")
                    await self.planner.disarm()
                    print("Mission ended and drone disarmed")
                
                asyncio.run(land_and_disarm())
                
            except Exception as e:
                print(f"Emergency landing failed: {e}")
        
        # Run emergency landing in separate thread
        emergency_thread = threading.Thread(target=emergency_land, daemon=True)
        emergency_thread.start()
        
        if self.status_label:
            self.status_label.config(text="Mission ended - Landing and disarming...", foreground="red")
    
    def run(self):
        """Start the GUI main loop"""
        self.root.mainloop()

# Example usage functions
def create_dashboard_only():
    """Create dashboard without connecting to planner (for testing GUI)"""
    dashboard = DWAControlDashboard()
    dashboard.run()

def create_dashboard_with_planner():
    """Create dashboard and connect to a DWA planner"""
    # Create planner with default parameters
    planner = DynamicWindowApproachPlanner(
        dist=4.0, vel=-0.5, obs=1.0,
        mx_hr_acc=5.0, mx_yaw_acc=20.0, mx_fwd_vel=3.0, mx_yaw_rate=60.0,
        fwd_samples=8, yaw_samples=10, slow_turn_samples=10, slow_turn_range=0.25,
        time_horizon=3.0, dt=0.1, safety_distance=0.28, traj_check_spacing=3,
        lethal_dist=1.0, inflation_radius=3.0, max_obstacle_cost=3.25
    )
    
    # Create dashboard and connect planner
    dashboard = DWAControlDashboard()
    dashboard.connect_planner(planner)
    dashboard.run()
    
    return dashboard, planner

if __name__ == "__main__":
    # Add import for file dialog
    import tkinter.filedialog
    
    # Run the dashboard
    create_dashboard_with_planner()