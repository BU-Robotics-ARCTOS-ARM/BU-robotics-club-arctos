import tkinter as tk
from tkinter import ttk
import sys
import os
import numpy as np

# Force Matplotlib to use the TkAgg backend
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Add the project root to the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from arctos.kinematics import Kinematics
from arctos.config import DH_PARAMS, JOINTS

COLORS = ['#000000', '#3498db', '#2ecc71', '#f1c40f', '#e74c3c', '#9b59b6', '#d35400']

class KinematicsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Arctos 3D Control Center")
        self.root.geometry("1250x850")

        self.kin = Kinematics(DH_PARAMS)
        self.joint_vars = []
        self.joint_entries = []
        self.last_safe_angles = [0.0] * 6
        
        self.setup_ui()
        self.init_plot()
        self.update_visuals()

    def setup_ui(self):
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        controls = ttk.Frame(self.root, padding="15")
        controls.grid(row=0, column=0, sticky="ns")

        ttk.Label(controls, text="Robot Configuration", font=('Arial', 16, 'bold')).pack(pady=(0, 20))

        # Joint Control Loop
        for i, (name, can_id, ratio, min_a, max_a) in enumerate(JOINTS):
            # Container for each joint
            j_frame = tk.LabelFrame(controls, text=f" Joint {i+1} - {name} ", padx=10, pady=5, fg=COLORS[i%len(COLORS)])
            j_frame.pack(fill="x", pady=5)
            
            var = tk.DoubleVar(value=0.0)
            self.joint_vars.append(var)
            
            # Slider
            slider = ttk.Scale(j_frame, from_=min_a, to=max_a, orient="horizontal", 
                               variable=var, command=lambda val, idx=i: self.on_slider_change(val, idx), length=180)
            slider.pack(side="left", padx=(0, 10))
            
            # Entry Box for direct input
            entry = ttk.Entry(j_frame, width=8, justify="center")
            entry.insert(0, "0.00")
            entry.pack(side="left")
            entry.bind('<Return>', lambda e, idx=i: self.on_entry_confirm(idx))
            entry.bind('<FocusOut>', lambda e, idx=i: self.on_entry_confirm(idx))
            self.joint_entries.append(entry)
            
            ttk.Label(j_frame, text="°").pack(side="left")

        ttk.Separator(controls, orient="horizontal").pack(fill="x", pady=20)
        
        # Pose Info
        info_frame = ttk.LabelFrame(controls, text=" End Effector Pose ", padding="10")
        info_frame.pack(fill="x", pady=10)
        self.pose_lbl = ttk.Label(info_frame, text="", font=('Courier New', 11, 'bold'), justify="left")
        self.pose_lbl.pack()
        
        # Safety Status
        self.status_frame = tk.Frame(controls, pady=12, bg="#e0ffe0", relief="sunken", borderwidth=1)
        self.status_frame.pack(fill="x", pady=10)
        self.safety_lbl = tk.Label(self.status_frame, text="STATUS: SAFE", fg="green", bg="#e0ffe0", font=('Arial', 11, 'bold'))
        self.safety_lbl.pack()

        # Action Buttons
        btn_frame = ttk.Frame(controls)
        btn_frame.pack(fill="x", pady=10)
        ttk.Button(btn_frame, text="Reset to Zero", command=self.reset).pack(side="left", expand=True, padx=2)
        ttk.Button(btn_frame, text="Center View", command=self.reset_view).pack(side="left", expand=True, padx=2)

    def init_plot(self):
        viz_panel = ttk.Frame(self.root, padding="10")
        viz_panel.grid(row=0, column=1, sticky="nsew")
        
        self.fig = plt.figure(figsize=(9, 9), facecolor='#f0f0f0')
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_panel)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def on_slider_change(self, val, idx):
        angles = [v.get() for v in self.joint_vars]
        self.process_angle_update(angles, idx)

    def on_entry_confirm(self, idx):
        try:
            val = float(self.joint_entries[idx].get())
            # Clamp to limits
            min_a, max_a = JOINTS[idx][3], JOINTS[idx][4]
            val = max(min_a, min(max_a, val))
            
            self.joint_vars[idx].set(val)
            angles = [v.get() for v in self.joint_vars]
            self.process_angle_update(angles, idx)
        except ValueError:
            self.sync_entries_to_vars()

    def process_angle_update(self, angles, trigger_idx):
        collision_reason = self.check_collision(angles)
        
        if collision_reason:
            # Block and Revert
            self.joint_vars[trigger_idx].set(self.last_safe_angles[trigger_idx])
            self.safety_lbl.config(text=f"BLOCK: {collision_reason.upper()}", fg="white")
            self.status_frame.config(bg="#e74c3c")
        else:
            self.last_safe_angles = angles.copy()
            self.safety_lbl.config(text="STATUS: SAFE", fg="green")
            self.status_frame.config(bg="#e0ffe0")
            self.sync_entries_to_vars()
            self.update_visuals()

    def sync_entries_to_vars(self):
        """Update entry text to match slider variables."""
        for i, var in enumerate(self.joint_vars):
            # Only update if the entry doesn't have focus (to allow typing)
            if self.root.focus_get() != self.joint_entries[i]:
                self.joint_entries[i].delete(0, tk.END)
                self.joint_entries[i].insert(0, f"{var.get():.2f}")

    def check_collision(self, angles):
        try:
            transforms = self.kin.get_all_joint_transforms(angles)
            pts = [t[:3, 3] for t in transforms]
            
            # 1. Floor Collision
            for i in range(len(pts) - 1):
                p_start, p_end = pts[i], pts[i+1]
                p_mid = (p_start + p_end) / 2.0
                if p_end[2] < -1.0 or p_mid[2] < -1.0:
                    return f"Floor (Link {i})"

            # 2. Pedestal Collision
            r_ped, h_ped = 85.0, 280.0
            for i in range(2, len(pts)):
                p = pts[i]
                if p[2] < h_ped:
                    if np.sqrt(p[0]**2 + p[1]**2) < r_ped:
                        return "Base Pedestal"
            return None
        except: return "Math Error"

    def update_visuals(self):
        angles = self.last_safe_angles
        transforms = self.kin.get_all_joint_transforms(angles)
        
        # 1. Update 3D
        self.draw_arm(transforms)
        
        # 2. Update Text
        pose = self.kin.forward(angles)
        self.pose_lbl.config(text=f"X: {pose.x:8.2f}  Roll:  {pose.roll:7.2f}°\n"
                                  f"Y: {pose.y:8.2f}  Pitch: {pose.pitch:7.2f}°\n"
                                  f"Z: {pose.z:8.2f}  Yaw:   {pose.yaw:7.2f}°")

    def draw_arm(self, transforms):
        x = [t[0, 3] for t in transforms]
        y = [t[1, 3] for t in transforms]
        z = [t[2, 3] for t in transforms]

        elev, azim = self.ax.elev, self.ax.azim
        self.ax.clear()
        
        # Draw Base Pedestal
        r, h = 80, 287.87
        zc = np.linspace(0, h, 8)
        tc = np.linspace(0, 2*np.pi, 16)
        tg, zg = np.meshgrid(tc, zc)
        self.ax.plot_surface(r*np.cos(tg), r*np.sin(tg), zg, alpha=0.2, color='#95a5a6')

        # Draw Skeleton
        self.ax.plot(x, y, z, '-', color='#2c3e50', linewidth=7, alpha=0.8)
        
        # Draw Joints
        for i in range(len(x)):
            self.ax.scatter(x[i], y[i], z[i], color=COLORS[i%len(COLORS)], 
                            s=150, edgecolors='white', linewidth=1.5, zorder=10)

        # Draw Floor Grid
        limit = 650
        fx, fy = np.meshgrid(np.linspace(-limit, limit, 10), np.linspace(-limit, limit, 10))
        self.ax.plot_surface(fx, fy, np.zeros_like(fx)-2, alpha=0.05, color='#3498db')

        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(-50, limit)
        self.ax.set_xlabel('X'), self.ax.set_ylabel('Y'), self.ax.set_zlabel('Z')
        self.ax.view_init(elev=elev, azim=azim)
        self.canvas.draw_idle()

    def reset_view(self):
        self.ax.view_init(elev=25, azim=45)
        self.canvas.draw_idle()

    def reset(self):
        for v in self.joint_vars: v.set(0.0)
        self.last_safe_angles = [0.0] * 6
        self.sync_entries_to_vars()
        self.update_visuals()

if __name__ == "__main__":
    root = tk.Tk()
    app = KinematicsApp(root)
    root.mainloop()
