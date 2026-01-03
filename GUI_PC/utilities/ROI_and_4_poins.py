import os
# --- BẮT BUỘC: Dòng này giúp MSMF khởi động nhanh ---
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2
import json
import numpy as np
import time
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import sys
import math

# Add parent directory to path to find connection_manager
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from connection_manager import ConnectionManager, list_ports

# =========================
# CLASS APPLICATION
# =========================
class ROIAndCalibrationApp:
    def __init__(self):
        # Paths relative to parent directory (GUI_PC)
        self.base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        self.json_file = os.path.join(self.base_dir, "camera_calibration.json")
        self.train_images_dir = os.path.join(self.base_dir, "yolov8_train_images")
        
        # State variables
        self.current_mode = 'roi'  # 'roi' or 'calib'
        self.cfg = {}
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cap = None
        self.window_name = "ROI & 4 Points Setup"
        
        self.screen_width = 1920
        self.screen_height = 1080
        
        # Connection & Conveyor
        self.conn_manager = ConnectionManager()
        self.conveyor_running = False
        self.conveyor_speed = 40.0
        self.control_window = None
        
        # ROI State
        self.roi_rect = None
        self.roi_points = []
        self.drawing_roi = False
        
        # Calibration State
        self.calib_points = [] # List of tuples [(x, y), ...] relative to ROI
        self.selected_point_idx = -1 # Index of currently selected point (0-3)
        self.capture_count = 0
        
        # UI Variables
        self.point_vars = [] # List of dictionaries [{'x': Var, 'y': Var}, ...]
        
        # Initialize
        self.load_config()
        self.check_existing_images()
        self.init_camera()
        
    def load_config(self):
        """Load configuration from JSON."""
        try:
            with open(self.json_file, "r") as f:
                self.cfg = json.load(f)
            print("✓ Loaded configuration.")
            
            # Load ROI
            if "roi" in self.cfg and self.cfg["roi"]:
                r = self.cfg["roi"]
                if all(k in r for k in ["x", "y", "w", "h"]):
                    self.roi_rect = (r["x"], r["y"], r["w"], r["h"])
                elif all(k in r for k in ["x", "y", "width", "height"]):
                    self.roi_rect = (r["x"], r["y"], r["width"], r["height"])
                print(f"  ROI: {self.roi_rect}")

            # Load Calibration Points
            if "calibration" in self.cfg:
                c = self.cfg["calibration"]
                pts = c.get("image_points_roi_px") or c.get("points")
                if pts and len(pts) == 4:
                    self.calib_points = [tuple(p) for p in pts]
                    print(f"  Calibration Points: {self.calib_points}")
                    
            # Load Distortion Coefficients
            if "distortion" in self.cfg:
                d = self.cfg["distortion"]
                if "camera_matrix" in d and "dist_coeffs" in d:
                    self.camera_matrix = np.array(d["camera_matrix"], dtype=np.float32)
                    self.dist_coeffs = np.array(d["dist_coeffs"], dtype=np.float32)
                    print("  Distortion Coefficients Loaded.")

        except FileNotFoundError:
            print("⚠ Config file not found, starting fresh.")
            self.cfg = {"camera": {}}
        except Exception as e:
            print(f"✗ Error loading config: {e}")

    def save_config(self, auto_save_calib=False):
        """Save current configuration to JSON."""
        try:
            # Load existing to preserve other keys
            try:
                with open(self.json_file, "r") as f:
                    existing = json.load(f)
            except:
                existing = {}

            # Update ROI
            if self.roi_rect:
                x, y, w, h = self.roi_rect
                existing["roi"] = {"x": int(x), "y": int(y), "w": int(w), "h": int(h)}

            # Update Calibration Points
            if self.calib_points or auto_save_calib:
                if "calibration" not in existing:
                    existing["calibration"] = {}
                
                # Fill with (0,0) if not enough points
                points_to_save = []
                for i in range(4):
                    if i < len(self.calib_points):
                        points_to_save.append([float(self.calib_points[i][0]), float(self.calib_points[i][1])])
                    else:
                        points_to_save.append([0.0, 0.0])
                
                existing["calibration"]["image_points_roi_px"] = points_to_save

            with open(self.json_file, "w") as f:
                json.dump(existing, f, indent=2)
            
            if not auto_save_calib:
                print("✓ Configuration Saved to JSON.")
        except Exception as e:
            print(f"✗ Error saving config: {e}")

    def check_existing_images(self):
        """Check existing training images to set counter."""
        if os.path.exists(self.train_images_dir):
            files = [f for f in os.listdir(self.train_images_dir) if f.startswith("train_") and f.endswith(".jpg")]
            nums = []
            for f in files:
                try:
                    nums.append(int(f.replace("train_", "").replace(".jpg", "")))
                except:
                    pass
            if nums:
                self.capture_count = max(nums)
        else:
            os.makedirs(self.train_images_dir)

    def init_camera(self):
        """Initialize camera with settings."""
        print("Initializing Camera...")
        # Get index from config, default to 0 if missing
        cam_idx = self.cfg.get("camera", {}).get("index", 0)
        print(f"Opening Camera Index: {cam_idx}")

        self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_MSMF)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Apply Camera Settings from Config
        cam_cfg = self.cfg.get("camera", {})
        param_map = {
            "brightness": cv2.CAP_PROP_BRIGHTNESS,
            "contrast": cv2.CAP_PROP_CONTRAST,
            "saturation": cv2.CAP_PROP_SATURATION,
            "hue": cv2.CAP_PROP_HUE,
            "gain": cv2.CAP_PROP_GAIN,
            "exposure": cv2.CAP_PROP_EXPOSURE,
            "white_balance": cv2.CAP_PROP_WB_TEMPERATURE,
            "backlight": cv2.CAP_PROP_BACKLIGHT,
            "focus": cv2.CAP_PROP_FOCUS
        }
        for name, val in cam_cfg.items():
            if name in param_map:
                self.cap.set(param_map[name], val)

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera started: {w}x{h}")

    def get_screen_size_without_taskbar(self):
        try:
            import win32gui
            import win32api
            hwnd = win32gui.GetDesktopWindow()
            monitor = win32api.MonitorFromWindow(hwnd, win32api.MONITOR_DEFAULTTOPRIMARY)
            info = win32gui.GetMonitorInfo(monitor)
            work_area = info['Work']
            return work_area[2] - work_area[0], work_area[3] - work_area[1]
        except:
            return 1280, 720

    # =========================
    # LOGIC: MOUSE & COORDINATES
    # =========================

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events."""
        
        if self.current_mode == 'roi':
            self._handle_roi_mouse(event, x, y)
        elif self.current_mode == 'calib':
            self._handle_calib_mouse(event, x, y)

    def _handle_roi_mouse(self, event, x, y):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.roi_points = [(x, y)]
            self.drawing_roi = True
            print(f"[ROI] Start drawing at ({x}, {y})")
        
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing_roi:
            if len(self.roi_points) > 0:
                if len(self.roi_points) == 2:
                    self.roi_points[1] = (x, y)
                else:
                    self.roi_points.append((x, y))
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing_roi = False
            if len(self.roi_points) >= 2:
                x1, y1 = self.roi_points[0]
                x2, y2 = self.roi_points[-1]
                rx, ry = min(x1, x2), min(y1, y2)
                rw, rh = abs(x2 - x1), abs(y2 - y1)
                
                if rw > 10 and rh > 10:
                    self.roi_rect = (rx, ry, rw, rh)
                    print(f"✓ ROI Set: {self.roi_rect}")
                else:
                    print("⚠ ROI too small, ignored.")

    def _handle_calib_mouse(self, event, x, y):
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.roi_rect:
                print("⚠ Set ROI first!")
                return
            
            rx, ry, _, _ = self.roi_rect
            # Calculate coordinate relative to ROI
            px = float(x - rx)
            py = float(y - ry)
            
            # Check if we clicked near an existing point (Selection)
            clicked_idx = -1
            min_dist = 20.0 # Selection radius
            
            for i, (cx, cy) in enumerate(self.calib_points):
                dist = math.sqrt((cx - px)**2 + (cy - py)**2)
                if dist < min_dist:
                    min_dist = dist
                    clicked_idx = i
            
            if clicked_idx != -1:
                # Select existing point
                self.selected_point_idx = clicked_idx
                print(f"[CALIB] Selected Point {clicked_idx+1}")
                self.update_ui_coords() # Sync UI
            else:
                # Add new point if < 4
                if len(self.calib_points) < 4:
                    self.calib_points.append((px, py))
                    self.selected_point_idx = len(self.calib_points) - 1
                    print(f"✓ Added Point {len(self.calib_points)}: ({px:.1f}, {py:.1f})")
                    self.update_ui_coords()
                else:
                    print("⚠ Already have 4 points. Select a point to move it.")

    def capture_image(self, frame):
        """Capture training image."""
        if not self.roi_rect:
            return
        x, y, w, h = self.roi_rect
        if w < 640 or h < 640:
            print(f"⚠ ROI too small ({w}x{h}). Need >= 640x640.")
            return

        crop_x = x + (w - 640)
        crop_y = y
        crop_w = 640
        crop_h = 640
        if crop_x < 0: crop_x = 0
        if crop_y < 0: crop_y = 0
        
        cropped = frame[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w]
        
        self.capture_count += 1
        filename = os.path.join(self.train_images_dir, f"train_{self.capture_count:04d}.jpg")
        cv2.imwrite(filename, cropped)
        print(f"✓ Saved: {filename}")

    # --- Conveyor Wrappers ---
    def conveyor_start(self, forward=True):
        if not self.conn_manager.is_connected(): return
        cmd = f"CONVEYOR:START:{'FWD' if forward else 'REV'}"
        if self.conn_manager.send_command(cmd):
            self.conveyor_running = True
            self.update_ui_state()

    def conveyor_stop(self):
        if not self.conn_manager.is_connected(): return
        if self.conn_manager.send_command("CONVEYOR:STOP"):
            self.conveyor_running = False
            self.update_ui_state()

    def conveyor_set_speed(self, speed):
        if not self.conn_manager.is_connected(): return
        try:
            s = float(speed)
            if self.conn_manager.send_command(f"CONVEYOR:SET_SPEED:{s}"):
                self.conveyor_speed = s
                self.update_ui_state()
        except ValueError: pass

    # =========================
    # UI LOGIC & CONTROLS
    # =========================
    
    def update_ui_state(self):
        if self.control_window and hasattr(self, 'btn_conveyor_on'):
            try:
                if self.conn_manager.is_connected():
                    state_on = 'disabled' if self.conveyor_running else 'normal'
                    state_off = 'normal' if self.conveyor_running else 'disabled'
                    color_on = '#90EE90' if self.conveyor_running else '#4CAF50'
                    color_off = '#FF6B6B' if self.conveyor_running else '#CCCCCC'
                    self.btn_conveyor_on.config(state=state_on, bg=color_on)
                    self.btn_conveyor_off.config(state=state_off, bg=color_off)
                else:
                    self.btn_conveyor_on.config(state='disabled', bg='#CCCCCC')
                    self.btn_conveyor_off.config(state='disabled', bg='#CCCCCC')
            except: pass

    def update_connection_ui(self, connected, message=""):
        if self.control_window and hasattr(self, 'lbl_status'):
            try:
                if connected:
                    self.lbl_status.config(text=f"Connected: {message}", foreground="green")
                    self.btn_connect.config(text="Disconnect")
                    self.update_ui_state()
                else:
                    self.lbl_status.config(text=f"Disconnected: {message}", foreground="red")
                    self.btn_connect.config(text="Connect")
                    self.btn_conveyor_on.config(state='disabled')
                    self.btn_conveyor_off.config(state='disabled')
            except: pass

    # --- Calibration Point Controls ---
    
    def update_ui_coords(self):
        """Update Entry boxes from self.calib_points."""
        # Use after(0) to ensure thread safety with Tkinter
        if self.control_window:
            self.control_window.after(0, self._update_ui_coords_main_thread)
            
    def _update_ui_coords_main_thread(self):
        for i in range(4):
            x_var = self.point_vars[i]['x']
            y_var = self.point_vars[i]['y']
            
            if i < len(self.calib_points):
                px, py = self.calib_points[i]
                x_var.set(f"{px:.1f}")
                y_var.set(f"{py:.1f}")
            else:
                x_var.set("0.0")
                y_var.set("0.0")
        
        # Highlight selected row (optional, simplified for now)
    
    def on_manual_coord_change(self, event=None):
        """Read values from Entry boxes and update self.calib_points."""
        new_points = []
        for i in range(4):
            try:
                x = float(self.point_vars[i]['x'].get())
                y = float(self.point_vars[i]['y'].get())
                # Only add if non-zero or if it was previously set (simple validation)
                if i < len(self.calib_points) or (x != 0 or y != 0):
                     new_points.append((x, y))
            except ValueError:
                # Invalid number, ignore or keep old value
                if i < len(self.calib_points):
                    new_points.append(self.calib_points[i])
        
        # We don't want to accidentally shrink the list if user clears a box
        # But for now, let's just update existing indices
        for i in range(min(len(new_points), len(self.calib_points))):
            self.calib_points[i] = new_points[i]
            
        print("Updated points from Manual Input")

    def nudge_selected_point(self, dx, dy):
        """Move the currently selected point by dx, dy."""
        idx = self.selected_point_idx
        if 0 <= idx < len(self.calib_points):
            cx, cy = self.calib_points[idx]
            self.calib_points[idx] = (cx + dx, cy + dy)
            self.update_ui_coords()
            print(f"Nudged Point {idx+1} to ({cx+dx:.1f}, {cy+dy:.1f})")

    # --- Main Window Builder ---

    def run_control_window(self):
        self.control_window = tk.Tk()
        self.control_window.title("Control Panel")
        self.control_window.geometry("400x750")
        self.control_window.resizable(False, False)
        
        # 1. CDC
        frame_cdc = ttk.LabelFrame(self.control_window, text="Connection", padding=10)
        frame_cdc.pack(fill='x', padx=5, pady=5)
        self.com_var = tk.StringVar()
        cb = ttk.Combobox(frame_cdc, textvariable=self.com_var, values=list_ports(), state='readonly')
        cb.pack(side='top', fill='x', pady=2)
        self.btn_connect = ttk.Button(frame_cdc, text="Connect", command=self.on_connect_click)
        self.btn_connect.pack(side='top', fill='x')
        self.lbl_status = ttk.Label(frame_cdc, text="Disconnected", foreground="red")
        self.lbl_status.pack(side='top')
        
        # 2. Mode
        frame_mode = ttk.LabelFrame(self.control_window, text="Mode", padding=10)
        frame_mode.pack(fill='x', padx=5, pady=5)
        self.mode_var = tk.StringVar(value='roi')
        def on_mode_change():
            self.current_mode = self.mode_var.get()
            print(f">>> Mode: {self.current_mode.upper()}")
        ttk.Radiobutton(frame_mode, text="ROI Setup", variable=self.mode_var, value='roi', command=on_mode_change).pack(anchor='w')
        ttk.Radiobutton(frame_mode, text="Calibration (4 Points)", variable=self.mode_var, value='calib', command=on_mode_change).pack(anchor='w')
        
        # 3. Calibration Points (THE NEW PART)
        frame_points = ttk.LabelFrame(self.control_window, text="Calibration Points (ROI Relative)", padding=10)
        frame_points.pack(fill='x', padx=5, pady=5)
        
        self.point_vars = []
        for i in range(4):
            row = ttk.Frame(frame_points)
            row.pack(fill='x', pady=2)
            
            lbl = ttk.Label(row, text=f"P{i+1}:", width=4)
            lbl.pack(side='left')
            
            # X Entry
            x_var = tk.StringVar(value="0.0")
            e_x = ttk.Entry(row, textvariable=x_var, width=8)
            e_x.pack(side='left', padx=2)
            e_x.bind('<Return>', self.on_manual_coord_change)
            e_x.bind('<FocusOut>', self.on_manual_coord_change)
            e_x.bind('<Button-1>', lambda e, idx=i: self._set_selected(idx)) # Select on click
            
            # Y Entry
            y_var = tk.StringVar(value="0.0")
            e_y = ttk.Entry(row, textvariable=y_var, width=8)
            e_y.pack(side='left', padx=2)
            e_y.bind('<Return>', self.on_manual_coord_change)
            e_y.bind('<FocusOut>', self.on_manual_coord_change)
            e_y.bind('<Button-1>', lambda e, idx=i: self._set_selected(idx))

            self.point_vars.append({'x': x_var, 'y': y_var})
        
        # Nudge Controls
        frame_nudge = ttk.Frame(frame_points)
        frame_nudge.pack(pady=5)
        
        btn_up = ttk.Button(frame_nudge, text="▲", width=4, command=lambda: self.nudge_selected_point(0, -1))
        btn_down = ttk.Button(frame_nudge, text="▼", width=4, command=lambda: self.nudge_selected_point(0, 1))
        btn_left = ttk.Button(frame_nudge, text="◄", width=4, command=lambda: self.nudge_selected_point(-1, 0))
        btn_right = ttk.Button(frame_nudge, text="►", width=4, command=lambda: self.nudge_selected_point(1, 0))
        
        # Grid layout for arrows
        btn_up.grid(row=0, column=1)
        btn_left.grid(row=1, column=0)
        btn_right.grid(row=1, column=2)
        btn_down.grid(row=2, column=1)

        ttk.Label(frame_nudge, text="Select point above or click on image").grid(row=3, column=0, columnspan=3, pady=5)
        
        # 4. Conveyor
        frame_conv = ttk.LabelFrame(self.control_window, text="Conveyor", padding=10)
        frame_conv.pack(fill='x', padx=5, pady=5)
        self.btn_conveyor_on = tk.Button(frame_conv, text="START", bg='#4CAF50', fg='white', state='disabled', command=lambda: self.conveyor_start(True))
        self.btn_conveyor_on.pack(fill='x', pady=2)
        self.btn_conveyor_off = tk.Button(frame_conv, text="STOP", bg='#CCCCCC', fg='white', state='disabled', command=self.conveyor_stop)
        self.btn_conveyor_off.pack(fill='x', pady=2)
        
        frame_speed = ttk.Frame(frame_conv)
        frame_speed.pack(fill='x', pady=5)
        ttk.Label(frame_speed, text="Speed:").pack(side='left')
        self.speed_var = tk.StringVar(value=str(self.conveyor_speed))
        ttk.Entry(frame_speed, textvariable=self.speed_var, width=8).pack(side='left', padx=5)
        ttk.Button(frame_speed, text="Set", command=lambda: self.conveyor_set_speed(self.speed_var.get())).pack(side='left')
        
        # 5. Buttons
        frame_actions = ttk.Frame(self.control_window)
        frame_actions.pack(fill='x', padx=10, pady=10)
        ttk.Button(frame_actions, text="Save Config [S]", command=self.save_config).pack(fill='x', pady=2)
        ttk.Button(frame_actions, text="Clear ROI [X]", command=lambda: setattr(self, 'roi_rect', None)).pack(fill='x', pady=2)
        ttk.Button(frame_actions, text="Clear Points [D]", command=lambda: setattr(self, 'calib_points', [])).pack(fill='x', pady=2)

        self.conn_manager.add_connection_callback(self.on_connection_callback)
        self.control_window.protocol("WM_DELETE_WINDOW", self.on_close_window)
        
        # Initialize UI with loaded points
        self.update_ui_coords()
        
        self.control_window.mainloop()

    def _set_selected(self, idx):
        self.selected_point_idx = idx
        print(f"UI Selected Point {idx+1}")

    def on_connect_click(self):
        port = self.com_var.get()
        if not port: return
        if self.conn_manager.is_connected():
            self.conn_manager.disconnect_async()
        else:
            self.lbl_status.config(text="Connecting...", foreground="orange")
            self.btn_connect.config(state='disabled')
            self.conn_manager.connect_async(port, baudrate=115200)

    def on_connection_callback(self, success, message):
        if self.control_window:
            self.control_window.after(0, lambda: self._handle_connection_result(success, message))

    def _handle_connection_result(self, success, message):
        self.btn_connect.config(state='normal')
        self.update_connection_ui(success, message)

    def on_close_window(self):
        if self.conn_manager.is_connected():
            self.conveyor_stop()
            self.conn_manager.disconnect_async()
        self.control_window.destroy()

    def run(self):
        t = threading.Thread(target=self.run_control_window, daemon=True)
        t.start()
        time.sleep(1.0)
        
        self.screen_width, self.screen_height = self.get_screen_size_without_taskbar()
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.screen_width, self.screen_height)
        cv2.moveWindow(self.window_name, 0, 0)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        print(">>> MAIN LOOP STARTED <<<")
        print("KEYS: [R] ROI Mode, [C] Calib Mode, [P] Capture, [S] Save, [X] Clear ROI, [D] Clear Points, [Q] Quit")

        while True:
            try:
                if self.cap is None or not self.cap.isOpened():
                    time.sleep(0.1)
                    continue
                    
                ret, frame = self.cap.read()
                if not ret:
                    print("Camera error")
                    time.sleep(0.5)
                    continue
                    
                if self.camera_matrix is not None and self.dist_coeffs is not None:
                    frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
                    
                display = frame.copy()
                
                # --- DRAWING ---
                
                # 1. ROI
                if self.roi_rect:
                    x, y, w, h = self.roi_rect
                    cv2.rectangle(display, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(display, "ROI", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    if w >= 640 and h >= 640:
                        cx = x + (w - 640)
                        cv2.rectangle(display, (cx, y), (cx+640, y+640), (255, 0, 0), 1)
                
                if self.drawing_roi and len(self.roi_points) >= 2:
                    cv2.rectangle(display, self.roi_points[0], self.roi_points[-1], (255, 255, 0), 1)
                    
                # 2. Calibration Points
                if self.roi_rect:
                    rx, ry, _, _ = self.roi_rect
                    
                    # Draw lines first
                    if len(self.calib_points) > 1:
                        pts_abs = [ (int(p[0]+rx), int(p[1]+ry)) for p in self.calib_points ]
                        for i in range(len(pts_abs)-1):
                            cv2.line(display, pts_abs[i], pts_abs[i+1], (0, 255, 255), 1)
                        if len(pts_abs) == 4:
                            cv2.line(display, pts_abs[3], pts_abs[0], (0, 255, 255), 1)

                    # Draw points
                    for i, pt in enumerate(self.calib_points):
                        dx = int(pt[0] + rx)
                        dy = int(pt[1] + ry)
                        
                        # Highlight selected
                        color = (0, 0, 255) # Red default
                        radius = 5
                        if i == self.selected_point_idx:
                            color = (0, 255, 255) # Yellow selected
                            radius = 8
                            cv2.circle(display, (dx, dy), radius+2, (0,0,0), 2) # Black outline
                        
                        cv2.circle(display, (dx, dy), radius, color, -1)
                        cv2.putText(display, str(i+1), (dx+12, dy-12), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

                # 3. HUD
                cv2.rectangle(display, (0, 0), (300, 100), (0,0,0), -1)
                cv2.putText(display, f"MODE: {self.current_mode.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display, f"Points: {len(self.calib_points)}/4", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
                cv2.putText(display, f"Captures: {self.capture_count}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
                
                cv2.imshow(self.window_name, display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): break
                elif key == ord('r'): 
                    self.current_mode = 'roi'; self.mode_var.set('roi')
                elif key == ord('c'): 
                    self.current_mode = 'calib'; self.mode_var.set('calib')
                elif key == ord('x'): self.roi_rect = None
                elif key == ord('d'): self.calib_points = []; self.update_ui_coords()
                elif key == ord('s'): self.save_config()
                elif key == ord('p'): self.capture_image(frame)
                
            except Exception as e:
                print(f"Loop error: {e}")
                time.sleep(1)
        
        if self.cap: self.cap.release()
        cv2.destroyAllWindows()
        if self.conn_manager.is_connected():
            self.conveyor_stop()
            self.conn_manager.disconnect_async()

if __name__ == "__main__":
    app = ROIAndCalibrationApp()
    app.run()