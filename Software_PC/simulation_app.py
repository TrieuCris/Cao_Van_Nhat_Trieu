import sys
import time
import math
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QGroupBox, QGridLayout, QDoubleSpinBox,
                             QSlider, QButtonGroup, QRadioButton)
from PyQt6.QtCore import QTimer, Qt
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# Import project modules
import constants as C
from kinematics import DeltaKinematics, MotionPlannerTrapezoidal
import matplotlib.pyplot as plt

# =============================================================================
# CLASS: DELTA DYNAMICS
# =============================================================================
class DeltaDynamics:
    """
    Tính toán động lực học cơ bản cho Delta Robot để ước lượng Moment (Torque).
    Sử dụng phương pháp Jacobian tĩnh + Gia tốc quán tính đơn giản.
    """
    def __init__(self, kinematics):
        self.k = kinematics
        self.g = 9.81 # m/s^2

        # Thông số vật lý mặc định (có thể chỉnh từ GUI)
        self.mass_payload = 0.5   # kg
        self.mass_upper_arm = 0.3 # kg (rf)
        self.mass_lower_arm = 0.1 # kg (re) - tính 1 thanh (có 2 thanh song song -> x2)
        self.mass_platform = 0.15 # kg

        # Góc xoay của 3 trụ (0, 120, 240 độ)
        self.phi = [0, math.radians(120), math.radians(240)]
        
        # Unit vector của trục quay motor (nằm ngang, vuông góc với hướng radius)
        self.axis_u = []
        for p in self.phi:
            self.axis_u.append(np.array([-math.sin(p), math.cos(p), 0.0]))
            
        # --- Pre-calculate Geometry in METERS ---
        # Chuyển đổi tất cả kích thước từ mm sang m để tính Torque ra N.m
        self.f_m = self.k.f / 1000.0
        self.e_m = self.k.e / 1000.0
        self.rf_m = self.k.rf / 1000.0
        self.re_m = self.k.re / 1000.0

    def compute_torque(self, theta_deg, velocity_vec, accel_vec):
        """
        Tính toán Torque tại 3 khớp động cơ.
        :param theta_deg: List 3 góc khớp [th1, th2, th3] (độ)
        :param velocity_vec: Vector vận tốc bàn kẹp [vx, vy, vz] (m/s)
        :param accel_vec: Vector gia tốc bàn kẹp [ax, ay, az] (m/s^2)
        :return: List 3 torque [tau1, tau2, tau3] (Nm)
        """
        # Chuyển đổi sang radian
        theta = [math.radians(t) for t in theta_deg]
        
        # Lấy tọa độ End Effector (P0) từ FK (đang là mm) -> Đổi sang Mét
        fk_res = self.k.forward_kinematics(*theta_deg)
        if not fk_res:
            return [0, 0, 0]
        P0_mm = np.array(fk_res) 
        P0 = P0_mm / 1000.0 # (x, y, z) tính bằng Mét

        # Tính toán vector Jacobian nghịch (Inverse Jacobian)
        J_inv = np.zeros((3, 3))
        
        R_base = self.f_m / (2 * math.sqrt(3))
        R_plat = self.e_m / (2 * math.sqrt(3))
        
        for i in range(3):
            # Tọa độ khớp vai (Base Joint) B_i (Mét)
            Bi = np.array([R_base * math.cos(self.phi[i]), R_base * math.sin(self.phi[i]), 0])
            
            # Tọa độ khớp khuỷu (Elbow) Ji (Mét)
            H = R_base + self.rf_m * math.cos(theta[i])
            Zi = -self.rf_m * math.sin(theta[i])
            Ji = np.array([H * math.cos(self.phi[i]), H * math.sin(self.phi[i]), Zi])
            
            # Tọa độ khớp bệ (Platform Joint) Pi (Mét)
            Pi = P0 + np.array([R_plat * math.cos(self.phi[i]), R_plat * math.sin(self.phi[i]), 0])
            
            # Vector b_i (Cánh tay dưới): Từ Ji đến Pi
            bi = Pi - Ji
            
            # Vector a_i (Cánh tay trên): Từ Bi đến Ji
            ai = Ji - Bi
            
            # Vector vận tốc tức thời của Elbow do Theta gây ra: w_i = u_i x a_i
            wi = np.cross(self.axis_u[i], ai)
            
            # Mẫu số Jacobian: bi . wi
            denom = np.dot(bi, wi)
            
            if abs(denom) < 1e-9:
                return [0, 0, 0] # Singularity
                
            # Row i của Inverse Jacobian (đơn vị 1/m)
            J_inv[i, :] = bi / denom

        # --- TÍNH LỰC (FORCE) - NEWTON ---
        # Tổng khối lượng tác động lên End-Effector
        m_total_load = self.mass_payload + self.mass_platform + 3 * (2 * self.mass_lower_arm * 0.5)
        
        # Lực trọng trường (Gravity)
        F_gravity = np.array([0, 0, -m_total_load * self.g])
        
        # Lực quán tính (F = m*a)
        # Robot cần sinh lực F_robot để thắng trọng lực và tạo gia tốc
        F_robot_needed = -F_gravity + (m_total_load * np.array(accel_vec))
        
        try:
            # Giải hệ phương trình: J_inv.T * tau = F_robot_needed
            # J_inv (1/m), tau (Nm) -> J_inv * tau = Force (N)
            tau_load = np.linalg.solve(J_inv.T, F_robot_needed)
        except np.linalg.LinAlgError:
            return [0, 0, 0]

        # --- MOMENT TĨNH CỦA CÁNH TAY TRÊN (UPPER ARM) ---
        # Torque = m * g * (rf/2) * cos(theta)
        tau_arm = []
        for i in range(3):
            # Tính bằng mét -> Torque ra Nm
            t_arm_g = self.mass_upper_arm * self.g * (self.rf_m / 2.0) * math.cos(theta[i])
            tau_arm.append(t_arm_g)
            
        # Tổng Torque (Nm)
        total_torque = [tau_load[i] + tau_arm[i] for i in range(3)]
        
        return total_torque


class SimulatedPlanner(MotionPlannerTrapezoidal):
    """Planner dành riêng cho mô phỏng: Nới lỏng kiểm tra workspace."""
    def _is_point_reachable(self, point, alpha_deg=None):
        # Gọi logic gốc
        reachable = super()._is_point_reachable(point, alpha_deg)
        if not reachable:
            x, y, z = point
            # Nếu điểm nằm ở vùng cao (khu vực Home, Z > -250), cho phép đi qua
            # để tránh lỗi "Start point outside workspace" khi robot đang ở Home.
            if z > -250.0:
                return True
        return reachable

class DeltaRobotVisualizer(gl.GLViewWidget):
    """Widget hiển thị 3D của Delta Robot"""
    def __init__(self, kinematics):
        super().__init__()
        self.kinematics = kinematics
        self.setCameraPosition(distance=1000, elevation=30, azimuth=-90)
        self.opts['center'] = pg.Vector(0, 0, -300) # Tâm nhìn
        
        # Grid mặt đất
        g = gl.GLGridItem()
        g.setSize(x=600, y=600)
        g.setSpacing(x=50, y=50)
        self.addItem(g)
        
        # Hệ trục tọa độ
        axis = gl.GLAxisItem()
        axis.setSize(100, 100, 100)
        self.addItem(axis)

        # --- KHỞI TẠO CÁC PHẦN CỦA ROBOT ---
        # 1. Base (Tam giác đều trên đỉnh) - Cố định
        self.f = self.kinematics.f
        R_base = self.f / (2 * math.sqrt(3))
        self.base_pts = self._create_triangle_points(0, 0, 0, R_base)
        
        # Faces định nghĩa mặt tam giác nối 3 đỉnh 0-1-2
        self.faces = np.array([[0, 1, 2]], dtype=np.int32)
        
        self.base_mesh = gl.GLMeshItem(vertexes=self.base_pts, faces=self.faces, drawEdges=True, color=(0.3, 0.3, 0.3, 1))
        self.addItem(self.base_mesh)

        # 2. Platform (Tam giác đều di động)
        self.e = self.kinematics.e
        self.R_plat = self.e / (2 * math.sqrt(3))
        self.plat_pts = self._create_triangle_points(0, 0, 0, self.R_plat)
        self.plat_mesh = gl.GLMeshItem(vertexes=self.plat_pts, faces=self.faces, drawEdges=True, color=(0, 1, 0, 1))
        self.addItem(self.plat_mesh)
        
        # 3. Arms (Cánh tay trên - RF) và Forearms (Cánh tay dưới - RE)
        self.arm_lines = []
        for i in range(3):
            line = gl.GLLinePlotItem(width=3, color=(1, 0, 0, 1)) # Đỏ
            self.addItem(line)
            self.arm_lines.append(line)
            
        self.forearm_lines = []
        for i in range(3):
            line = gl.GLLinePlotItem(width=2, color=(0, 1, 1, 1)) # Xanh Cyan
            self.addItem(line)
            self.forearm_lines.append(line)

        # 4. End Effector (Suction Cup) - ĐẦU HÚT
        # Tạo hình trụ: radius=[trên, dưới], length=40mm
        md_cup = gl.MeshData.cylinder(rows=10, cols=20, radius=[6, 12], length=40)
        self.suction_cup = gl.GLMeshItem(meshdata=md_cup, smooth=True, color=(1, 1, 0, 1), shader='balloon')
        self.addItem(self.suction_cup)
        
        # [NEW] Chỉ báo hướng xoay (Servo Indicator) - Một thanh ngang màu đỏ
        self.servo_indicator = gl.GLBoxItem(size=pg.Vector(30, 4, 4), color=(1, 0, 0, 1))
        # Dời tâm box về giữa để xoay cho đẹp (mặc định box vẽ từ góc 0,0,0)
        self.servo_indicator.translate(-15, -2, 0) 
        self.addItem(self.servo_indicator)
        
        self.servo_angle = 0 # [NEW] Lưu góc xoay hiện tại
            
        self.update_robot_pose(0, 0, 0) # Vị trí ban đầu

    def _create_triangle_points(self, x, y, z, r):
        pts = []
        angles = [-90, 30, 150] 
        for ang in angles:
            rad = math.radians(ang)
            pts.append([x + r * math.cos(rad), y + r * math.sin(rad), z])
        return np.array(pts)

    def update_robot_pose(self, theta1, theta2, theta3):
        R_base = self.f / (2 * math.sqrt(3))
        B = []
        angles_base = [-90, 30, 150]
        for ang in angles_base:
            rad = math.radians(ang)
            B.append([R_base * math.cos(rad), R_base * math.sin(rad), 0])
            
        rf = self.kinematics.rf
        J = []
        thetas = [theta1, theta2, theta3]
        
        for i, ang_base in enumerate(angles_base):
            rad_base = math.radians(ang_base)
            theta_rad = math.radians(thetas[i])
            
            horiz = rf * math.cos(theta_rad)
            vert = -rf * math.sin(theta_rad)
            
            jx = B[i][0] + horiz * math.cos(rad_base)
            jy = B[i][1] + horiz * math.sin(rad_base)
            jz = vert
            J.append([jx, jy, jz])
            self.arm_lines[i].setData(pos=np.array([B[i], [jx, jy, jz]]))

        coords = self.kinematics.forward_kinematics(theta1, theta2, theta3)
        if coords:
            x, y, z = coords
            plat_pts_new = self._create_triangle_points(x, y, z, self.R_plat)
            self.plat_mesh.setMeshData(vertexes=plat_pts_new, faces=self.faces)
            for i in range(3):
                self.forearm_lines[i].setData(pos=np.array([J[i], plat_pts_new[i]]))

            # Cập nhật vị trí đầu hút
            cup_len = 40
            self.suction_cup.resetTransform()
            
            # 1. Xoay quanh trục Z (tại gốc của chính nó) -> Xoay tại chỗ
            self.suction_cup.rotate(self.servo_angle, 0, 0, 1) 
            
            # 2. Dời về vị trí lệch tâm (so với tâm bàn kẹp)
            self.suction_cup.translate(0, -15.0, 0)
            
            # 3. Dời đến vị trí bàn kẹp trong không gian
            self.suction_cup.translate(x, y, z - cup_len)

    def set_suction_color(self, color):
        """Đổi màu đầu hút: color = (r, g, b, a)"""
        self.suction_cup.setColor(color)


class DeltaSimulationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Gemini Delta Robot Simulation (Real-time)")
        self.resize(1300, 900)

        # --- LOGIC ---
        self.kinematics = DeltaKinematics()
        self.planner = SimulatedPlanner(self.kinematics)
        self.dynamics = DeltaDynamics(self.kinematics)
        
        # State
        val = C.HOME_DEFAULT_THETA_DEG
        self.current_angles = [val, val, val]
        
        # [FIX] Tính toán tọa độ khởi đầu chính xác từ góc Home để đồng bộ
        coords = self.kinematics.forward_kinematics_tool(*self.current_angles, alpha_deg=-90.0)
        if coords:
            self.current_coords = list(coords)
        else:
            self.current_coords = [0.0, 0.0, -300.0]
            
        self.prev_coords = list(self.current_coords)
        self.prev_velocity = [0.0, 0.0, 0.0] 
        self.filtered_torque = [0.0, 0.0, 0.0] 
        
        # Simulation Loop
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation_step)
        self.sim_interval_ms = 15 # ~66 FPS
        self.trajectory_queue = [] 
        self.sim_start_time = 0

        # Data logging
        self.log_time = []
        self.log_theta1 = []
        self.log_theta2 = []
        self.log_theta3 = []
        self.log_velocity = [] # Lưu vận tốc
        self.log_torque1 = []
        self.log_torque2 = []
        self.log_torque3 = []
        self.start_time_ref = time.time()

        # --- GUI LAYOUT ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)

        # LEFT PANEL: Controls & 3D View
        left_layout = QVBoxLayout()
        self.viz = DeltaRobotVisualizer(self.kinematics)
        left_layout.addWidget(self.viz, stretch=2)
        
        # [UPDATE] Gộp thành 1 panel duy nhất
        self.panel_control = QWidget()
        self.init_compact_control_panel()
        left_layout.addWidget(self.panel_control)
        
        layout.addLayout(left_layout, stretch=2)

        # RIGHT PANEL: Graphs
        self.win_plot = pg.GraphicsLayoutWidget(show=True, title="Real-time Monitor")
        layout.addWidget(self.win_plot, stretch=1)
        
        # --- Cấu hình 5 biểu đồ ---
        # Plot 1: Theta 1
        self.p1 = self.win_plot.addPlot(title="Góc Khớp 1 (Theta 1)")
        self.p1.showGrid(x=True, y=True)
        self.p1.setLabel('left', 'Độ')
        self.curve1 = self.p1.plot(pen='r')
        self.win_plot.nextRow()

        # Plot 2: Theta 2
        self.p2 = self.win_plot.addPlot(title="Góc Khớp 2 (Theta 2)")
        self.p2.showGrid(x=True, y=True)
        self.p2.setLabel('left', 'Độ')
        self.p2.setXLink(self.p1)
        self.p2.setYLink(self.p1)
        self.curve2 = self.p2.plot(pen='g')
        self.win_plot.nextRow()

        # Plot 3: Theta 3
        self.p3 = self.win_plot.addPlot(title="Góc Khớp 3 (Theta 3)")
        self.p3.showGrid(x=True, y=True)
        self.p3.setLabel('left', 'Độ')
        self.p3.setXLink(self.p1)
        self.p3.setYLink(self.p1)
        self.curve3 = self.p3.plot(pen='b')
        self.win_plot.nextRow()

        # Plot 4: Velocity
        self.p4 = self.win_plot.addPlot(title="Vận tốc bệ (mm/s)")
        self.p4.showGrid(x=True, y=True)
        self.p4.setLabel('left', 'mm/s')
        self.p4.setXLink(self.p1)
        self.curve_vel = self.p4.plot(pen=pg.mkPen('y', width=2))
        self.win_plot.nextRow()

        # Plot 5: Moment (Torque)
        self.p5 = self.win_plot.addPlot(title="Moment Động cơ (N.m)")
        self.p5.showGrid(x=True, y=True)
        self.p5.setLabel('left', 'N.m')
        self.p5.setXLink(self.p1)
        self.p5.addLegend()
        self.curve_tau1 = self.p5.plot(pen='r', name="Tau 1")
        self.curve_tau2 = self.p5.plot(pen='g', name="Tau 2")
        self.curve_tau3 = self.p5.plot(pen='b', name="Tau 3")
        
        # Khởi động
        self.cmd_home()
        self.timer.start(self.sim_interval_ms)
        
        # --- JOG TIMER ---
        self.jog_timer = QTimer()
        self.jog_timer.setInterval(30) # 33Hz updates
        self.jog_timer.timeout.connect(self.jog_timer_callback)
        self.current_jog_axis = None
        self.current_jog_sign = 0

    def init_compact_control_panel(self):
        """Khởi tạo giao diện điều khiển TÍCH HỢP (Compact)"""
        main_layout = QHBoxLayout()
        
        # --- CỘT 1: MANUAL CONTROL (JOG & TOOLS) ---
        col1 = QVBoxLayout()
        
        # 1. JOG CONTROL
        jog_group = QGroupBox("Điều khiển JOG (Ấn giữ)")
        grid_jog = QGridLayout()
        
        # Helper tạo nút JOG
        def create_jog_btn(text, axis, sign, color=None):
            btn = QPushButton(text)
            btn.setFixedSize(60, 40) # Compact size
            if color: btn.setStyleSheet(f"background-color: {color}")
            
            # [IMPORTANT] Xử lý sự kiện nhấn/nhả thủ công để tránh lag/trôi
            btn.pressed.connect(lambda: self.start_jog(axis, sign))
            btn.released.connect(self.stop_jog)
            return btn
            
        # --- BỐ TRÍ LAYOUT JOG CHUẨN (4 Cột x 3 Hàng) ---
        # Hàng 0: HOME | Y+ | RAND | Z+
        btn_home = QPushButton("HOME")
        btn_home.setFixedSize(60, 40)
        btn_home.setStyleSheet("background-color: #FFA500; font-weight: bold")
        btn_home.clicked.connect(self.cmd_home)
        grid_jog.addWidget(btn_home, 0, 0)
        
        grid_jog.addWidget(create_jog_btn("Y+", 'y', 1), 0, 1)
        
        btn_rand = QPushButton("Rand")
        btn_rand.setFixedSize(60, 40)
        btn_rand.clicked.connect(self.cmd_move_random)
        grid_jog.addWidget(btn_rand, 0, 2)
        
        grid_jog.addWidget(create_jog_btn("Z+", 'z', 1, "#ADD8E6"), 0, 3)
        
        # Hàng 1: X- | (Trống) | X+ | (Trống)
        grid_jog.addWidget(create_jog_btn("X-", 'x', -1), 1, 0)
        grid_jog.addWidget(create_jog_btn("X+", 'x', 1), 1, 2)
        
        # Hàng 2: (Trống) | Y- | (Trống) | Z-
        grid_jog.addWidget(create_jog_btn("Y-", 'y', -1), 2, 1)
        grid_jog.addWidget(create_jog_btn("Z-", 'z', -1, "#ADD8E6"), 2, 3)
        
        jog_group.setLayout(grid_jog)
        col1.addWidget(jog_group)
        
        # 2. STEP SIZE (SPEED)
        step_group = QGroupBox("Tốc độ JOG")
        step_layout = QHBoxLayout()
        self.jog_speed_scale = 1.0 # Hệ số tốc độ
        
        step_bg = QButtonGroup(self)
        # Đổi ý nghĩa: Đây là tốc độ di chuyển mỗi tick (mm/tick)
        steps = [1, 2, 5, 10] 
        for val in steps:
            rb = QRadioButton(f"x{val}")
            if val == 2: rb.setChecked(True); self.jog_speed_scale = 2.0
            step_bg.addButton(rb, val)
            step_layout.addWidget(rb)
        step_bg.buttonClicked.connect(lambda btn: setattr(self, 'jog_speed_scale', float(step_bg.id(btn))))
        
        step_group.setLayout(step_layout)
        col1.addWidget(step_group)
        
        # 3. TOOLS (Pump & Servo)
        tool_group = QGroupBox("Đầu công tác")
        tool_layout = QGridLayout()
        
        # Pump
        self.btn_pump = QPushButton("BƠM: TẮT")
        self.btn_pump.setCheckable(True)
        self.btn_pump.setStyleSheet("background-color: #FFCCCC")
        self.btn_pump.clicked.connect(self.toggle_pump)
        tool_layout.addWidget(self.btn_pump, 0, 0, 1, 2)
        
        # Servo
        tool_layout.addWidget(QLabel("Xoay:"), 1, 0)
        self.slider_servo = QSlider(Qt.Orientation.Horizontal)
        self.slider_servo.setRange(-225, 45) # [UPDATE] Range thực tế hệ thống (-225 đến 45)
        self.slider_servo.setValue(-90) # Default Home Angle
        self.slider_servo.valueChanged.connect(self.update_servo_visual)
        tool_layout.addWidget(self.slider_servo, 1, 1)
        self.lbl_servo = QLabel("0°")
        tool_layout.addWidget(self.lbl_servo, 1, 2)
        
        tool_group.setLayout(tool_layout)
        col1.addWidget(tool_group)
        
        main_layout.addLayout(col1, stretch=1)
        
        # --- CỘT 2: CONFIG & INFO ---
        col2 = QVBoxLayout()
        
        # 1. Move To Target
        move_group = QGroupBox("Di chuyển tọa độ")
        grid_move = QGridLayout()
        
        grid_move.addWidget(QLabel("X:"), 0, 0); self.spin_x = QDoubleSpinBox(); self.spin_x.setRange(-200, 200); grid_move.addWidget(self.spin_x, 0, 1)
        grid_move.addWidget(QLabel("Y:"), 0, 2); self.spin_y = QDoubleSpinBox(); self.spin_y.setRange(-200, 200); grid_move.addWidget(self.spin_y, 0, 3)
        grid_move.addWidget(QLabel("Z:"), 1, 0); self.spin_z = QDoubleSpinBox(); self.spin_z.setRange(-500, -200); self.spin_z.setValue(-350); grid_move.addWidget(self.spin_z, 1, 1)
        
        btn_go = QPushButton("GO"); btn_go.clicked.connect(self.cmd_move_target)
        grid_move.addWidget(btn_go, 1, 2, 1, 2)
        
        move_group.setLayout(grid_move)
        col2.addWidget(move_group)
        
        # 2. Physics
        phys_group = QGroupBox("Vật lý & Tải trọng")
        phys_layout = QGridLayout()
        phys_layout.addWidget(QLabel("Payload (kg):"), 0, 0)
        self.spin_payload = QDoubleSpinBox(); self.spin_payload.setRange(0, 10); self.spin_payload.setValue(0.5); self.spin_payload.setSingleStep(0.1)
        self.spin_payload.valueChanged.connect(self.update_dynamics_params)
        phys_layout.addWidget(self.spin_payload, 0, 1)
        
        phys_layout.addWidget(QLabel("Arm Mass (kg):"), 1, 0)
        self.spin_arm_mass = QDoubleSpinBox(); self.spin_arm_mass.setRange(0, 5); self.spin_arm_mass.setValue(0.3); self.spin_arm_mass.setSingleStep(0.1)
        self.spin_arm_mass.valueChanged.connect(self.update_dynamics_params)
        phys_layout.addWidget(self.spin_arm_mass, 1, 1)
        phys_group.setLayout(phys_layout)
        col2.addWidget(phys_group)
        
        # 3. Status Info
        self.lbl_pos = QLabel("POS: X=0 Y=0 Z=0")
        self.lbl_pos.setStyleSheet("font-size: 10pt; font-weight: bold; color: blue")
        col2.addWidget(self.lbl_pos)
        
        col2.addStretch()
        main_layout.addLayout(col2, stretch=1)
        
        self.panel_control.setLayout(main_layout)

    # --- JOG LOGIC MỚI ---
    def start_jog(self, axis, sign):
        """Bắt đầu JOG"""
        self.current_jog_axis = axis
        self.current_jog_sign = sign
        # Xóa queue cũ để bắt đầu JOG ngay lập tức
        self.trajectory_queue.clear()
        self.jog_timer.start()

    def stop_jog(self):
        """Dừng JOG"""
        self.jog_timer.stop()
        self.current_jog_axis = None
        self.current_jog_sign = 0
        # Xóa queue để dừng ngay lập tức (chống trôi)
        self.trajectory_queue.clear()
        
    def jog_timer_callback(self):
        """Hàm được gọi liên tục khi giữ nút JOG"""
        if not self.current_jog_axis: return
        
        # Tính toán vị trí mục tiêu TIẾP THEO (ngắn thôi)
        # Di chuyển mỗi tick (30ms) một đoạn nhỏ
        step = self.jog_speed_scale # mm/tick
        
        cx, cy, cz = self.current_coords
        tx, ty, tz = cx, cy, cz
        
        if self.current_jog_axis == 'x': tx += step * self.current_jog_sign
        elif self.current_jog_axis == 'y': ty += step * self.current_jog_sign
        elif self.current_jog_axis == 'z': tz += step * self.current_jog_sign
        
        # Kiểm tra giới hạn Workspace đơn giản
        if tz > -200: tz = -200
        if tz < -500: tz = -500
        
        # Thay vì gọi _plan_move (Trapezoidal), ta tính trực tiếp IK và nhồi vào queue
        # Điều này tạo ra chuyển động tuyến tính đều (Constant Velocity) -> KHÔNG RUNG
        
        alpha = -90.0 # Góc mặc định
        
        angles = self.kinematics.inverse_kinematics_tool(tx, ty, tz, alpha)
        
        if angles:
            # Thêm trực tiếp vào queue với thời gian dài hơn Timer (30ms)
            # t=50ms > interval=30ms => Tạo buffer gối đầu -> Chuyển động mượt, không rung
            self.trajectory_queue.append({
                'theta': list(angles),
                't': 0.050 
            })
            
            # Cập nhật current_coords ảo để lần tick sau tính tiếp từ đây
            self.current_coords = [tx, ty, tz] # Hack để tick sau cộng dồn tiếp
        else:
            print("JOG Limit Reached")
            self.stop_jog()

    def toggle_pump(self):
        """Mô phỏng bật/tắt bơm (đổi màu đầu hút)"""
        is_on = self.btn_pump.isChecked()
        if is_on:
            self.btn_pump.setText("BƠM: BẬT")
            self.btn_pump.setStyleSheet("background-color: #90EE90") # Light Green
            self.viz.set_suction_color((0, 1, 0, 1)) # Xanh lá
        else:
            self.btn_pump.setText("BƠM: TẮT")
            self.btn_pump.setStyleSheet("background-color: #FFCCCC") # Light Red
            self.viz.set_suction_color((1, 1, 0, 1)) # Vàng

    def update_servo_visual(self):
        """Mô phỏng xoay servo (xoay đầu hút)"""
        angle = self.slider_servo.value()
        self.lbl_servo.setText(f"{angle}°")
        
        # Cập nhật góc cho Visualizer
        self.viz.servo_angle = angle
        
        # Vẽ lại robot (để áp dụng góc xoay mới ngay lập tức)
        self.viz.update_robot_pose(*self.current_angles)

    def update_dynamics_params(self):
        self.dynamics.mass_payload = self.spin_payload.value()
        self.dynamics.mass_upper_arm = self.spin_arm_mass.value()

    def cmd_home(self):
        """Giả lập lệnh Home"""
        self.trajectory_queue.clear()
        # Home về góc mặc định (thường là -40 độ)
        val = C.HOME_DEFAULT_THETA_DEG
        home_theta = [val, val, val]
        
        steps = 50
        start_theta = self.current_angles
        for i in range(steps):
            t = (i+1)/steps
            th1 = start_theta[0] + (home_theta[0] - start_theta[0]) * t
            th2 = start_theta[1] + (home_theta[1] - start_theta[1]) * t
            th3 = start_theta[2] + (home_theta[2] - start_theta[2]) * t
            self.trajectory_queue.append({'theta': [th1, th2, th3]})
            
    def cmd_move_random(self):
        import random
        x = random.uniform(-100, 100)
        y = random.uniform(-100, 100)
        z = random.uniform(-450, -300)
        self._plan_move(x, y, z)

    def cmd_move_target(self):
        x = self.spin_x.value()
        y = self.spin_y.value()
        z = self.spin_z.value()
        self._plan_move(x, y, z)

    def _plan_move(self, x, y, z):
        print(f"Planning move to: {x:.1f}, {y:.1f}, {z:.1f}")
        current_x, current_y, current_z = self.current_coords
        
        # --- DEBUG PROFILE ---
        dist = math.sqrt((x-current_x)**2 + (y-current_y)**2 + (z-current_z)**2)
        v_max = self.planner.v_linear_max
        accel = self.planner.accel_linear
        s_required = (v_max**2) / accel
        
        print(f"--- DEBUG MOVE ---")
        print(f"Distance: {dist:.2f} mm")
        print(f"Min Dist for Trapezoid: {s_required:.2f} mm (V_max={v_max}, Accel={accel})")
        if dist < s_required:
            print("=> PROFILE: TRIANGLE (Quãng đường quá ngắn)")
        else:
            print("=> PROFILE: TRAPEZOID (Đủ quãng đường)")
        # ---------------------

        try:
            plan = self.planner.plan_cartesian_move_time_sliced(
                start_cartesian=(current_x, current_y, current_z),
                end_cartesian=(x, y, z),
                alpha_deg=-90.0, 
                kinematics=self.kinematics,
                current_angles=tuple(self.current_angles)
            )
            
            if plan:
                self.trajectory_queue.clear() # Xóa queue cũ
                sim_angles = list(self.current_angles)
                for block in plan:
                    d_steps = block['s']
                    d_theta = self.kinematics.steps_to_angles(*d_steps)
                    sim_angles[0] += d_theta[0]
                    sim_angles[1] += d_theta[1]
                    sim_angles[2] += d_theta[2]
                    self.trajectory_queue.append({
                        'theta': list(sim_angles),
                        't': block['t'] # Lưu thời gian thực của block
                    })
        except ValueError as e:
             print(f"Lỗi lập kế hoạch: {e}")

    def update_simulation_step(self):
        if self.trajectory_queue:
            point = self.trajectory_queue.pop(0)
            self.current_angles = point['theta']
            
            # Dùng forward_kinematics_tool để lấy tọa độ ĐẦU HÚT
            coords = self.kinematics.forward_kinematics_tool(*self.current_angles, alpha_deg=-90.0)
            if coords:
                self.current_coords = list(coords)
                
                # --- TÍNH TOÁN VẬN TỐC (mm/s) ---
                # V = Distance / Time_Step
                dt_block = point.get('t', self.sim_interval_ms / 1000.0)
                if dt_block <= 0: dt_block = 0.001

                dx = self.current_coords[0] - self.prev_coords[0]
                dy = self.current_coords[1] - self.prev_coords[1]
                dz = self.current_coords[2] - self.prev_coords[2]
                
                dist = math.sqrt(dx**2 + dy**2 + dz**2)
                
                velocity = dist / dt_block # Scalar velocity mm/s
                
                # Vector Velocity m/s
                vel_vec = [dx/dt_block/1000.0, dy/dt_block/1000.0, dz/dt_block/1000.0]
                
                # Vector Acceleration m/s^2
                dvx = vel_vec[0] - self.prev_velocity[0]
                dvy = vel_vec[1] - self.prev_velocity[1]
                dvz = vel_vec[2] - self.prev_velocity[2]
                accel_vec = [dvx/dt_block, dvy/dt_block, dvz/dt_block]
                
                # --- TÍNH TOÁN MOMENT (TORQUE) ---
                raw_torques = self.dynamics.compute_torque(self.current_angles, vel_vec, accel_vec)
                
                alpha = 0.1
                torques = []
                for i in range(3):
                    val = (alpha * raw_torques[i]) + ((1 - alpha) * self.filtered_torque[i])
                    torques.append(val)
                self.filtered_torque = torques
                    
                self.prev_coords = list(self.current_coords)
                self.prev_velocity = vel_vec
            else:
                velocity = 0.0
                torques = [0.0, 0.0, 0.0] # Reset về 0 khi dừng
                self.filtered_torque = [0.0, 0.0, 0.0]

            # Cập nhật GUI
            self.viz.update_robot_pose(*self.current_angles)
            self.lbl_pos.setText(f"Pos: X={self.current_coords[0]:.1f} Y={self.current_coords[1]:.1f} Z={self.current_coords[2]:.1f} | V={velocity:.1f} mm/s | TauMax={max([abs(t) for t in torques]):.2f} Nm")
            
            # Logging
            now = time.time() - self.start_time_ref
            self.log_time.append(now)
            self.log_theta1.append(self.current_angles[0])
            self.log_theta2.append(self.current_angles[1])
            self.log_theta3.append(self.current_angles[2])
            self.log_velocity.append(velocity)
            self.log_torque1.append(torques[0])
            self.log_torque2.append(torques[1])
            self.log_torque3.append(torques[2])
            
            # Limit data
            limit = 500
            if len(self.log_time) > limit:
                self.log_time = self.log_time[-limit:]
                self.log_theta1 = self.log_theta1[-limit:]
                self.log_theta2 = self.log_theta2[-limit:]
                self.log_theta3 = self.log_theta3[-limit:]
                self.log_velocity = self.log_velocity[-limit:]
                self.log_torque1 = self.log_torque1[-limit:]
                self.log_torque2 = self.log_torque2[-limit:]
                self.log_torque3 = self.log_torque3[-limit:]
                
            # Update Plots
            self.curve1.setData(self.log_time, self.log_theta1)
            self.curve2.setData(self.log_time, self.log_theta2)
            self.curve3.setData(self.log_time, self.log_theta3)
            self.curve_vel.setData(self.log_time, self.log_velocity)
            
            self.curve_tau1.setData(self.log_time, self.log_torque1)
            self.curve_tau2.setData(self.log_time, self.log_torque2)
            self.curve_tau3.setData(self.log_time, self.log_torque3)
            
            # Cập nhật thông số hiển thị
            self.spin_x.setValue(self.current_coords[0])
            self.spin_y.setValue(self.current_coords[1])
            self.spin_z.setValue(self.current_coords[2])

    # =========================================================================
    # DIGITAL TWIN INTEGRATION
    # =========================================================================
    def start_digital_twin_mode(self):
        """Chuyển sang chế độ Digital Twin (Nhận dữ liệu từ Main App)"""
        self.setWindowTitle("Gemini Delta Robot - 3D Digital Twin")
        
        # 1. Ẩn bảng điều khiển (Vì điều khiển bằng Main App)
        self.panel_control.hide()
        
        # 2. Dừng Physics Loop nội bộ
        self.timer.stop()
        self.jog_timer.stop()
        
        # 3. Reset đồ thị
        self.log_time = []
        self.start_time_ref = time.time()
        
    def update_state_from_external(self, theta, coords, servo_angle=None, pump_state=None):
        """
        Hàm nhận dữ liệu từ Robot Controller thật
        :param theta: List [theta1, theta2, theta3] (Độ)
        :param coords: List [x, y, z] (mm)
        :param servo_angle: Góc servo đầu hút (Độ)
        :param pump_state: Trạng thái bơm (Boolean hoặc 0/1)
        """
        # Cập nhật Visualizer 3D
        self.current_angles = theta
        self.current_coords = coords
        
        if servo_angle is not None:
            self.viz.servo_angle = servo_angle
            
        if pump_state is not None:
            if pump_state:
                self.viz.set_suction_color((0, 1, 0, 1)) # Green
            else:
                self.viz.set_suction_color((1, 1, 0, 1)) # Yellow
                
        self.viz.update_robot_pose(*theta)
        
        # Cập nhật Biểu đồ (Logging)
        now = time.time() - self.start_time_ref
        
        self.log_time.append(now)
        self.log_theta1.append(theta[0])
        self.log_theta2.append(theta[1])
        self.log_theta3.append(theta[2])
        
        # Limit data
        limit = 500
        if len(self.log_time) > limit:
            self.log_time = self.log_time[-limit:]
            self.log_theta1 = self.log_theta1[-limit:]
            self.log_theta2 = self.log_theta2[-limit:]
            self.log_theta3 = self.log_theta3[-limit:]
            
        # Vẽ lại đồ thị
        self.curve1.setData(self.log_time, self.log_theta1)
        self.curve2.setData(self.log_time, self.log_theta2)
        self.curve3.setData(self.log_time, self.log_theta3)
        # (Tạm bỏ qua Velocity/Torque vì không tính toán real-time physics ở mode này)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DeltaSimulationApp()
    window.show()
    sys.exit(app.exec())