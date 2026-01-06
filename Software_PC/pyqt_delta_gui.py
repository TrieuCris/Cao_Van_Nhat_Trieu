import sys
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QFrame, QGridLayout, QLineEdit, 
                             QTextEdit, QComboBox, QGroupBox, QSlider, QCheckBox, QAbstractButton, QMessageBox)
from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer, QSize, QPropertyAnimation, QEasingCurve, pyqtProperty
from PyQt6.QtGui import QImage, QPixmap, QFont, QColor, QIcon, QPainter, QBrush, QPen, QTextCursor
import cv2
import numpy as np
from datetime import datetime
from connection_manager import ConnectionManager, list_ports
import constants as C

# --- STYLESHEET (CSS cho Desktop App) ---
DARK_THEME = """
QMainWindow { background-color: #1e1e1e; }
QWidget { color: #e0e0e0; font-family: 'Segoe UI', Arial; font-size: 14px; }
QGroupBox { 
    border: 1px solid #3e3e3e; 
    border-radius: 5px; 
    margin-top: 10px; 
    font-weight: bold; 
    color: #4CAF50;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }
QPushButton {
    background-color: #333333;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 5px;
    min-height: 25px;
}
QPushButton:hover { background-color: #444; border-color: #4CAF50; }
QPushButton:pressed { background-color: #222; }
QPushButton:disabled { background-color: #2a2a2a; color: #555; border-color: #333; }

/* Các nút đặc biệt */
QPushButton#btn_run { background-color: #2E7D32; color: white; font-weight: bold; font-size: 16px; }
QPushButton#btn_run:hover { background-color: #388E3C; }
QPushButton#btn_stop { background-color: #C62828; color: white; font-weight: bold; font-size: 16px; }
QPushButton#btn_stop:hover { background-color: #D32F2F; }
QPushButton#btn_jog { background-color: #0277BD; color: white; font-weight: bold; }

QLineEdit {
    background-color: #252526;
    border: 1px solid #3e3e3e;
    border-radius: 3px;
    padding: 3px;
    color: #fff;
}
QLineEdit:focus { border: 1px solid #4CAF50; }

QLabel#camera_view {
    background-color: #000;
    border: 2px solid #333;
}

QTextEdit {
    background-color: #111;
    color: #00FF00;
    border: 1px solid #333;
    font-family: 'Consolas', monospace;
    font-size: 12px;
}
"""

class ToggleSwitch(QAbstractButton):
    """Custom Toggle Switch with Animation"""
    stateChanged = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setFixedSize(50, 26)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        
        # Colors
        self._track_off = QColor("#555")
        self._track_on = QColor("#4CAF50")
        self._thumb_color = QColor("#FFFFFF")
        
        # Animation property
        self._position = 0.0 # 0.0 = Off, 1.0 = On
        self._anim = QPropertyAnimation(self, b"position", self)
        self._anim.setDuration(250)
        self._anim.setEasingCurve(QEasingCurve.Type.InOutQuad)
        
        self.toggled.connect(self._on_toggle)

    @pyqtProperty(float)
    def position(self):
        return self._position

    @position.setter
    def position(self, pos):
        self._position = pos
        self.update()

    def _on_toggle(self, checked):
        self.stateChanged.emit(checked)
        # Re-check state because a slot might have reverted it (e.g. validation failed)
        actual_checked = self.isChecked()
        
        self._anim.stop()
        self._anim.setStartValue(self._position)
        self._anim.setEndValue(1.0 if actual_checked else 0.0)
        self._anim.start()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Track Rect
        rect = self.rect()
        h = rect.height()
        w = rect.width()
        
        # Determine Track Color based on animation position
        # Use a smooth transition or simple threshold
        track_color = self._track_on if self._position > 0.5 else self._track_off
        
        p.setBrush(QBrush(track_color))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(0, 0, w, h, h/2, h/2)
        
        # Draw Thumb
        margin = 3
        thumb_d = h - 2*margin
        
        # Calculate X position
        x_off = margin
        x_on = w - margin - thumb_d
        current_x = x_off + (x_on - x_off) * self._position
        
        p.setBrush(QBrush(self._thumb_color))
        p.drawEllipse(int(current_x), margin, int(thumb_d), int(thumb_d))
        p.end()

    # Compatibility Methods
    def set_state(self, state):
        # Stop animation if running to prevent override
        self._anim.stop()
        
        if self.isChecked() != state:
            self.blockSignals(True)
            self.setChecked(state)
            self.blockSignals(False)
        
        # Instant update for programmatic set
        self._position = 1.0 if state else 0.0
        self.update()
            
    @property
    def state(self):
        return self.isChecked()
    
    @state.setter
    def state(self, value):
        self.set_state(value)

    def draw(self):
        pass

class DeltaRobotGUI(QMainWindow):
    # Các tín hiệu để giao tiếp Thread-safe
    log_signal = pyqtSignal(str, str)
    image_signal = pyqtSignal(np.ndarray)
    indicator_signal = pyqtSignal(str, bool)
    coord_signal = pyqtSignal(str, str)  # (axis, value)
    theta_signal = pyqtSignal(float, float, float)  # (t1, t2, t3)
    run_on_main_signal = pyqtSignal(object, object) # func, args
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hệ Thống Robot Delta - PyQt6 Pro")
        self.setGeometry(100, 100, 1600, 900) # Kích thước mặc định
        self.showMaximized() # Hiển thị full màn hình (có thanh tiêu đề)
        
        # --- Backend Adapters ---
        self.conn_manager = ConnectionManager()
        self.video_thread = None
        self.robot_controller = None
        self.button_callbacks = {}
        
        # Connect signal to handle run on main thread
        self.run_on_main_signal.connect(self._handle_run_on_main)

        # State variables (để thay thế tk.StringVar)
        self._coord_vals = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self._theta_vals = {'T1': 0.0, 'T2': 0.0, 'T3': 0.0}
        self._product_count = 0
        
        # Indicators mapping
        self.indicators = {} 
        self.product_slots = {}

        self.init_ui()
        self.apply_styles()
        
        # Kết nối tín hiệu nội bộ
        self.log_signal.connect(self._handle_log_signal)
        self.image_signal.connect(self._handle_image_signal)
        self.indicator_signal.connect(self._handle_indicator_signal)
        self.coord_signal.connect(self._handle_coord_signal)
        self.theta_signal.connect(self._handle_theta_signal)
        
        # Timer thay cho root.after của Tkinter
        self._timers = []
        
        # E-STOP Blinking Timer
        self.estop_blink_timer = QTimer(self)
        self.estop_blink_timer.setInterval(1000) # Chu kỳ 2s (1s ON, 1s OFF)
        self.estop_blink_timer.timeout.connect(self._toggle_estop_led)
        self.estop_led_state = False

        # --- LOG FILE INITIALIZATION ---
        self.log_filename = "session_log.txt"
        try:
            with open(self.log_filename, "w", encoding="utf-8") as f:
                f.write(f"=== SESSION STARTED: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        except Exception as e:
            print(f"Failed to initialize log file: {e}")

        # --- COMPATIBILITY ALIASES ---
        self.pump_switch = self.pump_toggle  # Alias cho code cũ gọi app.pump_switch

    def _toggle_estop_led(self):
        """Đảo trạng thái đèn E-STOP để tạo hiệu ứng nhấp nháy."""
        self.estop_led_state = not self.estop_led_state
        # Red vs Grey
        color = "#F44336" if self.estop_led_state else "#444" 
        
        if C.INDICATOR_E_STOP in self.indicator_labels:
             self.indicator_labels[C.INDICATOR_E_STOP].setStyleSheet(f"background-color: {color}; border-radius: 9px; border: 1px solid #555;")

    def apply_styles(self):
        self.setStyleSheet(DARK_THEME)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # Layout tổng thể: Header (Trên) + Content (Dưới)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setSpacing(0)
        root_layout.setContentsMargins(0, 0, 0, 0)
        
        # 1. HEADER
        header = self._create_header()
        root_layout.addWidget(header)
        
        # 2. MAIN CONTENT (3 Cột)
        content_widget = QWidget()
        root_layout.addWidget(content_widget)
        
        main_layout = QHBoxLayout(content_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # ================== CỘT 1: CAMERA (Trái) ==================
        left_panel = QGroupBox("CAMERA VIEW")
        left_layout = QVBoxLayout(left_panel)
        
        self.camera_label = QLabel("Waiting for camera...")
        self.camera_label.setObjectName("camera_view")
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setScaledContents(False) # Tắt auto-scale để tránh méo ảnh
        self.camera_label.setMinimumSize(400, 300) # Đảm bảo kích thước tối thiểu 400x300
        
        left_layout.addWidget(self.camera_label)
        
        # --- Reload Camera Button ---
        self.btn_reload_camera = QPushButton("Reload Camera")
        self.btn_reload_camera.setStyleSheet("background-color: #555; color: white;")
        left_layout.addWidget(self.btn_reload_camera)
        
        main_layout.addWidget(left_panel, stretch=2) 

        # ================== CỘT 2: CONTROL (Giữa) ==================
        center_panel = QWidget()
        center_layout = QVBoxLayout(center_panel)
        center_layout.setContentsMargins(0, 0, 0, 0)

        # --- Header & Status ---
        status_group = QGroupBox("TRẠNG THÁI HỆ THỐNG")
        status_layout = QGridLayout(status_group)
        
        # Tạo các đèn báo (Label tròn)
        self.indicator_labels = {}
        ind_list = [C.INDICATOR_STM_CONNECTED, C.INDICATOR_ERROR, 
                    C.INDICATOR_CAMERA_ON, C.INDICATOR_AUTO, C.INDICATOR_MANUAL, C.INDICATOR_E_STOP]
        
        for i, name in enumerate(ind_list):
            lbl_name = QLabel(name)
            lbl_name.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            
            lbl_status = QLabel()
            lbl_status.setFixedSize(18, 18)
            lbl_status.setStyleSheet("background-color: #444; border-radius: 9px; border: 1px solid #555;")
            
            self.indicator_labels[name] = lbl_status
            status_layout.addWidget(lbl_name, 0, i*2)
            status_layout.addWidget(lbl_status, 0, i*2+1, alignment=Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            
        center_layout.addWidget(status_group)

        # --- Auto Mode Controls ---
        auto_group = QGroupBox("ĐIỀU KHIỂN TỰ ĐỘNG")
        auto_layout = QHBoxLayout(auto_group)
        
        self.btn_run = QPushButton("START")
        self.btn_run.setObjectName("btn_run")
        self.btn_run.setMinimumHeight(40)
        
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_stop.setMinimumHeight(40)
        
        auto_layout.addWidget(self.btn_run)
        auto_layout.addWidget(self.btn_stop)
        center_layout.addWidget(auto_group)

        # --- Manual Mode (REFACTORED) ---
        manual_group = QGroupBox("CHẾ ĐỘ THỦ CÔNG (MANUAL)")
        manual_main_layout = QVBoxLayout(manual_group) 

        # 1. Toggle Switch
        man_toggle_layout = QHBoxLayout()
        man_toggle_layout.addWidget(QLabel("Kích hoạt Manual:"))
        self.manual_toggle = ToggleSwitch()
        self.manual_toggle.stateChanged.connect(self._on_manual_toggled)
        man_toggle_layout.addWidget(self.manual_toggle)
        man_toggle_layout.addStretch()
        manual_main_layout.addLayout(man_toggle_layout)

        # 2. Split Layout: Trái (JOG Compact) - Phải (XYZ & Home)
        jog_xyz_container = QWidget()
        split_layout = QHBoxLayout(jog_xyz_container)
        split_layout.setContentsMargins(0, 5, 0, 5)
        
        # --- LEFT: JOG CONTROLS (COMPACT) ---
        jog_container = QWidget()
        jog_layout = QGridLayout(jog_container)
        jog_layout.setSpacing(5) # Khoảng cách nút rất nhỏ
        jog_layout.setContentsMargins(0,0,0,0)
        
        # Tạo nút JOG với kích thước cố định nhỏ gọn
        def make_jog_btn(txt, key):
            b = self._create_jog_btn(txt, key)
            b.setFixedSize(45, 45) # Vuông vức, nhỏ gọn
            return b

        self.btn_y_plus = make_jog_btn("Y+", "move_y_plus")
        self.btn_y_minus = make_jog_btn("Y-", "move_y_minus")
        self.btn_x_plus = make_jog_btn("X+", "move_x_plus")
        self.btn_x_minus = make_jog_btn("X-", "move_x_minus")
        
        # Layout 3x3 cho XY
        jog_layout.addWidget(self.btn_y_plus, 0, 1)
        jog_layout.addWidget(self.btn_x_minus, 1, 0)
        
        lbl_center = QLabel("XY")
        lbl_center.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_center.setFixedSize(45, 45)
        jog_layout.addWidget(lbl_center, 1, 1)
        
        jog_layout.addWidget(self.btn_x_plus, 1, 2)
        jog_layout.addWidget(self.btn_y_minus, 2, 1)
        
        # Z Pad (Ngay cạnh)
        self.btn_z_plus = make_jog_btn("Z+", "move_z_plus")
        self.btn_z_minus = make_jog_btn("Z-", "move_z_minus")
        
        jog_layout.addWidget(self.btn_z_plus, 0, 3)
        jog_layout.addWidget(QLabel("Z", alignment=Qt.AlignmentFlag.AlignCenter), 1, 3)
        jog_layout.addWidget(self.btn_z_minus, 2, 3)
        
        split_layout.addWidget(jog_container)
        split_layout.addSpacing(20) # Khoảng cách giữa JOG và XYZ

        # --- RIGHT: XYZ Coords & HOME & THETA ---
        xyz_container = QWidget()
        xyz_layout = QVBoxLayout(xyz_container)
        xyz_layout.setContentsMargins(0,0,0,0)
        xyz_layout.setSpacing(5)
        
        self.coord_inputs = {}
        self.theta_labels = {} # Lưu label theta tại đây
        
        # Mapping: Axis -> Theta Key -> Symbol
        axis_map = [
            ('X', 'T1', 'θ₁'),
            ('Y', 'T2', 'θ₂'),
            ('Z', 'T3', 'θ₃')
        ]
        
        for axis, theta_key, theta_sym in axis_map:
            row = QHBoxLayout()
            
            # 1. Axis Input
            row.addWidget(QLabel(f"{axis}:"))
            inp = QLineEdit("0")
            inp.setAlignment(Qt.AlignmentFlag.AlignRight)
            inp.setFixedWidth(80) # Cố định chiều rộng ô nhập (tăng lên 80 để hiển thị số dài)
            self.coord_inputs[axis] = inp
            row.addWidget(inp)
            
            row.addSpacing(10) # Khoảng cách
            
            # 2. Theta Display
            lbl_sym = QLabel(f"{theta_sym}:")
            lbl_sym.setStyleSheet("color: #bbb;")
            row.addWidget(lbl_sym)
            
            lbl_val = QLabel("0.0")
            lbl_val.setStyleSheet("color: #00FFFF; font-weight: bold; font-family: Consolas;")
            lbl_val.setFixedWidth(40)
            lbl_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.theta_labels[theta_key] = lbl_val
            row.addWidget(lbl_val)
            
            xyz_layout.addLayout(row)
            
        # Buttons Move/Home
        btn_row = QHBoxLayout()
        self.btn_move = QPushButton("MOVE")
        self.btn_home = QPushButton("HOME")
        self.btn_home.setStyleSheet("background-color: #C62828; color: white; font-weight: bold;")
        btn_row.addWidget(self.btn_move)
        btn_row.addWidget(self.btn_home)
        xyz_layout.addLayout(btn_row)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFrameShadow(QFrame.Shadow.Sunken)
        sep.setStyleSheet("background-color: #444; margin: 5px 0;")
        xyz_layout.addWidget(sep)

        # --- REORDERED CONTROLS: SERVO -> CONVEYOR -> PUMP ---

        # 1. Servo Control (Moved to Top)
        servo_row = QHBoxLayout()
        servo_row.addWidget(QLabel("Servo:"))
        
        H_BTN = 35 # Chiều cao chung cho các nút
        
        self.btn_servo_minus = QPushButton("-")
        self.btn_servo_minus.setFixedSize(35, H_BTN)
        servo_row.addWidget(self.btn_servo_minus)
        
        self.servo_slider = QSlider(Qt.Orientation.Horizontal)
        self.servo_slider.setRange(-225, 45)
        self.servo_slider.setValue(0)
        self.servo_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.servo_slider.setTickInterval(45)
        self.servo_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #555;
                height: 8px; 
                background: #333;
                margin: 2px 0;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #4CAF50;
                border: 1px solid #5c5c5c;
                width: 16px;
                margin: -4px 0; 
                border-radius: 8px;
            }
        """)
        servo_row.addWidget(self.servo_slider)
        
        self.btn_servo_plus = QPushButton("+")
        self.btn_servo_plus.setFixedSize(35, H_BTN)
        servo_row.addWidget(self.btn_servo_plus)
        
        self.servo_input = QLineEdit("0")
        self.servo_input.setFixedSize(45, H_BTN)
        self.servo_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
        servo_row.addWidget(self.servo_input)
        
        self.btn_servo_set = QPushButton("SET")
        self.btn_servo_set.setFixedSize(45, H_BTN)
        servo_row.addWidget(self.btn_servo_set)
        
        xyz_layout.addLayout(servo_row)

        # 2. Conveyor Control (Middle)
        conv_row = QHBoxLayout()
        conv_row.setSpacing(5) 
        conv_row.addWidget(QLabel("Băng tải:"))
        
        # Group Speed + Set
        speed_set_layout = QHBoxLayout()
        speed_set_layout.setSpacing(5) # Tách xa ra một chút
        speed_set_layout.setContentsMargins(0,0,0,0)
        
        self.conv_speed = QLineEdit("40")
        self.conv_speed.setFixedSize(45, H_BTN)
        self.conv_speed.setAlignment(Qt.AlignmentFlag.AlignCenter)
        speed_set_layout.addWidget(self.conv_speed)
        
        self.btn_set_speed = QPushButton("SET")
        self.btn_set_speed.setFixedSize(45, H_BTN)
        speed_set_layout.addWidget(self.btn_set_speed)
        
        conv_row.addLayout(speed_set_layout)
        
        # Spacer
        conv_row.addSpacing(10)
        
        # Action Buttons
        self.btn_conv_rev = QPushButton("REV")
        self.btn_conv_rev.setMinimumHeight(H_BTN) 
        conv_row.addWidget(self.btn_conv_rev, 1) 
        
        self.btn_conv_fwd = QPushButton("FWD")
        self.btn_conv_fwd.setMinimumHeight(H_BTN)
        conv_row.addWidget(self.btn_conv_fwd, 1) 
        
        xyz_layout.addLayout(conv_row)

        # 3. Pump Control (Moved to Bottom)
        pump_row = QHBoxLayout()
        pump_row.addWidget(QLabel("Bơm hút:"))
        self.pump_toggle = ToggleSwitch()
        pump_row.addWidget(self.pump_toggle)
        pump_row.addStretch()
        xyz_layout.addLayout(pump_row)
        
        # Thêm stretch để đẩy tất cả lên trên
        xyz_layout.addStretch()
        
        split_layout.addWidget(xyz_container, stretch=1) # XYZ chiếm 1 phần
        manual_main_layout.addWidget(jog_xyz_container)

        # 3. SERVO (Bottom Row) - REMOVED
        
        center_layout.addWidget(manual_group)

        # --- Monitor (Robot info) ---
        monitor_group = QGroupBox("GIÁM SÁT SẢN PHẨM")
        monitor_group.setMinimumHeight(230)
        monitor_layout = QVBoxLayout(monitor_group)
        monitor_layout.setContentsMargins(5, 5, 5, 5)
        
        # --- 4 Product Types Grid ---
        detail_container = QWidget()
        detail_layout = QGridLayout(detail_container)
        detail_layout.setSpacing(10)
        detail_layout.setContentsMargins(0,0,0,0)
        
        self.lbl_product_details = {}
        # Names based on data.yaml: 0: chuoi, 1: dau, 2: kiwi, 3: socola
        product_names = ["Chuối", "Dâu", "Kiwi", "Socola"]
        p_colors = ["#FFEB3B", "#FF5252", "#66BB6A", "#8D6E63"] # Yellow, Red, Green, Brown
        
        for i, name in enumerate(product_names):
            frame = QFrame()
            frame.setMinimumHeight(70)
            frame.setStyleSheet("background-color: #2a2a2a; border: 1px solid #3e3e3e; border-radius: 5px;")
            f_layout = QVBoxLayout(frame)
            f_layout.setContentsMargins(2, 5, 2, 5)
            f_layout.setSpacing(2)
            
            lbl_title = QLabel(name)
            lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl_title.setStyleSheet("font-size: 12px; color: #ccc; font-weight: bold;")
            
            lbl_cnt = QLabel("0")
            lbl_cnt.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl_cnt.setStyleSheet(f"font-size: 22px; color: {p_colors[i]}; font-weight: bold;")
            
            self.lbl_product_details[i] = lbl_cnt
            
            f_layout.addWidget(lbl_title)
            f_layout.addWidget(lbl_cnt)
            
            row = i // 2
            col = i % 2
            detail_layout.addWidget(frame, row, col)
            
        monitor_layout.addWidget(detail_container)
        monitor_layout.addSpacing(5)
            
        # Product Count Row
        count_row = QHBoxLayout()
        lbl_prod_title = QLabel("Tổng sản phẩm:")
        lbl_prod_title.setStyleSheet("font-size: 16px; font-weight: bold;")
        count_row.addWidget(lbl_prod_title)
        
        self.lbl_prod_count = QLabel("0")
        self.lbl_prod_count.setStyleSheet("font-size: 32px; color: #00FF00; font-weight: bold; padding: 0 5px;")
        count_row.addWidget(self.lbl_prod_count)
        
        count_row.addStretch() # Đẩy sang trái

        btn_reset_count = QPushButton("Reset")
        btn_reset_count.setFixedSize(60, 30)
        btn_reset_count.clicked.connect(self.reset_all_counts)
        count_row.addWidget(btn_reset_count)
        
        monitor_layout.addLayout(count_row)
        
        monitor_layout.addStretch()

        center_layout.addWidget(monitor_group)
        main_layout.addWidget(center_panel, stretch=3) 

        # ================== CỘT 3: SETTINGS & LOG (Phải) ==================
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        # --- Connection ---
        conn_group = QGroupBox("KẾT NỐI")
        conn_layout = QVBoxLayout(conn_group)
        
        self.cbb_ports = QComboBox()
        conn_layout.addWidget(self.cbb_ports)
        
        hbox_conn = QHBoxLayout()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        # ✅ LUÔN BẬT, kiểm tra trạng thái bên trong hàm
        hbox_conn.addWidget(self.btn_refresh)
        hbox_conn.addWidget(self.btn_connect)
        hbox_conn.addWidget(self.btn_disconnect)
        conn_layout.addLayout(hbox_conn)
        
        right_layout.addWidget(conn_group, stretch=1) # Ít ưu tiên

        # --- Log ---
        log_group = QGroupBox("LOG HỆ THỐNG")
        log_layout = QVBoxLayout(log_group)
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        log_layout.addWidget(self.txt_log)
        
        btn_clear_log = QPushButton("Xóa Log")
        btn_clear_log.clicked.connect(self.txt_log.clear)
        log_layout.addWidget(btn_clear_log)
        
        right_layout.addWidget(log_group, stretch=10) # Log sẽ chiếm hết phần còn lại
        
        main_layout.addWidget(right_panel, stretch=2) # Chiếm 2 phần

        # --- Event Binds (Internal) ---
        self.servo_slider.valueChanged.connect(lambda v: self.servo_input.setText(str(v)))
        self.servo_input.editingFinished.connect(lambda: self.servo_slider.setValue(int(float(self.servo_input.text()))))
        self.btn_refresh.clicked.connect(self.refresh_ports)

    def _create_header(self):
        header_frame = QFrame()
        header_frame.setStyleSheet("background-color: #1565C0;") # Nền xanh dương, bỏ viền
        layout = QVBoxLayout(header_frame)
        layout.setSpacing(5)
        
        lbl_uni = QLabel("ĐẠI HỌC SƯ PHẠM KỸ THUẬT")
        lbl_uni.setStyleSheet("font-size: 22px; font-weight: bold; color: white;")
        lbl_uni.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        lbl_proj = QLabel("HỆ THỐNG ROBOT DELTA SẮP XẾP VẬT")
        lbl_proj.setStyleSheet("font-size: 28px; font-weight: bold; color: white;")
        lbl_proj.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        layout.addWidget(lbl_uni)
        layout.addWidget(lbl_proj)
        return header_frame

    def _create_jog_btn(self, text, cmd_key):
        btn = QPushButton(text)
        btn.setObjectName("btn_jog")
        # Lưu cmd_key vào property của nút để dùng sau
        btn.setProperty("cmd_key", cmd_key)
        return btn

    # ===============================================================
    #           BACKEND COMPATIBILITY INTERFACE (ADAPTERS)
    # ===============================================================
    # NOTE: Old Tkinter variable wrappers have been removed.
    # Use direct accessor methods defined below.
    
    @property
    def servo_entry(self):
        """DEPRECATED: Use get_servo_entry_value() instead."""
        class EntryWrapper:
            def __init__(self, widget): self.widget = widget
            def get(self): return self.widget.text()
            def bind(self, event, func): pass 
        return EntryWrapper(self.servo_input)

    # --- Timer Management ---
    @pyqtSlot(object, object)
    def _handle_run_on_main(self, func, args):
        """Execute function on main thread."""
        try:
            func(*args)
        except Exception as e:
            print(f"Error in run_on_main: {e}")

    def after(self, ms, func, *args):
        """Wrapper for QTimer.singleShot to mimic Tkinter's after.
        Now thread-safe using signals."""
        if ms == 0:
            # Execute ASAP on main thread
            self.run_on_main_signal.emit(func, args)
        else:
            # For delayed execution, we also want to start the timer on the main thread
            # to ensure the callback runs on the main thread.
            def start_timer_on_main():
                QTimer.singleShot(ms, lambda: func(*args))
            self.run_on_main_signal.emit(start_timer_on_main, ())
        return "single_shot_timer"

    def process_events(self):
        """Wrapper for QApplication.processEvents() to keep GUI responsive."""
        QApplication.processEvents()

    def start_timer(self, ms, func, *args):
        """Starts a repeating timer (or single shot that can be stored). Returns QTimer object."""
        timer = QTimer(self)
        timer.setInterval(ms)
        timer.timeout.connect(lambda: func(*args))
        timer.start()
        return timer

    def stop_timer(self, timer):
        """Stops a timer started with start_timer."""
        if isinstance(timer, QTimer):
            timer.stop()
            timer.deleteLater()

    # --- Accessor Methods (New API) ---
    def get_selected_port(self):
        return self.cbb_ports.currentText()
        
    def set_selected_port(self, port_name):
        self.cbb_ports.setCurrentText(port_name)

    def get_coordinate(self, axis):
        if axis in self.coord_inputs:
            try:
                return float(self.coord_inputs[axis].text())
            except ValueError: return 0.0
        return 0.0

    def set_coordinate(self, axis, value):
        """Thread-safe: Emit signal to update coordinate"""
        print(f"[DEBUG] set_coordinate called: axis={axis}, value={value}")
        self.coord_signal.emit(axis, str(value))
        print(f"[DEBUG] coord_signal emitted")

    def get_theta(self, axis_key):
        """axis_key: 'T1', 'T2', 'T3'"""
        if axis_key in self.theta_labels:
            try:
                return float(self.theta_labels[axis_key].text())
            except ValueError: return 0.0
        return 0.0

    def set_theta(self, t1, t2, t3):
        """Thread-safe: Emit signal to update theta"""
        self.theta_signal.emit(float(t1), float(t2), float(t3))
        
    def get_servo_angle(self):
        return self.servo_slider.value()
        
    def set_servo_angle(self, angle):
        """Sets servo angle on slider and input box."""
        self.servo_slider.blockSignals(True)
        self.servo_slider.setValue(int(angle))
        self.servo_input.setText(str(angle))
        self.servo_slider.blockSignals(False)

    def get_servo_entry_value(self):
        return self.servo_input.text()
        
    def set_servo_entry_value(self, value):
        self.servo_input.setText(str(value))

    def get_conveyor_speed(self):
        return self.conv_speed.text()
        
    def set_conveyor_speed(self, value):
        self.conv_speed.setText(str(value))

    def get_product_count(self):
        try:
            return int(self.lbl_prod_count.text())
        except ValueError: return 0
        
    def set_product_count(self, count):
        self.lbl_prod_count.setText(str(count))

    def set_detail_count(self, idx, count):
        if idx in self.lbl_product_details:
             self.lbl_product_details[idx].setText(str(count))

    def get_detail_count(self, idx):
        if idx in self.lbl_product_details:
             try:
                 return int(self.lbl_product_details[idx].text())
             except ValueError: return 0
        return 0

    def reset_all_counts(self):
        self.set_product_count(0)
        for i in self.lbl_product_details:
            self.set_detail_count(i, 0)

    def is_manual_mode(self):
        return self.manual_toggle.isChecked()

    def set_manual_mode(self, enabled):
        self.manual_toggle.set_state(enabled)

    def set_manual_mode_enabled(self, enabled):
        """Enable/Disable the manual toggle switch interaction."""
        self.manual_toggle.setEnabled(enabled)

    def set_pump_enabled(self, enabled):
        self.pump_toggle.setEnabled(enabled)

    def set_pump_state(self, state):
        self.pump_toggle.set_state(state)

    def destroy(self):
        """Cleanup."""
        if self.conn_manager and self.conn_manager.is_connected():
            self.conn_manager.shutdown()
        self.close()

    # --- Logic Methods ---
    def set_button_commands(self, commands):
        """Gán callback cho các nút."""
        self.button_callbacks = commands
        
        # Click events
        if 'run' in commands: self.btn_run.clicked.connect(commands['run'])
        if 'stop' in commands: self.btn_stop.clicked.connect(commands['stop'])
        if 'move' in commands: self.btn_move.clicked.connect(commands['move'])
        if 'home' in commands: self.btn_home.clicked.connect(commands['home'])
        if 'connect_stm' in commands: self.btn_connect.clicked.connect(commands['connect_stm'])
        if 'disconnect_stm' in commands: self.btn_disconnect.clicked.connect(commands['disconnect_stm'])
        if 'conveyor_set_speed' in commands: self.btn_set_speed.clicked.connect(commands['conveyor_set_speed'])
        
        if 'pump' in commands: 
            # Toggle switch logic
            self.pump_toggle.stateChanged.connect(lambda s: commands['pump'](s))

        if 'servo_set_angle' in commands:
            # Slider release event
            self.servo_slider.sliderReleased.connect(commands['servo_set_angle'])
        
        if 'servo_set_from_entry' in commands:
            self.servo_input.returnPressed.connect(commands['servo_set_from_entry'])
            self.btn_servo_set.clicked.connect(commands['servo_set_from_entry'])
            
        if 'servo_nudge_plus' in commands:
            self.btn_servo_plus.clicked.connect(commands['servo_nudge_plus'])
            
        if 'servo_nudge_minus' in commands:
            self.btn_servo_minus.clicked.connect(commands['servo_nudge_minus'])

        # Băng tải giữ nút (Hold button)
        if 'conveyor_fwd_press' in commands: self.btn_conv_fwd.pressed.connect(commands['conveyor_fwd_press'])
        if 'conveyor_release' in commands: self.btn_conv_fwd.released.connect(commands['conveyor_release'])
        
        if 'conveyor_rev_press' in commands: self.btn_conv_rev.pressed.connect(commands['conveyor_rev_press'])
        if 'conveyor_release' in commands: self.btn_conv_rev.released.connect(commands['conveyor_release'])

    def log_message(self, message, msg_type="sent"):
        """Thread-safe logging."""
        self.log_signal.emit(message, msg_type)

    @pyqtSlot(str, str)
    def _handle_log_signal(self, message, msg_type):
        color = "#00FF00" if msg_type == "sent" else "#00BFFF" # Green vs Blue
        if msg_type == "error": color = "#FF0000"
        
        timestamp = datetime.now().strftime("[%H:%M:%S]")

        # --- WRITE TO LOG FILE ---
        try:
            with open(self.log_filename, "a", encoding="utf-8") as f:
                f.write(f"{timestamp} [{msg_type.upper()}] {message}\n")
        except Exception as e:
            print(f"Failed to write to log file: {e}")

        html = f'<span style="color:{color};">{timestamp} [{msg_type.upper()}] {message}</span>'
        
        # Insert at top (Prepend)
        cursor = self.txt_log.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.Start)
        self.txt_log.setTextCursor(cursor)
        self.txt_log.insertHtml(html + "<br>")

    def update_camera_feed(self, frame):
        """Nhận numpy array (RGB) từ thread và hiển thị."""
        if frame is not None:
            self.image_signal.emit(frame)

    @pyqtSlot(np.ndarray)
    def _handle_image_signal(self, frame):
        """Convert numpy array to QPixmap and display."""
        # Đảm bảo dữ liệu liền mạch trong bộ nhớ để tăng tốc độ QImage
        if not frame.flags['C_CONTIGUOUS']:
            frame = np.ascontiguousarray(frame)
        
        # Lấy kích thước label để resize frame đúng tỷ lệ
        label_width = self.camera_label.width()
        label_height = self.camera_label.height()
        
        # Chỉ resize nếu label có kích thước hợp lệ
        if label_width > 1 and label_height > 1:
            frame_h, frame_w = frame.shape[:2]
            
            # Tính tỷ lệ scale để fit vào label mà không méo
            scale_w = label_width / frame_w
            scale_h = label_height / frame_h
            scale = min(scale_w, scale_h)  # Dùng scale nhỏ hơn để fit cả 2 chiều
            
            # Resize frame đúng tỷ lệ
            new_w = int(frame_w * scale)
            new_h = int(frame_h * scale)
            
            if new_w > 0 and new_h > 0:
                frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        # Frame từ video_thread đã là RGB
        qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        self.camera_label.setPixmap(QPixmap.fromImage(qt_image))

    def update_indicator(self, name, state):
        self.indicator_signal.emit(name, state)

    @pyqtSlot(str, bool)
    def _handle_indicator_signal(self, name, state):
        if name == C.INDICATOR_HOME_OK:
            # Update HOME button color directly
            color = "#2E7D32" if state else "#C62828" # Green if homed, Red if not
            self.btn_home.setStyleSheet(f"background-color: {color}; color: white; font-weight: bold;")
            return

        if name == C.INDICATOR_E_STOP:
            if state:
                if not self.estop_blink_timer.isActive():
                    self.estop_blink_timer.start()
                    self.estop_led_state = True # Bắt đầu sáng ngay
                    self.indicator_labels[name].setStyleSheet("background-color: #F44336; border-radius: 9px; border: 1px solid #555;")
            else:
                if self.estop_blink_timer.isActive():
                    self.estop_blink_timer.stop()
                # Tắt đèn
                self.indicator_labels[name].setStyleSheet("background-color: #444; border-radius: 9px; border: 1px solid #555;")
            return

        if name in self.indicator_labels:
            color = "#4CAF50" if state else "#444" # Green vs Grey
            if name == C.INDICATOR_ERROR and state: color = "#F44336"
            # Use border-radius to keep it a circle
            self.indicator_labels[name].setStyleSheet(f"background-color: {color}; border-radius: 9px; border: 1px solid #555;")

    @pyqtSlot(str, str)
    def _handle_coord_signal(self, axis, value):
        """Thread-safe handler for coordinate updates"""
        if axis in self.coord_inputs:
            widget = self.coord_inputs[axis]
            # ✅ FIX: Chỉ cập nhật nếu người dùng KHÔNG đang nhập liệu vào ô này
            if not widget.hasFocus():
                widget.setText(value)
                # print(f"[GUI SIGNAL] Updated coord {axis} = {value}")  # DEBUG (Disabled to reduce noise)

    @pyqtSlot(float, float, float)
    def _handle_theta_signal(self, t1, t2, t3):
        """Thread-safe handler for theta updates"""
        self.theta_labels['T1'].setText(f"{t1:.1f}")
        self.theta_labels['T2'].setText(f"{t2:.1f}")
        self.theta_labels['T3'].setText(f"{t3:.1f}")
        print(f"[GUI SIGNAL] Updated theta: {t1:.1f}, {t2:.1f}, {t3:.1f}")  # DEBUG

    def get_indicator_state(self, name):
        # Trả về logic state
        if name == C.INDICATOR_HOME_OK:
            style = self.btn_home.styleSheet()
            # Green color means Home OK
            return "#2E7D32" in style

        if name in self.indicator_labels:
            style = self.indicator_labels[name].styleSheet()
            return "#4CAF50" in style or "red" in style # Hacky check but works
        return False

    def get_coordinate(self, axis):
        if axis in self.coord_inputs:
            try:
                return float(self.coord_inputs[axis].text())
            except: return 0.0
        return 0.0

    def set_product_count(self, count):
        self.lbl_prod_count.setText(str(count))
        
    def get_product_count(self):
        return int(self.lbl_prod_count.text())

    def get_conveyor_speed(self):
        return self.conv_speed.text()

    def is_manual_mode(self):
        return self.manual_toggle.isChecked()

    def refresh_ports(self):
        ports = list_ports()
        self.cbb_ports.clear()
        self.cbb_ports.addItems(ports)

    def _on_manual_toggled(self, state):
        # Logic gọi callback manual toggle
        if hasattr(self, '_on_manual_toggle_callback'):
            self._on_manual_toggle_callback(state)
            
    # Wrapper để map callback từ main.py vào toggle switch của PyQt
    # Trong main.py dòng 186: app.toggle.command = self._on_manual_toggle
    # Chúng ta cần fake thuộc tính 'command' cho toggle switch
    # Tuy nhiên cách tốt nhất là override trong set_button_commands hoặc gán trực tiếp.
    
    # --- Specific Override for JOG Buttons ---
    # Vì logic JOG trong Tkinter được bind vào ButtonPress/Release riêng lẻ
    # Chúng ta cần một cơ chế để main.py bind được vào các nút Qt
    
    # Phương thức hỗ trợ để main.py gọi bind()
    def bind_jog(self, axis_name, press_func, release_func):
        """Hàm helper để bind event Jog cho các nút PyQt"""
        btn = None
        if axis_name == 'Y+': btn = self.btn_y_plus
        elif axis_name == 'Y-': btn = self.btn_y_minus
        elif axis_name == 'X+': btn = self.btn_x_plus
        elif axis_name == 'X-': btn = self.btn_x_minus
        elif axis_name == 'Z+': btn = self.btn_z_plus
        elif axis_name == 'Z-': btn = self.btn_z_minus
        
        if btn:
            btn.pressed.connect(press_func)
            btn.released.connect(release_func)

    # --- Update State Methods ---
    def _update_toggle_switches_state(self, enabled):
        self.manual_toggle.setEnabled(enabled)
        self.pump_toggle.setEnabled(enabled)

    # Các hàm thừa kế từ Tkinter mà có thể bị gọi
    def winfo_exists(self):
        return self.isVisible()

    def handle_camera_error(self, message):
        """Xử lý khi camera bị lỗi (được gọi từ main thread)."""
        self.log_message(f"❌ {message}", "error")
        self.update_indicator(C.INDICATOR_CAMERA_ON, False)
        self.update_indicator(C.INDICATOR_ERROR, True)
        
        # Cập nhật đèn nút bấm (sẽ tắt vì camera lỗi)
        # Và BẬT ĐÈN ĐỎ TRÊN MẠCH (ERROR STATE)
        if self.robot_controller:
            self.robot_controller.update_hardware_button_leds()
            self.robot_controller.set_system_led(2) # 2 = ERROR (Đỏ tĩnh)
        
        # Nếu đang chạy Auto, dừng ngay lập tức
        if self.robot_controller and self.robot_controller.is_running_auto:
            self.log_message("⚠️ Dừng AUTO do mất kết nối camera!", "error")
            self.robot_controller.set_auto_mode(False)
            
            # Bật đèn Error đỏ lòm
            self.update_indicator(C.INDICATOR_ERROR, True)
            
            # Show Dialog cảnh báo (Non-blocking)
            QMessageBox.critical(self, "Camera Error", "Mất kết nối Camera khi đang chạy Auto!\nHệ thống đã dừng khẩn cấp.")
