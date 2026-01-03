# constants.py

# =============================================================================
# --- Tên các đèn báo (Indicator) và Widget trên GUI ---
# =============================================================================

# --- Top Bar ---
INDICATOR_AUTO = "AUTO"
INDICATOR_MANUAL = "MANUAL"
INDICATOR_ERROR = "ERROR"

# --- Control Status Frame ---
INDICATOR_STM_CONNECTED = "Kết nối STM32"
INDICATOR_CAMERA_ON = "CAMERA ON"
INDICATOR_E_STOP = "E-STOP"
INDICATOR_MANUAL_OFF = "MANUAL OFF"

# --- Monitor Frame ---
INDICATOR_CONVEYOR_1 = "Băng tải 1"
INDICATOR_PUMP = "Bơm"
INDICATOR_TRAY_SENSOR = "Cảm biến Khay"
INDICATOR_SENSOR_1 = "Cảm biến 1"

# --- Manual Frame ---
INDICATOR_HOME_OK = "HOME OK"
INDICATOR_LS_1 = "LS 1"
INDICATOR_LS_2 = "LS 2"
INDICATOR_LS_3 = "LS 3"


# =============================================================================
# --- Màu sắc ---
# =============================================================================
COLOR_BG = "#E3F2FD"
COLOR_FRAME_BG = "#FFFFFF"
COLOR_HEADER_BG = "#1565C0"

COLOR_INDICATOR_ON = "#4CAF50"
COLOR_INDICATOR_OFF = "#d9d9d9"
COLOR_ERROR_ON = "#f44336"

COLOR_BUTTON_RUN_BG = "#4CAF50"
COLOR_BUTTON_STOP_BG = "#f44336"
COLOR_BUTTON_FG = "white"

COLOR_TEXT_SENT = "#1565C0"
COLOR_TEXT_RECEIVED = "#4CAF50"
COLOR_TEXT_VALUE = "#0D47A1"

# =============================================================================
# --- Font Styles ---
# =============================================================================
FONT_FAMILY_UI = "Segoe UI"

# =============================================================================
# --- Cài đặt khác ---
# =============================================================================
SERVO_NUDGE_AMOUNT = 5 # Độ lớn bước nhảy của servo khi nhấn nút "+" / "-"

# === Camera Calibration ===
# Đường dẫn file config camera calibration (tạo bởi camera_calibration_tool.py)
import os
CAMERA_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "camera_calibration.json")


# =============================================================================

# --- Giới hạn không gian làm việc (Workspace Limits) ---

# =============================================================================

# Giới hạn góc của cánh tay robot (độ).

# Đây là cách kiểm tra không gian làm việc chính xác nhất.

ARM_ANGLE_MIN = -41.0

ARM_ANGLE_MAX = 70.0

# =============================================================================
# --- Thông số Robot & Homing ---
# =============================================================================

# --- CẤU HÌNH HÌNH HỌC ROBOT (KINEMATICS) ---
# [QUAN TRỌNG] Cần đo chính xác bằng thước kẹp để robot di chuyển đúng quãng đường.
# Hiện tượng:
# - Nếu robot đi NGẮN hơn lệnh (bảo 10cm đi 9cm) -> GIẢM giá trị RF hoặc RE (Code đang nghĩ tay dài hơn thực tế).
# - Nếu robot đi DÀI hơn lệnh -> TĂNG giá trị RF hoặc RE.

ROBOT_E_SIDE = 139    # Cạnh tam giác effector (mm) 
ROBOT_F_SIDE = 208    # Cạnh tam giác đế (mm) 
ROBOT_RE_LOWER = 322.0  # Cánh tay dưới (mm) - Passive arm (Thanh song song). Đo từ tâm khớp trên đến tâm khớp dưới.
ROBOT_RF_UPPER = 148.5  # Cánh tay trên (mm) - Active arm (Gắn motor). Đo từ tâm trục motor đến tâm khớp nối.

# --- CẤU HÌNH ĐỘNG CƠ (MOTOR) ---
# Calib quãng đường: Nếu đi thiếu, hãy TĂNG Gear Ratio hoặc Steps/Rev
MOTOR_STEPS_PER_REV = 800           # Microstep 1/4: 200 * 4 = 800
MOTOR_GEAR_RATIO = 3969.0 / 289.0   # Tỷ số truyền hộp số (~13.733)

# Tốc độ động cơ cho các thao tác thông thường (Move, Jog)
# [UPDATE 02/12] Giảm tốc độ JOG cho microstep 1/8
# Lý do: Mô-men xoắn thấp hơn ở microstep cao, cần tốc độ thấp để tránh giật
JOG_SPEED_RPM = 70   # Tốc độ JOG (giảm từ 85 → 60 RPM)

# Cartesian Path Planning - Độ dài mỗi segment khi nội suy궤đạo thẳng
# Đơn vị: mm
# Giá trị đề xuất: 3-10mm
# - Nhỏ hơn (2-3mm): Chính xác hơn, mượt hơn, nhưng nhiều tính toán và blocks
# - Lớn hơn (10-15mm): Nhanh hơn, ít blocks, nhưng có thể lệch궤đạo
CARTESIAN_SEGMENT_LENGTH_MM = 5.0

# Góc (theta) của các tay robot tại vị trí Home
# LƯU Ý: Phải nằm trong range [ARM_ANGLE_MIN, ARM_ANGLE_MAX]
# Khi STM32 homing (chạm limit switch), cánh tay NÂNG LÊN ở góc 40° so với phương ngang
# Quy ước: góc âm = nâng lên, góc dương = hạ xuống -> theta = -40°
HOME_DEFAULT_THETA_DEG = -40.0  
# Góc của servo tại vị trí Home (Coordinate System: -225 to 45)
HOME_DEFAULT_SERVO_ANGLE_DEG = -90.0




# Kiểm tra xem góc servo có nằm trong phạm vi hợp lý (-225 đến 45 độ) không.
assert -225 <= HOME_DEFAULT_SERVO_ANGLE_DEG <= 45, (
    f"Góc servo mặc định ({HOME_DEFAULT_SERVO_ANGLE_DEG}) nằm ngoài phạm vi -225 đến 45."
)

# Kiểm tra xem bước nhảy servo có phải là một giá trị dương hợp lý không.
assert SERVO_NUDGE_AMOUNT > 0, "SERVO_NUDGE_AMOUNT phải là số dương."

# Cấu hình Motion Planner (Trapezoidal) ---
# =============================================================================
# Các thông số này được sử dụng trong file: GUI_PC/motion_planner_trapezoidal.py

# --- GIỚI HẠN PHẦN CỨNG (RPM) ---
# Tốc độ khởi động (RPM)
TRAJ_V_START_RPM = 80

# Tốc độ tối đa (RPM) - Giới hạn vật lý của motor để tránh trượt bước
TRAJ_V_MAX_RPM = 300 

# --- CẤU HÌNH TỐC ĐỘ ĐẦU HÚT (LINEAR SPEED - mm/s) ---
# Robot sẽ cố gắng di chuyển với vận tốc này
TRAJ_LINEAR_V_MAX_MM_S = 500.0   # Tốc độ chạy ổn định (mm/s)
TRAJ_LINEAR_ACCEL_MM_S2 = 2000.0 # Gia tốc (mm/s^2)
TRAJ_LINEAR_V_START_MM_S = 20.0   # Tốc độ bắt đầu (mm/s)

# Thời gian tăng tốc (giây) - CŨ (Chỉ dùng nếu tính theo RPM)
TRAJ_ACCEL_TIME = 0.15

# Thời gian mỗi phân đoạn (giây) - Update rate cho STM32
TRAJ_SEGMENT_TIME = 0.020

# Junction deviation (mm) - Sai số cho phép tại các góc cua (Cornering)
# Giá trị càng lớn -> cua càng nhanh nhưng càng "lượn" (kém chính xác tại góc)
# Giá trị càng nhỏ -> cua càng chậm nhưng càng bám sát quỹ đạo gốc
TRAJ_JUNCTION_DEVIATION = 0.05


from enum import Enum, auto

# =============================================================================
# --- State Enums ---
# =============================================================================

class RobotState(Enum):
    """Trạng thái chính của robot controller."""
    IDLE = "idle"           # Robot nhàn rỗi, sẵn sàng nhận lệnh
    MOVING = "moving"       # Đang thực hiện lệnh di chuyển
    HOMING = "homing"       # Đang thực hiện quá trình homing
    JOGGING = "jogging"     # Đang jog (di chuyển liên tục)
    ESTOP = "estop"         # Emergency stop đã kích hoạt

class HomingStatus(Enum):
    """Trạng thái chi tiết của quá trình Homing."""
    NOT_HOMED = auto()
    IN_PROGRESS = auto()
    COMPLETED = auto()
    FAILED = auto()


# =============================================================================
# --- LOG UTILS ---
# =============================================================================
import time
import threading

_lock = threading.Lock()
_last_message = None
_last_time = 0.0
_dedupe_window_sec = 1.0

def log_dedupe(message: str, level: str = "info"):
    """Print a log message, suppressing duplicates within _dedupe_window_sec.

    Args:
        message: The message string to log.
        level: A simple level tag (info, warn, error, debug).
    """
    global _last_message, _last_time
    now = time.time()
    with _lock:
        full_message = f"[{level.upper()}] {message}"
        if _last_message == full_message and (now - _last_time) < _dedupe_window_sec:
            return  # Suppress duplicate
        _last_message = full_message
        _last_time = now
    print(full_message)

def set_dedupe_window(seconds: float):
    global _dedupe_window_sec
    with _lock:
        _dedupe_window_sec = max(0.0, seconds)