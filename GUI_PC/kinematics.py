import math
import numpy as np
from numba import jit
import constants as C

# ============================================================================
# NUMBA-OPTIMIZED CORE FUNCTIONS (20-100x faster)
# ============================================================================
# Numba không hỗ trợ methods với 'self', nên tách thành static functions
# Các hằng số phải truyền vào như parameters

@jit(nopython=True, cache=True)
def _calc_angle_yz_numba(x0, y0, z0, tan30, f, e, rf, re, pi):
    """
    Numba-optimized: Tính góc theta cho 1 cánh tay.
    Tốc độ: ~50-100x nhanh hơn Python thuần.
    """
    # Hằng số f/2 * tan30
    y1 = -0.5 * tan30 * f
    
    # Trừ offset effector
    y0_mod = y0 - 0.5 * tan30 * e
    
    if abs(z0) < 1e-9:
        return np.nan  # Tránh chia cho 0

    # z = a + b*y
    a = (x0**2 + y0_mod**2 + z0**2 + rf**2 - re**2 - y1**2) / (2.0 * z0)
    b = (y1 - y0_mod) / z0
    
    # discriminant
    d = -(a+b*y1)**2 + rf*(b*b*rf + rf)
    
    # Nếu d < 0, điểm không tồn tại
    if d < 1e-9:
        return np.nan
    
    # choosing outer point
    yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1.0)
    zj = a + b * yj
    
    # Ngăn chia cho 0 nếu yj == y1
    if abs(y1 - yj) < 1e-9:
        if zj < 0:
            theta_deg = -90.0
        else:
            theta_deg = 90.0
    else:
        theta_rad = math.atan(-zj / (y1 - yj))
        theta_deg = 180.0 * theta_rad / pi

    # Xử lý góc phần tư
    if yj > y1:
        theta_deg += 180.0
        
    return theta_deg


@jit(nopython=True, cache=True)
def inverse_kinematics_numba(x0, y0, z0, tan30, f, e, rf, re, pi, sqrt3, 
                             cos120, sin120, arm_angle_min, arm_angle_max):
    """
    Numba-optimized: Tính động học nghịch cho TÂM BỆ.
    Trả về (theta1, theta2, theta3) hoặc (nan, nan, nan) nếu thất bại.
    """
    # Đảo dấu X để phù hợp với hệ tọa độ robot
    x0_corrected = x0
    y0_corrected = -y0
    
    # Cánh tay 1
    theta1 = _calc_angle_yz_numba(x0_corrected, y0_corrected, z0, 
                                   tan30, f, e, rf, re, pi)
    
    # Cánh tay 2 (xoay -120 độ)
    x_rot2 = x0_corrected * cos120 + y0_corrected * sin120
    y_rot2 = y0_corrected * cos120 - x0_corrected * sin120
    theta2 = _calc_angle_yz_numba(x_rot2, y_rot2, z0, 
                                   tan30, f, e, rf, re, pi)

    # Cánh tay 3 (xoay +120 độ)
    x_rot3 = x0_corrected * cos120 - y0_corrected * sin120
    y_rot3 = y0_corrected * cos120 + x0_corrected * sin120
    theta3 = _calc_angle_yz_numba(x_rot3, y_rot3, z0, 
                                   tan30, f, e, rf, re, pi)

    # Kiểm tra thất bại
    if np.isnan(theta1) or np.isnan(theta2) or np.isnan(theta3):
        return np.nan, np.nan, np.nan

    # Kiểm tra giới hạn góc
    if not (arm_angle_min <= theta1 <= arm_angle_max):
        return np.nan, np.nan, np.nan
    if not (arm_angle_min <= theta2 <= arm_angle_max):
        return np.nan, np.nan, np.nan
    if not (arm_angle_min <= theta3 <= arm_angle_max):
        return np.nan, np.nan, np.nan

    return theta1, theta2, theta3


@jit(nopython=True, cache=True)
def inverse_kinematics_tool_numba(xt, yt, zt, alpha_deg, 
                                  tool_offset_radius, tool_offset_z,
                                  tan30, f, e, rf, re, pi, sqrt3, 
                                  cos120, sin120, arm_angle_min, arm_angle_max):
    """
    Numba-optimized: Tính động học nghịch cho ĐẦU HÚT với tool offset.
    """
    alpha_rad = math.radians(alpha_deg)
    
    # Tính (x0, y0, z0) của tâm bệ TỪ (xt, yt, zt) của đầu hút
    x0 = xt - tool_offset_radius * math.cos(alpha_rad)
    y0 = yt - tool_offset_radius * math.sin(alpha_rad)
    z0 = zt + tool_offset_z
    
    # Gọi hàm IK chuẩn
    return inverse_kinematics_numba(x0, y0, z0, tan30, f, e, rf, re, pi, sqrt3,
                                   cos120, sin120, arm_angle_min, arm_angle_max)


@jit(nopython=True, cache=True)
def calc_trapezoidal_time_numba(dist_mm, max_steps, v_mm_s, a_mm_s2, v_hz_limit, segment_time):
    """
    Tính thời gian di chuyển chính xác mô phỏng theo Motion Planner thực tế.
    Trả về thời gian (giây) đã được Quantized theo Segment Time (Block).
    """
    if dist_mm < 0.001: return 0.0
    
    # 1. Tính toán Profile theo Cartesian (mm/s)
    # Thời gian tăng tốc để đạt V_max
    t_accel_ideal = v_mm_s / a_mm_s2
    s_accel_ideal = 0.5 * a_mm_s2 * (t_accel_ideal ** 2)
    
    # Kiểm tra có đạt được V_max không (Profile Hình thang hay Tam giác)
    if (2 * s_accel_ideal) > dist_mm:
        # Không đạt V_max -> Profile Tam giác
        # s_total = a * t_accel^2  (vì s_accel = s_decel)
        t_accel = math.sqrt(dist_mm / a_mm_s2)
        t_total_ideal = 2 * t_accel
    else:
        # Đạt V_max -> Profile Hình thang
        s_const = dist_mm - 2 * s_accel_ideal
        t_const = s_const / v_mm_s
        t_total_ideal = 2 * t_accel_ideal + t_const
        
    # 2. Kiểm tra giới hạn tốc độ động cơ (Motor Hz Limit)
    # Đây là bước quan trọng khiến thực tế chậm hơn lý thuyết
    if t_total_ideal > 0:
        required_hz = max_steps / t_total_ideal
        if required_hz > v_hz_limit:
            # Nếu vượt quá Hz cho phép, kéo dài thời gian ra
            t_total_ideal = max_steps / v_hz_limit
            
    # 3. Quantization (Chia thành các blocks 20ms)
    # Mô phỏng việc cắt lớp thời gian
    min_time = 2 * segment_time # Tối thiểu 2 block (Start + End)
    if t_total_ideal < min_time:
        t_total_ideal = min_time
        
    # Làm tròn lên số lượng block
    num_blocks = math.ceil(t_total_ideal / segment_time)
    
    # Thời gian thực tế = Số block * 20ms
    return num_blocks * segment_time


# ============================================================================
# DELTA KINEMATICS CLASS (Wrapper cho Numba functions)
# ============================================================================

class DeltaKinematics:


    def __init__(self, e_side=None, f_side=None, re_lower=None, rf_upper=None, tool_offset_radius=15, tool_offset_z=53):
        """
        Khởi tạo với các thông số hình học của robot.
        Nếu không truyền tham số, sẽ lấy mặc định từ constants.py
        """
        # Lấy từ tham số hoặc constants
        self.e = e_side if e_side is not None else C.ROBOT_E_SIDE
        self.f = f_side if f_side is not None else C.ROBOT_F_SIDE
        self.re = re_lower if re_lower is not None else C.ROBOT_RE_LOWER
        self.rf = rf_upper if rf_upper is not None else C.ROBOT_RF_UPPER

        # Hằng số lượng giác
        self.sqrt3 = math.sqrt(3.0)
        self.pi = math.pi
        self.sin120 = self.sqrt3 / 2.0
        self.cos120 = -0.5
        self.tan30 = 1.0 / self.sqrt3
        
        # Hằng số 't' từ C++ forward kinematics
        self.t = (self.f - self.e) * self.tan30 / 2.0
        self.dtr = self.pi / 180.0 # Độ sang Radian

        # Tool offset (QUAN TRỌNG)
        self.tool_offset_radius = tool_offset_radius
        self.tool_offset_z = tool_offset_z

        # Hằng số động cơ bước - Lấy từ constants.py để dễ calib
        self.gear_ratio = C.MOTOR_GEAR_RATIO
        self.steps_per_revolution = C.MOTOR_STEPS_PER_REV

        # Độ phân giải cuối cùng: số xung động cơ để quay cánh tay robot 1 độ.
        self.steps_per_arm_degree = (self.steps_per_revolution * self.gear_ratio) / 360.0

    def warmup(self):
        """
        Kích hoạt JIT compilation (warm-up) cho các hàm Numba bằng cách gọi chúng với dữ liệu mẫu.
        Giúp tránh lag ở lần chạy đầu tiên.
        """
        # Dummy coordinates (gần vị trí Home)
        x, y, z = 0.0, 0.0, -380.0
        alpha = -90.0
        
        # Warmup inverse_kinematics_numba
        inverse_kinematics_numba(
            x, y, z,
            self.tan30, self.f, self.e, self.rf, self.re, self.pi, self.sqrt3,
            self.cos120, self.sin120, C.ARM_ANGLE_MIN, C.ARM_ANGLE_MAX
        )
        
        # Warmup inverse_kinematics_tool_numba
        inverse_kinematics_tool_numba(
            x, y, z, alpha,
            self.tool_offset_radius, self.tool_offset_z,
            self.tan30, self.f, self.e, self.rf, self.re, self.pi, self.sqrt3,
            self.cos120, self.sin120, C.ARM_ANGLE_MIN, C.ARM_ANGLE_MAX
        )

        # Warmup _calc_angle_yz_numba implicitly via above calls
        
        # Warmup new Cartesian Time Calculation (quan trọng cho Scheduler)
        self.inverse_kinematics_tool_to_steps(x, y, z, alpha)
        calc_trapezoidal_time_numba(
            100.0, 5000, 
            500.0, 2000.0, 
            20000.0, 0.020
        )
        
        print("✅ DeltaKinematics: Numba functions warmed up!")
 
    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Tính toán tọa độ TÂM BỆ DI ĐỘNG (x0, y0, z0) từ các góc động cơ.
        Logic dựa trên delta_calcForward.
        
        :return: (x0, y0, z0) hoặc None nếu không có nghiệm
        """
        theta1_rad = theta1 * self.dtr
        theta2_rad = theta2 * self.dtr
        theta3_rad = theta3 * self.dtr

        y1 = -(self.t + self.rf * math.cos(theta1_rad))
        z1 = -self.rf * math.sin(theta1_rad)

        y2 = (self.t + self.rf * math.cos(theta2_rad)) * 0.5  # 0.5 = sin30
        x2 = y2 * self.sqrt3  # tan60
        z2 = -self.rf * math.sin(theta2_rad)

        y3 = (self.t + self.rf * math.cos(theta3_rad)) * 0.5
        x3 = -y3 * self.sqrt3
        z3 = -self.rf * math.sin(theta3_rad)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2
        
        if abs(dnm) < 1e-9:
            return None # Cấu hình kỳ dị

        w1 = y1**2 + z1**2
        w2 = x2**2 + y2**2 + z2**2
        w3 = x3**2 + y3**2 + z3**2

        # x = (a1*z + b1)/dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        # y = (a2*z + b2)/dnm;
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # a*z^2 + b*z + c = 0
        a = a1**2 + a2**2 + dnm**2
        b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm)**2 + b1**2 + dnm**2 * (z1**2 - self.re**2)

        # discriminant
        d = b**2 - 4.0 * a * c
        if d < 0:
            return None  # non-existing point

        z0 = -0.5 * (b + math.sqrt(d)) / a
        x0 = (a1 * z0 + b1) / dnm
        y0 = (a2 * z0 + b2) / dnm
        
        # ✅ FIX: Đảo dấu X để phù hợp với hệ tọa độ robot
        # Update 28/11/2025: Xoay hệ tọa độ 180 độ (X -> -X, Y -> -Y)
        # User report: X+ bị ngược (ở bên trái) -> Đổi lại thành x0
        return (x0, -y0, z0)

    def forward_kinematics_tool(self, theta1, theta2, theta3, alpha_deg):
        """
        Tính toán tọa độ ĐẦU HÚT (xt, yt, zt) từ các góc động cơ
        VÀ góc servo. (HÀM MỚI)
        
        :param theta1, theta2, theta3: Góc 3 động cơ (độ)
        :param alpha_deg: Góc của servo (độ) dùng cho tool offset
        :return: (xt, yt, zt) của đầu hút, hoặc None nếu không có nghiệm
        """
        
        # 1. Tính tọa độ TÂM BỆ DI ĐỘNG (effector)
        effector_coords = self.forward_kinematics(theta1, theta2, theta3)
        
        if effector_coords is None:
            # Không thể tính được vị trí tâm bệ
            return None
            
        x0, y0, z0 = effector_coords
        
        # 2. Từ tâm bệ, tính ra vị trí đầu hút (xt, yt, zt)
        alpha_rad = math.radians(alpha_deg)
        
        # Tool offset theo alpha (góc quay của servo)
        # Quy ước servo: 0° → X+, 90° → Y+, 180° → X-, 270° → Y-
        # Update 02/12: Fix dấu Y - phải CỘNG cả X và Y
        xt = x0 + self.tool_offset_radius * math.cos(alpha_rad)
        yt = y0 + self.tool_offset_radius * math.sin(alpha_rad) 
        
        # zt = z_effector - offset_z (vì tool thấp hơn)
        zt = z0 - self.tool_offset_z
        
        return (xt, yt, zt)

    def _calc_angle_yz(self, x0, y0, z0):
        """
        Hàm trợ giúp tính góc theta cho 1 cánh tay.
        Tính cho TÂM BỆ (x0, y0, z0).
        """
        # Hằng số f/2 * tan30
        y1 = -0.5 * self.tan30 * self.f
        
        # Trừ offset effector
        y0_mod = y0 - 0.5 * self.tan30 * self.e
        
        if abs(z0) < 1e-9:
             return None # Tránh chia cho 0

        # z = a + b*y
        a = (x0**2 + y0_mod**2 + z0**2 + self.rf**2 - self.re**2 - y1**2) / (2.0 * z0)
        b = (y1 - y0_mod) / z0
        
        # discriminant
        d = -(a+b*y1)**2 + self.rf*(b*b*self.rf + self.rf)
        
        # Nếu d < 0, điểm không tồn tại. 
        # Nếu d = 0, điểm nằm trên biên của không gian làm việc (điểm kỳ dị).
        # Sử dụng một sai số nhỏ để tránh các vấn đề về số thực và coi các điểm rất gần kỳ dị là không thể đạt tới.
        if d < 1e-9:
            return None  # Điểm không tồn tại hoặc là điểm kỳ dị
        
        # choosing outer point
        yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1.0)
        zj = a + b * yj
        
        # Ngăn chia cho 0 nếu yj == y1
        if abs(y1 - yj) < 1e-9:
            if zj < 0:
                theta_deg = -90.0
            else:
                theta_deg = 90.0
        else:
            theta_rad = math.atan(-zj / (y1 - yj))
            theta_deg = 180.0 * theta_rad / self.pi

        # Xử lý góc phần tư
        if yj > y1:
            theta_deg += 180.0
            
        return theta_deg

    def inverse_kinematics(self, x0, y0, z0):
        """
        Tính toán các góc động cơ (theta1, 2, 3) cho một tọa độ TÂM BỆ (x0, y0, z0).
        ⚡ OPTIMIZED: Sử dụng Numba JIT compilation (20-100x nhanh hơn).
        
        :return: (theta1, theta2, theta3) trong độ, hoặc None nếu không thể đạt
        """
        # Gọi Numba-optimized function
        theta1, theta2, theta3 = inverse_kinematics_numba(
            x0, y0, z0,
            self.tan30, self.f, self.e, self.rf, self.re, self.pi, self.sqrt3,
            self.cos120, self.sin120, C.ARM_ANGLE_MIN, C.ARM_ANGLE_MAX
        )
        
        # Kiểm tra thất bại (Numba trả về nan)
        if np.isnan(theta1):
            return None
            
        return (theta1, theta2, theta3)

    def inverse_kinematics_tool(self, xt, yt, zt, alpha_deg):
        """
        Tính động học nghịch cho ĐẦU HÚT (xt, yt, zt) có offset servo.
        ⚡ OPTIMIZED: Sử dụng Numba JIT compilation (20-100x nhanh hơn).
        """
        # Gọi Numba-optimized function trực tiếp
        theta1, theta2, theta3 = inverse_kinematics_tool_numba(
            xt, yt, zt, alpha_deg,
            self.tool_offset_radius, self.tool_offset_z,
            self.tan30, self.f, self.e, self.rf, self.re, self.pi, self.sqrt3,
            self.cos120, self.sin120, C.ARM_ANGLE_MIN, C.ARM_ANGLE_MAX
        )
        
        # Kiểm tra thất bại (Numba trả về nan)
        if np.isnan(theta1):
            return None
            
        return (theta1, theta2, theta3)

    # --- CÁC HÀM TIỆN ÍCH CHUYỂN ĐỔI SANG XUNG ---

    def angles_to_steps(self, theta1, theta2, theta3):
        """
        Chuyển đổi góc khớp (độ) thành số xung động cơ.
        """
        steps1 = theta1 * self.steps_per_arm_degree
        steps2 = theta2 * self.steps_per_arm_degree
        steps3 = theta3 * self.steps_per_arm_degree

        return (round(steps1), round(steps2), round(steps3))

    def steps_to_angles(self, steps1, steps2, steps3):
        """
        Chuyển đổi số xung động cơ thành góc khớp (độ).
        Đây là hàm ngược lại của angles_to_steps.
        """
        theta1 = steps1 / self.steps_per_arm_degree
        theta2 = steps2 / self.steps_per_arm_degree
        theta3 = steps3 / self.steps_per_arm_degree

        return (theta1, theta2, theta3)

    def inverse_kinematics_to_steps(self, x0, y0, z0):
        """
        Tính số xung động cơ trực tiếp từ tọa độ TÂM BỆ (effector).
        (Hàm này chỉ dùng để test, controller nên dùng _tool_)
        """
        angles = self.inverse_kinematics(x0, y0, z0)
        if angles is None:
            return None
        return self.angles_to_steps(*angles)

    def inverse_kinematics_tool_to_steps(self, xt, yt, zt, alpha_deg):
        """
        Tính số xung động cơ từ tọa độ ĐẦU HÚT.
        Đây là hàm mà robot_controller nên sử dụng.
        """
        angles = self.inverse_kinematics_tool(xt, yt, zt, alpha_deg)
        if angles is None:
            return None
        return self.angles_to_steps(*angles)


# ============================================================================
# MOTION PLANNER CLASSES
# ============================================================================

class MotionPlanner:
    """
    Motion Planner đơn giản: Constant Velocity (Rectangular Profile).
    Phù hợp cho JOG mode (điều khiển thủ công) vì phản hồi nhanh.
    """
    def __init__(self, kinematics, speed_rpm=100.0):
        self.steps_per_arm_degree = kinematics.steps_per_arm_degree
        self.steps_per_revolution = kinematics.steps_per_revolution
        self.speed_hz = (speed_rpm * self.steps_per_revolution) / 60.0

    def _calculate_move_time(self, delta_steps):
        max_steps = max(abs(s) for s in delta_steps)
        if max_steps == 0: return 0.0
        return max_steps / self.speed_hz

    def plan_move(self, current_angles, target_angles):
        return self.plan_move_with_options(current_angles, target_angles, pump_state=0)
    
    def plan_move_with_options(self, current_angles, target_angles, pump_state=0):
        current_steps = [angle * self.steps_per_arm_degree for angle in current_angles]
        target_steps = [angle * self.steps_per_arm_degree for angle in target_angles]
        delta_steps = [int(target - current) for target, current in zip(target_steps, current_steps)]

        if all(s == 0 for s in delta_steps): return []

        move_time = self._calculate_move_time(delta_steps)
        plan = [{"t": move_time, "s": delta_steps}]
        return plan
    
    def get_move_duration(self, current_angles, target_angles):
        current_steps = [angle * self.steps_per_arm_degree for angle in current_angles]
        target_steps = [angle * self.steps_per_arm_degree for angle in target_angles]
        delta_steps = [int(target - current) for target, current in zip(target_steps, current_steps)]
        return self._calculate_move_time(delta_steps)


class MotionPlannerTrapezoidal:
    """
    Motion Planner với biên dạng hình thang (Trapezoidal Profile).
    Hỗ trợ Continuous Motion (Look-ahead & Cornering).
    """
    def __init__(self, kinematics):
        self.steps_per_arm_degree = kinematics.steps_per_arm_degree
        self.kinematics = kinematics
        
        self.V_MAX_RPM = C.TRAJ_V_MAX_RPM
        self.steps_per_revolution = kinematics.steps_per_revolution
        self.v_max_hz_limit = (self.V_MAX_RPM * self.steps_per_revolution) / 60.0
        
        self.v_linear_max = C.TRAJ_LINEAR_V_MAX_MM_S
        self.accel_linear = C.TRAJ_LINEAR_ACCEL_MM_S2
        self.v_linear_start = C.TRAJ_LINEAR_V_START_MM_S
        self.SEGMENT_TIME = C.TRAJ_SEGMENT_TIME
        self.CARTESIAN_MM_TO_STEPS_AVG = 15.0
        self.JUNCTION_DEVIATION = C.TRAJ_JUNCTION_DEVIATION

    @staticmethod
    def _clean_float(value, tolerance=0.001):
        return 0.0 if abs(value) < tolerance else value
    
    def _clean_point(self, point):
        return tuple(round(v, 2) for v in point)

    def _is_point_reachable(self, point, alpha_deg=None):
        x, y, z = point
        if alpha_deg is None: alpha_deg = C.HOME_DEFAULT_SERVO_ANGLE_DEG
        
        TOLERANCE = 5.0
        if hasattr(C, 'WORKSPACE_X_MAX'):
            if abs(x) > C.WORKSPACE_X_MAX + TOLERANCE or abs(y) > C.WORKSPACE_Y_MAX + TOLERANCE: return False
            if z < C.WORKSPACE_Z_MIN - TOLERANCE or z > C.WORKSPACE_Z_MAX + TOLERANCE: return False
        
        angles = self.kinematics.inverse_kinematics_tool(x, y, z, alpha_deg)
        if angles is None: angles = self.kinematics.inverse_kinematics_tool(x, y, z - 1.0, alpha_deg)
        if angles is None: angles = self.kinematics.inverse_kinematics_tool(x, y, z + 1.0, alpha_deg)

        if angles is None: return False
            
        ANGLE_TOLERANCE = 5.0
        for angle in angles:
            if angle < C.ARM_ANGLE_MIN - ANGLE_TOLERANCE or angle > C.ARM_ANGLE_MAX + ANGLE_TOLERANCE: return False
        return True
    
    def _estimate_cartesian_to_steps(self, p1, p2, alpha_deg):
        angles1 = self.kinematics.inverse_kinematics_tool(p1[0], p1[1], p1[2], alpha_deg)
        angles2 = self.kinematics.inverse_kinematics_tool(p2[0], p2[1], p2[2], alpha_deg)
        
        if angles1 is None or angles2 is None: return None
            
        delta_steps = [abs(angles2[i] - angles1[i]) * self.steps_per_arm_degree for i in range(3)]
        return max(delta_steps)

    def _compute_profile_parameters(self, distance, v_start, v_end, v_max, accel):
        distance = abs(distance)
        if distance == 0:
            return {"total_time": 0.0, "v_peak": v_start, "t_accel": 0, "t_const": 0, "t_decel": 0,
                    "s_accel": 0, "s_const": 0, "s_decel": 0, "v_start": v_start, "v_end": v_end}

        min_dist_to_change_speed = abs(v_end**2 - v_start**2) / (2 * accel)
        
        if distance < min_dist_to_change_speed:
            if v_end > v_start: v_end = math.sqrt(v_start**2 + 2 * accel * distance)

        v_peak_sq = (2 * accel * distance + v_start**2 + v_end**2) / 2.0
        if v_peak_sq < 0: v_peak_sq = 0
        v_peak_theoretical = math.sqrt(v_peak_sq)
        v_peak = min(v_max, v_peak_theoretical)
        
        t_accel = abs(v_peak - v_start) / accel if accel > 0 else 0
        t_decel = abs(v_peak - v_end) / accel if accel > 0 else 0
        s_accel = (v_peak + v_start) * t_accel / 2.0
        s_decel = (v_peak + v_end) * t_decel / 2.0
        s_const = distance - s_accel - s_decel
        
        if s_const < 0: s_const = 0
        
        t_const = s_const / v_peak if v_peak > 0 else 0
        total_time = t_accel + t_const + t_decel
        
        return {"total_time": total_time, "v_peak": v_peak, "v_start": v_start, "v_end": v_end,
                "t_accel": t_accel, "t_const": t_const, "t_decel": t_decel,
                "s_accel": s_accel, "s_const": s_const, "s_decel": s_decel, "accel": accel}

    def _compute_general_profile(self, distance_steps, v_start, v_end):
        accel_steps = self.v_max_hz_limit / 0.2
        return self._compute_profile_parameters(distance_steps, v_start, v_end, self.v_max_hz_limit, accel_steps)

    def _calculate_position_at_t(self, t, p):
        if t <= 0: return 0.0
        if t >= p["total_time"]: return p["s_accel"] + p["s_const"] + p["s_decel"]
        
        accel = p["accel"]
        if t <= p["t_accel"]:
            sign = 1 if p["v_peak"] >= p["v_start"] else -1
            return p["v_start"] * t + 0.5 * sign * accel * (t**2)
        elif t <= (p["t_accel"] + p["t_const"]):
            dt = t - p["t_accel"]
            return p["s_accel"] + p["v_peak"] * dt
        else:
            dt = t - p["t_accel"] - p["t_const"]
            return p["s_accel"] + p["s_const"] + (p["v_peak"] * dt - 0.5 * accel * (dt**2))

    def calc_travel_time(self, start_pos, end_pos, alpha_deg):
        """
        Tính thời gian di chuyển CHÍNH XÁC từ A đến B (Cartesian).
        Sử dụng Numba để tính IK và mô phỏng Trapezoidal Profile + Motor Limit.
        
        Args:
            start_pos: (x, y, z)
            end_pos: (x, y, z)
            alpha_deg: Góc servo (Coordinate System)
            
        Returns:
            float: Thời gian di chuyển (giây) hoặc None nếu lỗi IK
        """
        # 1. Tính khoảng cách Cartesian
        dist_mm = math.sqrt((end_pos[0]-start_pos[0])**2 + 
                            (end_pos[1]-start_pos[1])**2 + 
                            (end_pos[2]-start_pos[2])**2)
        
        # 2. Tính số bước động cơ cần thiết (Max Steps)
        # Dùng IK Numba để tốc độ cực nhanh
        steps_start = self.kinematics.inverse_kinematics_tool_to_steps(
            start_pos[0], start_pos[1], start_pos[2], alpha_deg)
        
        steps_end = self.kinematics.inverse_kinematics_tool_to_steps(
            end_pos[0], end_pos[1], end_pos[2], alpha_deg)
            
        if steps_start is None or steps_end is None:
            return None # Không với tới
            
        max_steps = 0
        for i in range(3):
            diff = abs(steps_end[i] - steps_start[i])
            if diff > max_steps: max_steps = diff
            
        # 3. Gọi hàm Numba tính thời gian
        return calc_trapezoidal_time_numba(
            dist_mm, max_steps,
            self.v_linear_max, self.accel_linear,
            self.v_max_hz_limit, self.SEGMENT_TIME
        )

    def plan_cartesian_trajectory(self, points, alpha_deg, current_angles):
        if len(points) < 2: return []
        full_plan = []
        velocities = [0.0] * len(points)
        
        for i in range(1, len(points) - 1):
            p_prev, p_curr, p_next = points[i-1], points[i], points[i+1]
            v1 = (p_curr[0]-p_prev[0], p_curr[1]-p_prev[1], p_curr[2]-p_prev[2])
            v2 = (p_next[0]-p_curr[0], p_next[1]-p_curr[1], p_next[2]-p_curr[2])
            len1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
            len2 = math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)
            
            if len1 == 0 or len2 == 0:
                velocities[i] = 0
                continue
                
            dot_product = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
            cos_theta = max(-1.0, min(1.0, dot_product / (len1 * len2)))
            sin_half_theta = math.sqrt((1.0 - cos_theta) / 2.0)
            
            v_corner = self.v_linear_max
            if sin_half_theta > 0.001:
                v_corner = math.sqrt(self.accel_linear * self.JUNCTION_DEVIATION / sin_half_theta)
            v_corner = min(v_corner, self.v_linear_max)
            max_reachable = math.sqrt(2 * self.accel_linear * len1)
            velocities[i] = min(v_corner, max_reachable)

        for i in range(len(points) - 2, 0, -1):
            dist_mm = math.sqrt(sum((points[i+1][k]-points[i][k])**2 for k in range(3)))
            max_entry = math.sqrt(velocities[i+1]**2 + 2 * self.accel_linear * dist_mm)
            if velocities[i] > max_entry: velocities[i] = max_entry

        simulated_angles = list(current_angles)
        for i in range(len(points) - 1):
            p_start, p_end = points[i], points[i+1]
            v_s, v_e = velocities[i], velocities[i+1]
            
            segment_plan = self.plan_cartesian_move_time_sliced(
                p_start, p_end, alpha_deg, self.kinematics, simulated_angles, v_start_mm_s=v_s, v_end_mm_s=v_e
            )
            if segment_plan:
                full_plan.extend(segment_plan)
                for block in segment_plan:
                    simulated_angles = [simulated_angles[j] + (block["s"][j] / self.steps_per_arm_degree) for j in range(3)]
        return full_plan

    def plan_cartesian_move_time_sliced(self, start_cartesian, end_cartesian, 
                                        alpha_deg, kinematics, current_angles,
                                        v_start_mm_s=0.0, v_end_mm_s=0.0):
        start_cartesian = self._clean_point(start_cartesian)
        end_cartesian = self._clean_point(end_cartesian)
        x0, y0, z0 = start_cartesian
        xt, yt, zt = end_cartesian
        
        if not self._is_point_reachable(start_cartesian, alpha_deg): raise ValueError(f"Start {start_cartesian} outside workspace")
        if not self._is_point_reachable(end_cartesian, alpha_deg): raise ValueError(f"End {end_cartesian} outside workspace")
        
        total_distance_mm = math.sqrt((xt - x0)**2 + (yt - y0)**2 + (zt - z0)**2)
        if total_distance_mm < 0.1: return []
        
        num_checks = 5
        for check_i in range(1, num_checks):
            ratio = check_i / num_checks
            x_check = x0 + (xt - x0) * ratio
            y_check = y0 + (yt - y0) * ratio
            z_check = z0 + (zt - z0) * ratio
            if not self._is_point_reachable((x_check, y_check, z_check), alpha_deg):
                raise ValueError(f"Path crosses workspace boundary at {ratio*100:.1f}%")
        
        end_angles = kinematics.inverse_kinematics_tool(xt, yt, zt, alpha_deg)
        if end_angles is None: raise ValueError(f"End point {end_cartesian} IK failed")
        
        params_mm = self._compute_profile_parameters(total_distance_mm, v_start_mm_s, v_end_mm_s, self.v_linear_max, self.accel_linear)
        total_time = params_mm["total_time"]
        
        delta_steps_list = [abs(end_angles[j] - current_angles[j]) * self.steps_per_arm_degree for j in range(3)]
        max_delta_steps = max(delta_steps_list)
        
        if total_time > 0:
            required_motor_hz = max_delta_steps / total_time
            if required_motor_hz > self.v_max_hz_limit:
                min_time_motor = max_delta_steps / self.v_max_hz_limit
                scale_factor = min_time_motor / total_time
                total_time = min_time_motor
                params_mm["total_time"] = total_time
                params_mm["t_accel"] *= scale_factor
                params_mm["t_const"] *= scale_factor
                params_mm["t_decel"] *= scale_factor
        
        min_time = 2 * self.SEGMENT_TIME
        if total_time < min_time:
            scale = min_time / total_time if total_time > 0 else 1
            total_time = min_time
            params_mm["total_time"] = total_time
            params_mm["t_accel"] *= scale
            params_mm["t_const"] *= scale
            params_mm["t_decel"] *= scale
        
        num_slices = max(1, int(math.ceil(total_time / self.SEGMENT_TIME)))
        dt = total_time / num_slices
        plan = []
        start_steps_absolute = [current_angles[j] * self.steps_per_arm_degree for j in range(3)]
        final_target_steps = [int(round(end_angles[j] * self.steps_per_arm_degree)) for j in range(3)]
        total_delta_required = [final_target_steps[j] - int(round(start_steps_absolute[j])) for j in range(3)]
        accumulated_steps_sent = [0, 0, 0]
        fractional_accumulator = [0.0, 0.0, 0.0]
        
        for i in range(1, num_slices + 1):
            t_current = i * dt
            is_last_slice = (i == num_slices)
            
            if is_last_slice:
                delta_steps = [total_delta_required[j] - accumulated_steps_sent[j] for j in range(3)]
            else:
                distance_traveled_mm = self._calculate_position_at_t(t_current, params_mm)
                ratio = distance_traveled_mm / total_distance_mm if total_distance_mm > 0 else 0
                ratio = min(1.0, max(0.0, ratio))
                
                x_current = x0 + (xt - x0) * ratio
                y_current = y0 + (yt - y0) * ratio
                z_current = z0 + (zt - z0) * ratio
                
                target_angles_slice = kinematics.inverse_kinematics_tool(x_current, y_current, z_current, alpha_deg)
                if target_angles_slice is None: raise ValueError(f"IK failed at slice {i}/{num_slices}")
                
                target_steps_float = [target_angles_slice[j] * self.steps_per_arm_degree for j in range(3)]
                delta_steps = []
                for j in range(3):
                    ideal_steps_from_start = target_steps_float[j] - start_steps_absolute[j]
                    ideal_with_fraction = ideal_steps_from_start + fractional_accumulator[j]
                    target_int = int(round(ideal_with_fraction))
                    d = target_int - accumulated_steps_sent[j]
                    delta_steps.append(d)
                    fractional_accumulator[j] = ideal_with_fraction - target_int
            
            accumulated_steps_sent = [accumulated_steps_sent[j] + delta_steps[j] for j in range(3)]
            block_time = dt if not is_last_slice else (total_time - (num_slices - 1) * dt)
            has_movement = any(d != 0 for d in delta_steps)
            if block_time > 0.001 and (has_movement or is_last_slice):
                plan.append({"t": block_time, "s": delta_steps})
        
        start_steps_int = [int(round(start_steps_absolute[j])) for j in range(3)]
        final_steps_actual = [start_steps_int[j] + total_delta_required[j] for j in range(3)]
        final_angles_actual = [final_steps_actual[j] / self.steps_per_arm_degree for j in range(3)]
        
        if plan:
            plan[-1]["_final_angles_actual"] = final_angles_actual
            plan[-1]["_final_steps_actual"] = final_steps_actual
            
        return plan

    def plan_move(self, current_angles, target_angles):
        return self.plan_move_with_options(current_angles, target_angles, pump_state=0)

    def plan_move_with_options(self, current_angles, target_angles, pump_state=0):
        current_steps = [angle * self.steps_per_arm_degree for angle in current_angles]
        target_steps = [angle * self.steps_per_arm_degree for angle in target_angles]
        
        delta_steps = [int(target - current) for target, current in zip(target_steps, current_steps)]
        if all(s == 0 for s in delta_steps): return []
            
        max_distance = max(abs(s) for s in delta_steps)
        params = self._compute_general_profile(max_distance, 0, 0)
        total_time = params["total_time"]
        
        num_segments = max(1, int(math.ceil(total_time / self.SEGMENT_TIME)))
        segment_time = total_time / num_segments
        
        plan = []
        for i in range(num_segments):
            t_current = (i + 1) * segment_time
            s_current = self._calculate_position_at_t(t_current, params)
            s_prev = self._calculate_position_at_t(t_current - segment_time, params) if i > 0 else 0
            
            ratio = (s_current - s_prev) / max_distance if max_distance > 0 else 0
            segment_steps = [int(round(delta * ratio)) for delta in delta_steps]
            
            if any(s != 0 for s in segment_steps) or segment_time > 0:
                 plan.append({"t": segment_time, "s": segment_steps})
        
        if plan:
            total_sent = [sum(block["s"][j] for block in plan) for j in range(3)]
            remaining = [delta_steps[j] - total_sent[j] for j in range(3)]
            if any(s != 0 for s in remaining):
                plan[-1]["s"] = [plan[-1]["s"][j] + remaining[j] for j in range(3)]
        return plan
    
    def get_move_duration(self, current_angles, target_angles):
        current_steps = [angle * self.steps_per_arm_degree for angle in current_angles]
        target_steps = [angle * self.steps_per_arm_degree for angle in target_angles]
        delta_steps = [int(target - current) for target, current in zip(target_steps, current_steps)]
        max_distance = max(abs(s) for s in delta_steps)
        
        params = self._compute_general_profile(max_distance, 0, 0)
        total_time = params["total_time"]
        
        min_time = 2 * self.SEGMENT_TIME
        if total_time < min_time: total_time = min_time
        return total_time