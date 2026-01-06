import constants as C
import time
import math
import threading
import json
import queue
import math
from kinematics import DeltaKinematics, MotionPlanner, MotionPlannerTrapezoidal
from enum import Enum, auto
from typing import Optional, Tuple
from auto_mode_controller import AutoModeController, AutoState


from PyQt6.QtCore import QObject, pyqtSignal

class RobotController(QObject):
    # Thêm 2 Signal mới
    hardware_start_pressed = pyqtSignal()
    hardware_stop_pressed = pyqtSignal()

    def __init__(self, app, conn_manager):
        """
        Khởi tạo Robot Controller.
        
        Args:
            app: Instance của GUI application
            conn_manager: Connection manager để giao tiếp với STM32
        """
        QObject.__init__(self)
        print("✅ RobotController V2.1 LOADED - Fixed ACK Handling")
        self.app = app
        self.conn_manager = conn_manager
        # Lưu trạng thái cũ của nút cứng để phát hiện cạnh lên
        self._last_btn_start_state_signal = 0
        self._last_btn_stop_state_signal = 0
        
        # ✅ LOAD PARAMETERS TỪ CONSTANTS.PY
        # Để thay đổi kích thước robot, hãy sửa file constants.py
        self.kinematics = DeltaKinematics()
        self.kinematics.warmup() # ✅ Warmup Numba JIT functions to prevent lag
        
        # ✅ Planner riêng cho lệnh MOVE - LUÔN dùng Trapezoidal
        self.planner_trapezoidal = MotionPlannerTrapezoidal(self.kinematics)
        self.app.log_message("🎯 MOVE luôn sử dụng Trapezoidal Motion Planner", "info")

        # ✅ State machine đơn giản
        self.state = C.RobotState.IDLE
        self.state_lock = threading.RLock()
        
        # self.is_running_auto = False # Delegated
        self.command_id_counter = 0
        self.active_command_ids = set()
        
        # ✅ NEW: Track pending moves - lưu target position của mỗi lệnh
        # Format: {block_id: {"target_coords": (x,y,z), "target_theta": [t1,t2,t3], "servo": angle, "pump": state}}
        self.pending_moves = {}
        self.pending_moves_lock = threading.Lock()
        
        # ✅ FRIENDLY LOG: Track active operations để log thân thiện
        # Đổi thành dict để support multiple concurrent moves
        self.active_move_info = {}  # {block_id: {"target": (x,y,z), "total_blocks": N}}
        self.active_jog_info = None   # {"axis": "X", "direction": "+"}
        self.operation_lock = threading.Lock()
        
        # ✅ FIX BUG 2: Flow Control - Theo dõi slots trong queue STM32
        self.MAX_QUEUE_SIZE = 64  # Phải khớp với QUEUE_SIZE trong command_queue.c
        self.available_slots = self.MAX_QUEUE_SIZE  # Số slots còn trống
        self.slots_lock = threading.RLock()
        
        # ✅ NEW: Sliding Window Flow Control cho trajectories dài (>64 blocks)
        # Thay vì gửi tất cả blocks một lúc (gây tràn queue), ta:
        # - Gửi batch đầu tiên (20 blocks - tăng từ 16)
        # - Khi nhận DONE, refill thêm blocks để queue không bị trống (starvation)
        # - CHỈ refill khi blocks_in_flight < LOW_THRESHOLD (tránh tràn)
        self.pending_blocks_queue = []  # List of {"params": {...}, "target_position": {...}}
        self.blocks_in_flight = 0  # Số blocks đã gửi nhưng chưa nhận DONE
        self.pending_blocks_lock = threading.RLock()
        # ✅ FIX STARVATION: Refill nhanh hơn để tránh queue STM32 cạn kiệt (robot giật)
        # LOW: Refill khi blocks_in_flight < 6 (giảm từ 8 để sớm hơn)
        # Batch size: Gửi 12 blocks mỗi lần (tăng từ 8 để nhanh hơn)
        self.BUFFER_LOW_THRESHOLD = 6    # Refill khi queue < 6 blocks (giảm từ 8)
        self.REFILL_BATCH_SIZE = 12      # Gửi 12 blocks mỗi lần (tăng từ 8)
        self.INITIAL_BATCH_SIZE = 20     # Lần đầu gửi 20 blocks (tăng từ 16)
        self.current_trajectory_block_index = 0  # Block index cho trajectory hiện tại (0, 1, 2, ...)
        
        # ✅ NEW: Block Sequence Validation - Phát hiện STM32 nhận thiếu hoặc làm thiếu block
        self.sent_block_ids = {}  # {block_id: {"time": timestamp, "source": "AUTO/MANUAL/JOG"}} - Tracking blocks đã gửi
        self.received_done_ids = set()  # Set of block_ids đã nhận DONE
        self.sequence_lock = threading.RLock()
        self.BLOCK_TIMEOUT_SEC = 15.0  # Timeout để phát hiện block bị mất (15s - giảm từ 30s)
        self.last_timeout_check = time.time()  # Tránh spam timeout check
        
        self.virtual_coords = None
        self.coord_lock = threading.Lock()

        self.homing_state = "NOT_HOMED"
        
        # Theo dõi góc theta hiện tại (sau Home)
        self.current_theta = [C.HOME_DEFAULT_THETA_DEG, C.HOME_DEFAULT_THETA_DEG, C.HOME_DEFAULT_THETA_DEG]
        
        # Debounce cho nút cứng
        self.last_button_press_time = 0
        
        # ✅ Đơn giản hóa JOG tracking
        self.jog_stop_requested = False
        self.jog_at_limit = False  # ✅ NEW: Cờ đánh dấu đã đạt giới hạn workspace
        self.jog_limit_logged = False # ✅ NEW: Cờ chặn spam log khi đạt giới hạn
        self.current_jog_axis = None
        self.current_jog_direction = None
        
        self.current_servo_angle = C.HOME_DEFAULT_SERVO_ANGLE_DEG  # -90.0 (Coordinate)
        self.current_pump_state = 0  # ✅ Khởi tạo: 0 = TẮT
        self.last_pump_command_time = 0  # ✅ Debounce cho pump
        self.PUMP_DEBOUNCE_MS = 300  # 300ms debounce
        
        # ✅ Đơn giản hóa: không cần pending_move_updates và timeout_handlers nữa
        
        self.waiting_for_flush_ack_before_homing = False
        self.conn_manager.add_message_listener(self.handle_stm_message)
        
        # Buffer control for JOG
        self.jog_pending_count = 0
        self.MAX_JOG_PENDING = 5  # Số lượng lệnh JOG tối đa trong buffer
        self.jog_target_tracker = None # ✅ NEW: Theo dõi vị trí đích dự kiến của JOG để queue chính xác
        
        # ✅ NEW: Thời gian gửi lệnh di chuyển cuối cùng (để tránh Sync IDLE sai lúc khởi động)
        self.last_move_command_time = 0
        
        # --- AUTO MODE V4: DỰ ĐOÁN VỊ TRÍ BẰNG MÔ PHỎNG MOTION PLANNER ---
        # self.auto_state = AutoState.IDLE # Delegated to AutoModeController
        
        # ✅ CÁC THÔNG SỐ CỐ ĐỊNH
        self.wait_position = (0, -15.0, -390.0)    # Vị trí chờ khi RUN
        self.z_safe = -390.0                          # Z an toàn (nhấc lên)
        self.z_pick = -414.0                          # Z hạ xuống hút
        self.drop_position_default = (60.0, -15.0, -390.0)    # Vị trí thả mặc định (fallback khi không nhận diện được class)
        self.servo_angle_fixed = -90.0                # Góc servo cố định (như Home - Coordinate)
        
        # ✅ CALIBRATION CAMERA -> ROBOT  
        # Vật đi từ Y=-300 về Y=-15
        # TRIGGER: Đường vuông góc tại Y = -240mm
        self.trigger_robot_y = -240.0                 # Đường trigger tại Y=-240mm
        self.conveyor_speed_mm_s = 40.0               # Tốc độ băng tải (mm/s)
        self.conveyor_direction = 1                   # +1 = Y tăng (vật đi từ Y=-300 -> Y=-15)
        
        # ✅ AUTO MODE CONTROLLER
        self.auto_controller = AutoModeController(self)
        self.auto_objects_queue = self.auto_controller.auto_objects_queue
        # self.app.log_message("✅ Auto-logic thread started", "info")
        
        # ✅ Trạng thái cảm biến khay hứng (Mặc định False = Chưa có khay)
        self.is_tray_present = False
        
        # ✅ NEW: Bộ đệm lưu trạng thái cuối cùng của Robot để tránh cập nhật GUI thừa
        self._last_status_cache = {
            "is_homed": None,
            "is_estop": None,
            "btn_start": None,
            "btn_stop": None,
            "conv_status": None,
            "conv_speed": None,
            "pump_status": None,
            "servo_angle": None,
            "is_tray": None,
            "coords": (None, None, None),
            "theta": (None, None, None)
        }

    @property
    def auto_state(self):
        return self.auto_controller.auto_state

    @auto_state.setter
    def auto_state(self, value):
        self.auto_controller.auto_state = value

    @property
    def is_running_auto(self):
        return self.auto_controller.is_running_auto

    @is_running_auto.setter
    def is_running_auto(self, value):
        self.auto_controller.is_running_auto = value

    def compute_trigger_line_pixel(self, camera_config):
        """Tính tọa độ pixel của đường trigger Y=-220mm trong ROI.
        Tự động phát hiện trục băng tải (X hay Y trên ảnh) để lấy tọa độ đúng.
        """
        if camera_config and camera_config.is_calibrated() and camera_config.roi:
            # 1. Tính điểm pixel tại trigger Y và một điểm lân cận để xác định trục
            trigger_point = camera_config.mm_to_pixel(0.0, self.trigger_robot_y)
            test_point = camera_config.mm_to_pixel(0.0, self.trigger_robot_y - 50.0) # Điểm cách 50mm
            
            if trigger_point and test_point:
                # 2. So sánh delta để biết trục nào thay đổi nhiều hơn
                dx = abs(trigger_point[0] - test_point[0])
                dy = abs(trigger_point[1] - test_point[1])
                
                _, _, roi_w, roi_h = camera_config.roi
                
                if dx > dy:
                    # Băng tải chạy dọc theo trục X của ảnh (Width)
                    # Đường trigger vuông góc với băng tải => Đường DỌC (Vertical Line)
                    # Tọa độ quan trọng là X
                    self.trigger_axis = 'X'
                    trigger_val_px = int(trigger_point[0])
                    
                    # Lưu 2 điểm đầu/cuối của đường trigger (Vertical trong ảnh gốc)
                    # Từ (x, 0) đến (x, h)
                    self.trigger_line_p1 = (trigger_val_px, 0)
                    self.trigger_line_p2 = (trigger_val_px, roi_h)
                    
                    self.app.log_message(f"✅ Trigger (Vertical): Robot Y={self.trigger_robot_y} -> Img X={trigger_val_px}", "info")
                else:
                    # Băng tải chạy dọc theo trục Y của ảnh (Height) - Cũ
                    # Đường trigger vuông góc => Đường NGANG (Horizontal Line)
                    # Tọa độ quan trọng là Y
                    self.trigger_axis = 'Y'
                    trigger_val_px = int(trigger_point[1])
                    
                    # Lưu 2 điểm đầu/cuối của đường trigger (Horizontal trong ảnh gốc)
                    # Từ (0, y) đến (w, y)
                    self.trigger_line_p1 = (0, trigger_val_px)
                    self.trigger_line_p2 = (roi_w, trigger_val_px)
                    
                    self.app.log_message(f"✅ Trigger (Horizontal): Robot Y={self.trigger_robot_y} -> Img Y={trigger_val_px}", "info")
            else:
                self.trigger_line_p1 = None
                self.trigger_line_p2 = None
                self.trigger_axis = None
                self.app.log_message("⚠️ Không thể tính pixel cho trigger line", "warning")
        else:
            self.trigger_line_p1 = None
            self.trigger_line_p2 = None
            self.trigger_axis = None
    
    def _update_virtual_coords(self, x, y, z, theta=None, update_gui=True):
        """Cập nhật tọa độ và tùy chọn hiển thị lên GUI"""
        with self.coord_lock:
            if self.virtual_coords is None:
                self.virtual_coords = {}
            self.virtual_coords['x'] = x
            self.virtual_coords['y'] = y
            self.virtual_coords['z'] = z
        
        if update_gui:
            # Calculate Theta if not provided
            if theta is None:
                alpha = self._get_kinematics_alpha(self.current_servo_angle)
                thetas = self.kinematics.inverse_kinematics_tool(x, y, z, alpha)
            else:
                thetas = theta

            # ✅ PERFORMANCE: Batch update tất cả 3 tọa độ
            # Gọi trực tiếp vì các hàm này emit signal (Thread-safe)
            # Loại bỏ app.after để tránh lỗi khi gọi từ luồng Serial (không có Event Loop)
            # print(f"[RC] Updating GUI: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Theta={thetas}")  # DEBUG
            self.app.set_coordinate('X', f"{x:.2f}")
            self.app.set_coordinate('Y', f"{y:.2f}")
            self.app.set_coordinate('Z', f"{z:.2f}")
            if thetas:
                self.app.set_theta(thetas[0], thetas[1], thetas[2])

    def get_virtual_coords(self):
        with self.coord_lock:
            if self.virtual_coords is None:
                return None
            return self.virtual_coords.copy()

    def get_next_command_id(self):
        self.command_id_counter += 1
        return self.command_id_counter

    def can_execute(self, command_type):
        """Kiểm tra xem robot có thể thực hiện lệnh không dựa trên state hiện tại. 
        
        Args:
            command_type: Loại lệnh ("move", "home", "servo", "auto")
        
        Returns:
            True nếu có thể thực hiện, False nếu không
        """
        with self.state_lock:
            # ✅ USER REQ: Cho phép điều khiển thủ công (xử lý sự cố) khi đang E-STOP
            # Chỉ chặn AUTO
            
            if command_type == "auto":
                return self.state == C.RobotState.IDLE
            
            elif command_type in ["move", "home", "servo"]:
                return self.state in [C.RobotState.IDLE, C.RobotState.ESTOP]
            
            elif command_type == "jog":
                return self.state in [C.RobotState.IDLE, C.RobotState.JOGGING, C.RobotState.ESTOP]
            
            return False
    
    def check_block_timeout(self):
        """Kiểm tra blocks bị timeout (đã gửi nhưng không nhận DONE sau 15s).
        Gọi định kỳ từ main loop để phát hiện blocks bị mất.
        
        Returns:
            List of timed-out block IDs
        """
        current_time = time.time()
        
        # ✅ Tránh spam: chỉ check mỗi 10s
        if current_time - self.last_timeout_check < 10.0:
            return []
        self.last_timeout_check = current_time
        
        timed_out_blocks = []
        
        with self.sequence_lock:
            for block_id, block_info in list(self.sent_block_ids.items()):
                # Handle both old (timestamp) and new (dict) format
                send_time = block_info["time"] if isinstance(block_info, dict) else block_info
                source = block_info.get("source", "UNKNOWN") if isinstance(block_info, dict) else "UNKNOWN"
                
                age = current_time - send_time
                if age > self.BLOCK_TIMEOUT_SEC:
                    timed_out_blocks.append((block_id, age, source))
            
            # ✅ CLEANUP: Xóa timeout blocks (giả định đã mất)
            if timed_out_blocks:
                for block_id, age, source in timed_out_blocks:
                    self.sent_block_ids.pop(block_id, None)
                
                # ✅ FIX: Reclaim slots for timed-out blocks to prevent "Slot Leak"
                # Nếu timeout nghĩa là ACK đã mất hoặc block đã thực thi xong từ lâu
                with self.slots_lock:
                    self.available_slots += len(timed_out_blocks)
                    if self.available_slots > self.MAX_QUEUE_SIZE:
                         self.available_slots = self.MAX_QUEUE_SIZE
                
                # ✅ REMOVED: Không log nữa để tránh spam
                # ids_with_source = [f"{bid}[{src}]" for bid, _, src in timed_out_blocks[:5]]
                # ages = [f"{bid}({age:.1f}s)" for bid, age, _ in timed_out_blocks[:5]]
                # msg = f"⚠️ CLEANUP: Xóa {len(timed_out_blocks)} blocks timeout: {', '.join(ages)} (Slots reclaimed)"
                # self.app.after(0, lambda m=msg: self.app.log_message(m, "warning"))
        
        return [bid for bid, _, _ in timed_out_blocks]
    
    def get_block_tracking_stats(self):
        """Lấy thống kê block tracking để debug.
        
        Returns:
            Dict với keys: sent_count, received_count, pending_count, oldest_pending_age
        """
        with self.sequence_lock:
            sent_count = len(self.sent_block_ids)
            received_count = len(self.received_done_ids)
            
            oldest_age = 0
            oldest_id = None
            if self.sent_block_ids:
                current_time = time.time()
                for block_id, send_time in self.sent_block_ids.items():
                    age = current_time - send_time
                    if age > oldest_age:
                        oldest_age = age
                        oldest_id = block_id
        
        stats = {
            "sent_pending": sent_count,
            "received_total": received_count,
            "oldest_pending_age": oldest_age,
            "oldest_pending_id": oldest_id
        }
        
        # ✅ DEBUG: Log nếu có blocks pending > 5s
        # if oldest_age > 5.0:
        #     print(f"[BLOCK TRACKING] Pending: {sent_count}, Oldest: ID={oldest_id} ({oldest_age:.1f}s)")
        
        return stats
    


    def reset_system_state(self):
        """Reset toàn bộ trạng thái nội bộ về như lúc mới khởi động."""
        self.app.log_message("🔄 Resetting system state...", "info")
        
        # 1. Reset State & Flags
        with self.state_lock:
            self.state = C.RobotState.IDLE
            self.homing_state = "NOT_HOMED"
            self.is_running_auto = False
            self.jog_stop_requested = False
            self.jog_at_limit = False
            self.jog_pending_count = 0
            self.jog_target_tracker = None
            self.command_id_counter = 0
        
        # 2. Clear Queues & Buffers
        with self.pending_blocks_lock:
            self.pending_blocks_queue.clear()
            self.blocks_in_flight = 0
            self.current_trajectory_block_index = 0
            
        with self.pending_moves_lock:
            self.pending_moves.clear()
            
        with self.operation_lock:
            self.active_move_info.clear()
            self.active_jog_info = None
            
        with self.slots_lock:
            self.available_slots = self.MAX_QUEUE_SIZE
        
        # 3. Clear Block Tracking
        with self.sequence_lock:
            self.sent_block_ids.clear()
            self.received_done_ids.clear()
            
        self.active_command_ids.clear()
        
        # 3. Reset Auto Mode State
        if hasattr(self, 'auto_controller'):
             self.auto_controller.auto_state = AutoState.IDLE
             with self.auto_controller.candidate_lock:
                 self.auto_controller.pick_candidates.clear()
             with self.auto_controller.auto_objects_queue.mutex:
                 self.auto_controller.auto_objects_queue.queue.clear()
             with self.auto_controller.pick_execution_queue.mutex:
                 self.auto_controller.pick_execution_queue.queue.clear()
             self.auto_controller.processed_ids.clear()
             self.auto_controller.triggered_objects.clear()
             self.auto_controller.current_picking_obj_id = None
            
        # 4. Reset Logic Helpers
        self.waiting_for_flush_ack_before_homing = False
        
        self.app.log_message("✅ System state reset complete.", "info")

    def _send_command(self, command_string, block_id=None):
        if not self.conn_manager.is_connected():
            # print(f"[DEBUG] _send_command FAILED: Not connected (command={command_string})")
            # self.app.log_message("Lỗi: Không có kết nối STM32!", "error")
            return False
        if block_id:
            self.active_command_ids.add(str(block_id))
        
        # ✅ DEBUG: Log lệnh đang được gửi (trừ ADD_BLOCK và STATUS để tránh spam)
        # ✅ OPTIMIZATION: ADD_BLOCK giờ không có prefix, check format id:json (bắt đầu bằng số)
        is_add_block = len(command_string) > 0 and command_string[0].isdigit() and ':' in command_string
        is_status = command_string == "STATUS"
        
        if not is_add_block and not is_status:
            print(f"[DEBUG] Sending command: {command_string}")
        
        self.conn_manager.send_command(command_string)
        
        if not is_add_block and not is_status:
            print(f"[DEBUG] Command sent successfully: {command_string[:50]}...")
        
        return True

    def send_add_block(self, params, block_id=None, target_position=None, blocking=True, source="UNKNOWN"):
        """Gửi lệnh ADD_BLOCK và lưu target position để cập nhật sau khi hoàn thành. 
        
        Args:
            params: Dict chứa t, s, a, b
            block_id: ID của block (nếu None sẽ tự tạo)
            target_position: Dict {"coords": (x,y,z), "theta": [t1,t2,t3]} - vị trí đích
            blocking: Nếu True (mặc định), sẽ block chờ slot. Nếu False (cho JOG), trả về None ngay nếu queue đầy.
            source: Nguồn gốc block ("AUTO", "MANUAL", "JOG") - để debug
        """
        # ✅ SAFETY CHECK: Chặn lệnh AUTO nếu mode đã tắt (Zombie Command protection)
        if source == "AUTO" and not self.is_running_auto:
            return None

        # Nếu không có ID, hoặc ID là 0, thì tạo ID mới
        effective_id = block_id if block_id is not None else self.get_next_command_id()
        
        # ✅ CRITICAL: CHỜ buffer nếu đầy (event-based, không block message handler)
        max_wait_time = 5.0  # 5 seconds timeout
        wait_event = threading.Event()
        start_time = time.time()
        
        while True:
            with self.slots_lock:
                if self.available_slots > 0:
                    break  # Có slot trống, tiếp tục
                
                if not blocking:
                    # ✅ NON-BLOCKING MODE (cho JOG): Nếu đầy thì bỏ qua lệnh này
                    return None
                
                # ✅ CRITICAL: Kiểm tra bất thường - nếu slots quá cao, reset
                if self.available_slots > self.MAX_QUEUE_SIZE:
                    self.app.after(0, lambda: self.app.log_message(
                        f"⚠️ WARNING: slots desync detected ({self.available_slots}), resetting to MAX", "error"))
                    self.available_slots = self.MAX_QUEUE_SIZE
                    break
            
            # Kiểm tra timeout
            if time.time() - start_time > max_wait_time:
                # Timeout - buffer vẫn đầy sau 5s -> DEADLOCK DETECTED
                self.app.after(0, lambda: self.app.log_message(
                    f"⚠️ CRITICAL: Queue Deadlock detected (slots={self.available_slots}). Force resetting slots!", "error"))
                
                # Force Reset slots để cứu hệ thống
                with self.slots_lock:
                    self.available_slots = self.MAX_QUEUE_SIZE
                
                # Tùy chọn: Gửi lệnh xóa queue để đồng bộ lại (nếu cần)
                # self._send_command("CLEAR_QUEUE") 
                
                return None
            
            # Chờ 50ms bằng event (không block message handler)
            wait_event.wait(timeout=0.05)
            wait_event.clear()
        
        if not self._validate_add_block_params(params):
            self.app.log_message("ADD_BLOCK bị từ chối: tham số không hợp lệ", "error")
            return None
        
        # ✅ Chuyển đổi sang Fixed-Point Integer trước khi gửi xuống STM32
        params_converted = self._convert_to_fixed_point(params)
            
        params_str = json.dumps(params_converted, separators=(',', ':'))
        # ✅ REVERTED: Thêm lại prefix "ADD_BLOCK:" để đảm bảo tương thích và an toàn
        # Short format "id:json" tạm thời không dùng để debug lỗi JOG
        command = f"ADD_BLOCK:{effective_id}:{params_str}"
        
        # print(f"DEBUG: Sending command: {command}")
        # ✅ PERFORMANCE: Tắt log cho ADD_BLOCK để tránh spam GUI (có thể gửi 66 lệnh/giây)
        # Chỉ bật lại khi cần debug
        # msg = f"📤 Sending ADD_BLOCK ID={effective_id}: {params_str}"
        # self.app.after(0, lambda: self.app.log_message(msg, "sent"))
        
        # Chỉ theo dõi các ID > 0
        send_success = self._send_command(command, block_id=(effective_id if effective_id > 0 else None))
        
        if send_success:
            # ✅ FIX: Track SAU khi gửi thành công (tránh track block không gửi được)
            if effective_id > 0:
                block_id_str = str(effective_id).strip()
                with self.sequence_lock:
                    self.sent_block_ids[block_id_str] = {
                        "time": time.time(),
                        "source": source
                    }
            
            # ✅ FIX BUG 2: Giảm số slots còn trống khi gửi lệnh thành công
            with self.slots_lock:
                self.available_slots -= 1
                if self.available_slots < 0:
                    self.available_slots = 0  # Safety check
            
            # ✅ NEW: Lưu target position để cập nhật khi nhận DONE
            if target_position and effective_id > 0:
                # Lưu góc servo TỌA ĐỘ vào state
                target_servo_coord = self.current_servo_angle
                if "a" in params:
                    target_servo_coord = params["a"]
                
                with self.pending_moves_lock:
                    self.pending_moves[str(effective_id)] = {
                        "target_coords": target_position.get("coords"),
                        "target_theta": target_position.get("theta"),
                        "servo": target_servo_coord,
                        "pump": params.get("b", self.current_pump_state)
                    }
            
            return effective_id
        else:
            # ✅ DEBUG: Log khi send fail (giúp debug timeout)
            # print(f"[SEND FAIL] Block ID={effective_id} không gửi được!")
            return None
    
    def _convert_to_fixed_point(self, params):
        """Chuyển đổi params từ float sang fixed-point integer cho STM32. 
        
        Quy tắc chuyển đổi:
        - "t" (seconds): float → int (milliseconds). VD: 1.5 → 1500
        - "a" (angle degrees): float → int (angle × 100). VD: 135.5 → 13550
        - "s" (steps): giữ nguyên (đã là int)
        - "b" (pump): giữ nguyên (đã là 0/1)
        
        Returns:
            dict: params đã chuyển đổi sang integer
        """
        converted = {}
        
        # Time: giây → mili-giây (x1000)
        # ✅ FIX: Dùng round() trước int() để tránh cắt bỏ (floor)
        # VD: 0.019999 * 1000 = 19.999 → round(19.999) = 20 → int(20) = 20
        # Thay vì int(19.999) = 19 (sai số tích lũy)
        if "t" in params:
            t_ms = int(round(params["t"] * 1000))  # 1.5s → 1500ms
            # ✅ FIX: Đảm bảo thời gian tối thiểu 10ms để STM32 không từ chối
            if t_ms < 10:
                t_ms = 10
            converted["t"] = t_ms
        
        # Angle: độ → độ × 100
        if "a" in params:
            converted["a"] = int(round(params["a"] * 100))   # 135.5° → 13550
        
        # Steps: giữ nguyên
        if "s" in params:
            converted["s"] = params["s"]
        
        # Pump: giữ nguyên
        if "b" in params:
            converted["b"] = params["b"]
        
        return converted

    def send_flush_buffer(self):
        print("[DEBUG] send_flush_buffer called")
        command = "FLUSH_BUFFER"
        if self._send_command(command):
            print("[DEBUG] FLUSH_BUFFER sent successfully")
            self.active_command_ids.clear()
            
            # ✅ CRITICAL: Clear pending moves để tránh memory leak
            with self.pending_moves_lock:
                self.pending_moves.clear()
            
            # ✅ SLIDING WINDOW: Clear pending blocks queue
            with self.pending_blocks_lock:
                self.pending_blocks_queue.clear()
                self.blocks_in_flight = 0
                self.current_trajectory_block_index = 0
            
            # ✅ FIX: KHÔNG reset state ngay ở đây
            # Chờ nhận ACK:FLUSH từ STM32 rồi mới reset state
            # Điều này tránh race condition với xử lý DONE
            
            # ✅ FIX BUG 2: Reset slots về full khi flush
            with self.slots_lock:
                self.available_slots = self.MAX_QUEUE_SIZE
            
            # ✅ SLIDING WINDOW: Clear tracking khi flush trực tiếp
            with self.pending_blocks_lock:
                self.current_trajectory_block_index = 0
        else:
            print("[DEBUG] send_flush_buffer FAILED")

    def _validate_add_block_params(self, params):
        """Kiểm tra tính hợp lệ của payload ADD_BLOCK trước khi gửi xuống STM32.
        Yêu cầu:
          - t: float > 0 và < 30s (giới hạn thực tế an toàn)
          - s: list dài 3 phần tử, toàn số nguyên, mỗi giá trị có |step| < 200000
          - a: float trong [0, 270]
          - b: 0 hoặc 1
        Trả về True nếu hợp lệ, False nếu không.
        """
        if not isinstance(params, dict):
            self.app.log_message(f"❌ Params không phải dict: {type(params)}", "error")
            return False
        required_keys = {"t", "s", "a", "b"}
        if not required_keys.issubset(params.keys()):
            missing = required_keys - set(params.keys())
            self.app.log_message(f"❌ Thiếu trường: {missing}. Có: {set(params.keys())}", "error")
            return False
        # Thời gian
        t = params.get("t")
        try:
            t_val = float(t)
        except (TypeError, ValueError):
            self.app.log_message(f"❌ Trường t không phải số: {t} ({type(t)})", "error")
            return False
        if not (0 < t_val < 30.0):
            self.app.log_message(f"❌ t không hợp lệ (0 < t < 30): {t_val}", "error")
            return False
        # Steps
        s = params.get("s")
        if not (isinstance(s, list) and len(s) == 3):
            self.app.log_message(f"❌ Trường s phải là list 3 phần tử, nhận: {s} ({type(s)})", "error")
            return False
        for idx, v in enumerate(s):
            if not isinstance(v, int):
                self.app.log_message(f"❌ s[{idx}] không phải số nguyên: {v} ({type(v)})", "error")
                return False
            if abs(v) > 200000:
                self.app.log_message(f"❌ s[{idx}] quá lớn: {v}", "error")
                return False
        # Servo angle (coordinate angle: -225° to 45°)
        a = params.get("a")
        try:
            a_val = float(a)
        except (TypeError, ValueError):
            self.app.log_message(f"❌ Trường a không phải số: {a} ({type(a)})", "error")
            return False
        if not (-225.0 <= a_val <= 45.0):
            self.app.log_message(f"❌ Góc servo tọa độ a ngoài phạm vi [-225,45]: {a_val}", "error")
            return False
        # Pump state
        b = params.get("b")
        if b not in (0, 1):
            self.app.log_message(f"❌ Trường b phải là 0 hoặc 1, nhận: {b} ({type(b)})", "error")
            return False
        return True
    
    def _send_blocks_batch(self, is_initial=False):
        """
        ✅ SLIDING WINDOW FLOW CONTROL (AGGRESSIVE REFILL):
        - Initial: Gửi 20 blocks lần đầu (tăng từ 16)
        - Refill: CHỈ gửi khi blocks_in_flight < 6 (giảm từ 8 để sớm hơn)
        - Mỗi lần refill: Gửi 12 blocks (tăng từ 8 để nhanh hơn)
        
        Logic mới:
        - Queue STM32 = 64 slots
        - Giữ queue ở mức 6-30 blocks (cao hơn để tránh starvation)
        - Refill sớm (< 6) và nhiều hơn (12) để robot không giật
        """
        with self.pending_blocks_lock:
            if not self.pending_blocks_queue:
                return  # Hết lệnh
            
            with self.slots_lock:
                free_slots = self.available_slots
            
            # Xác định số lượng cần gửi
            num_to_send = 0
            
            if is_initial:
                # Lần đầu: Gửi 20 blocks (tăng từ 16)
                num_to_send = min(self.INITIAL_BATCH_SIZE, len(self.pending_blocks_queue), free_slots)
            else:
                # ✅ FIX: Refill CHỈ khi thực sự cần (blocks_in_flight < LOW_THRESHOLD)
                # Không cần check free_slots vì đã check blocks_in_flight rồi
                num_to_send = min(self.REFILL_BATCH_SIZE, len(self.pending_blocks_queue), free_slots)
            
            if num_to_send <= 0:
                return
            
            sent_count = 0
            for _ in range(num_to_send):
                if not self.pending_blocks_queue:
                    break
                
                block_info = self.pending_blocks_queue.pop(0)  # FIFO
                params = block_info["params"]
                target_position = block_info["target_position"]
                is_last = block_info.get("is_last", False)
                
                # Dùng block index làm ID
                block_id = self.current_trajectory_block_index
                self.current_trajectory_block_index += 1
                
                # Non-blocking send
                result = self.send_add_block(params, block_id=block_id, target_position=target_position, blocking=False, source="MANUAL")
                
                if result is None:
                    # Thất bại (lý thuyết không nên xảy ra do đã check slots), trả lại queue
                    self.pending_blocks_queue.insert(0, block_info)
                    self.current_trajectory_block_index -= 1
                    break
                
                self.blocks_in_flight += 1
                sent_count += 1
                
                # ✅ FIX STARVATION: Giảm delay 10ms → 5ms để refill nhanh hơn
                time.sleep(0.005)  # 5ms delay
                
                if is_last:
                    with self.operation_lock:
                        move_info = block_info.get("move_info")
                        if move_info:
                            self.active_move_info[str(block_id)] = move_info
            
            if sent_count > 0:
                # ✅ DEBUG: Log số blocks đã gửi - DISABLED (too verbose)
                pass
                # self.app.after(0, lambda n=sent_count, remain=len(self.pending_blocks_queue), inflight=self.blocks_in_flight: 
                #     self.app.log_message(f"📤 Sent batch: {n} blocks (Pending: {remain}, In-flight: {inflight})", "info"))

    def _request_status(self):
        """Gửi lệnh STATUS lấy trạng thái robot."""
        self._send_command("STATUS")

    def _start_status_polling(self):
        if getattr(self, "_status_polling", False):
            return
        self._status_polling = True
        def poll():
            if not getattr(self, "_status_polling", False):
                return
            if self.conn_manager.is_connected():
                self._request_status()
                
                # ✅ OPTIMIZED POLLING: Giảm interval để phản hồi nút nhấn nhanh hơn
                interval = 100  # 100ms khi IDLE (giảm từ 500ms → phản hồi nhanh hơn)
                if self.state == C.RobotState.MOVING or self.state == C.RobotState.HOMING:
                    interval = 50  # 50ms khi đang bận (giảm từ 200ms → update realtime hơn)
                
                self.app.after(interval, poll)
            else:
                self._status_polling = False
        self.app.after(0, poll)
    
    def send_conveyor_start(self, forward):
        """Gửi lệnh khởi động băng tải."""
        if not self.conn_manager.is_connected():
            return False
            
        dir_str = "FWD" if forward else "REV"
        dir_vn = "Tiến" if forward else "Lùi"
        self.app.log_message(f"⏩ Conveyor: {dir_vn}", "sent")
        command = f"CONVEYOR:START:{dir_str}"
        success = self._send_command(command)
        
        # ✅ Cập nhật indicator băng tải
        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_CONVEYOR_1, True))
        
        return success

    def send_conveyor_stop(self):
        """Gửi lệnh dừng băng tải."""
        if not self.conn_manager.is_connected():
            return False

        self.app.log_message("⏸️ Conveyor: Dừng", "sent")
        command = "CONVEYOR:STOP"
        success = self._send_command(command)
        
        # ✅ Cập nhật indicator băng tải
        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_CONVEYOR_1, False))
        
        return success

    def send_conveyor_set_speed(self, speed):
        """Gửi lệnh đặt tốc độ băng tải (mm/s)."""
        if not self.conn_manager.is_connected():
            return False

        self.app.log_message(f"⏩ Conveyor: Tốc độ {speed} mm/s", "sent")
        # Cập nhật biến nội bộ để tính toán timing
        self.conveyor_speed_mm_s = float(speed)
        command = f"CONVEYOR:SET_SPEED:{speed}"
        success = self._send_command(command)
        
        return success
    
    def _get_kinematics_alpha(self, servo_angle_coord):
        """
        Chuyển đổi góc servo (Hệ tọa độ) sang góc alpha cho kinematics.
        
        CÔNG THỨC MỚI (Coordinate System):
        - servo_coord = -90° (HOME) → alpha = -90° (hướng Y-)
        - servo_coord = 45° → alpha = -225° ≡ 135° (hướng X-)
        - Công thức: alpha = -180.0 - servo_angle_coord
        """
        alpha_deg = -180.0 - servo_angle_coord
        return alpha_deg



    def update_hardware_button_leds(self):
        """Cập nhật trạng thái đèn LED trên nút bấm cứng dựa trên trạng thái hệ thống."""
        if not self.conn_manager.is_connected():
            return

        # Điều kiện: Đã kết nối, Đã Home, Không E-Stop, Không Manual
        is_homed = (self.homing_state == "COMPLETED")
        is_estop = (self.state == C.RobotState.ESTOP)
        # Lưu ý: Cần truy cập an toàn vào UI thread hoặc biến cache
        try:
            is_manual = self.app.is_manual_mode()
        except:
            is_manual = False # Fallback

        # ✅ CHECK CAMERA STATUS
        is_camera_ready = False
        if hasattr(self.app, 'video_thread') and self.app.video_thread:
             if self.app.video_thread.running and self.app.video_thread.cap and self.app.video_thread.cap.isOpened():
                 is_camera_ready = True
        
        # ✅ CHECK TRAY SENSOR
        is_tray_present = self.is_tray_present

        # ✅ Đèn START chỉ sáng khi:
        # 1. Đủ điều kiện (homed, không estop, không manual, camera ok, có khay)
        # 2. CHƯA ĐANG CHẠY AUTO (nếu đang chạy thì TẮT đèn)
        can_start_auto = is_homed and not is_estop and not is_manual and is_camera_ready and is_tray_present
        should_light_on = can_start_auto and not self.is_running_auto
        
        # Gửi lệnh điều khiển LED (START Button - ID 0)
        state = 1 if should_light_on else 0
        
        # Cache để tránh spam lệnh
        if getattr(self, "_last_btn_led_state", -1) != state:
            # Gửi lệnh BTN_LED:0:state
            self._send_command(f"BTN_LED:0:{state}")
            self._last_btn_led_state = state

    def _handle_hard_button_press(self, btn_id):
        """Xử lý sự kiện nút cứng trên luồng GUI (Main Thread)."""
        current_time = time.time()
        if current_time - self.last_button_press_time < 0.5:
            return
        self.last_button_press_time = current_time

        if btn_id == 0:  # START
            self.app.log_message("🔘 [HARDWARE] Nút START đã nhấn", "received")
            
            if self.is_running_auto:
                self.app.log_message("⚠️ Auto đã đang chạy.", "warning")
                return

            if self.state == C.RobotState.ESTOP:
                self.app.log_message("❌ Lỗi: Hệ thống đang E-STOP!", "error")
                return

            if self.homing_state != "COMPLETED":
                self.app.log_message("❌ Lỗi: Robot chưa về HOME!", "error")
                return

            if self.app.is_manual_mode():
                self.app.log_message("❌ Lỗi: Cần tắt chế độ MANUAL trước khi chạy AUTO!", "error")
                return
            
            self.set_auto_mode(True)
                
        elif btn_id == 1:  # STOP
            self.app.log_message("🛑 [HARDWARE] Nút STOP đã nhấn", "received")
            if self.is_running_auto:
                self.set_auto_mode(False)
            else:
                self.send_flush_buffer()
                self.app.log_message("ℹ️ Hệ thống đã được Flush.", "info")

    def handle_stm_message(self, message):
        """Xử lý các tin nhắn từ STM32."""
        try:
                    # ✅ DEBUG: Log tất cả message để debug
                    # print(f"[DEBUG handle_stm_message] Received: '{message}'")
                    
                    # parts = message.strip().split(':', 1)
                    # Dùng strip() để loại bỏ whitespace thừa trước khi split
                    clean_msg = message.strip()
                    parts = clean_msg.split(':', 1)
                    msg_type = parts[0]
        
                    if msg_type == "DONE":
                        if len(parts) > 1:
                            done_id = parts[1].strip()  # ✅ FIX: Strip whitespace
                            
                            # ✅ VALIDATION: Kiểm tra block sequence (SILENT MODE - chỉ log debug)
                            with self.sequence_lock:
                                if done_id in self.sent_block_ids:
                                    # Block hợp lệ - xóa khỏi sent list
                                    block_info = self.sent_block_ids.pop(done_id)
                                    send_time = block_info["time"] if isinstance(block_info, dict) else block_info
                                    self.received_done_ids.add(done_id)
                                    
                                    # ✅ CLEANUP: Giới hạn received_done_ids (max 1000 items)
                                    if len(self.received_done_ids) > 1000:
                                        # Xóa 500 items cũ nhất (giả sử ID tăng dần)
                                        to_remove = sorted(self.received_done_ids)[:500]
                                        for old_id in to_remove:
                                            self.received_done_ids.discard(old_id)
                                    
                                    # Optional: Log thời gian thực hiện (chỉ khi quá chậm)
                                    execution_time = time.time() - send_time
                                    if execution_time > 10.0:  # Chỉ cảnh báo khi > 10s
                                        self.app.after(0, lambda id=done_id, t=execution_time: self.app.log_message(
                                            f"⚠️ Block {id} chạy lâu: {t:.2f}s", "warning"))
                                # else:
                                #     # ✅ SILENT: Không log error (có thể do FLUSH race condition)
                                #     pass
                            
                            # ✅ FIX BUG 2: Tăng số slots còn trống khi nhận DONE
                            with self.slots_lock:
                                self.available_slots += 1
                                if self.available_slots > self.MAX_QUEUE_SIZE:
                                    # ✅ SILENT FIX: Nếu slots > MAX (do race condition khi Flush/Stop), 
                                    # chỉ cần clamp về MAX, không cần báo lỗi gây hoang mang.
                                    # self.app.after(0, lambda: self.app.log_message(
                                    #     f"⚠️ WARNING: slots overflow ({self.available_slots}/{self.MAX_QUEUE_SIZE}), reset to MAX", "warning"))
                                    self.available_slots = self.MAX_QUEUE_SIZE
                            
                            # ✅ SLIDING WINDOW: Giảm blocks_in_flight và refill nếu cần
                            with self.pending_blocks_lock:
                                if self.blocks_in_flight > 0:
                                    self.blocks_in_flight -= 1
                                    
                                    # ✅ FIX STARVATION: Refill khi:
                                    # 1. blocks_in_flight < threshold (bình thường)
                                    # 2. available_slots > 48 (queue STM32 gần cạn - backup mechanism)
                                    should_refill = (
                                        self.pending_blocks_queue and 
                                        (self.blocks_in_flight < self.BUFFER_LOW_THRESHOLD or 
                                         self.available_slots > 48)
                                    )
                                    
                                    if should_refill:
                                        # ✅ WARNING: Nếu backup refill (queue cạn kiệt)
                                        # if self.available_slots > 48:
                                        #     print(f"[STARVATION WARNING] Queue STM32 cạn ({self.available_slots}/64 free) - Force refill!")
                                        #     self.app.after(0, lambda: self.app.log_message(
                                        #         f"⚠️ Queue gần cạn kiệt! Force refill {self.REFILL_BATCH_SIZE} blocks", "warning"))
                                        
                                        self._send_blocks_batch()
                            
                            # ✅ NEW: Cập nhật vị trí dựa trên target position đã lưu
                            with self.pending_moves_lock:
                                if done_id in self.pending_moves:
                                    move_info = self.pending_moves.pop(done_id)
                                    target_coords = move_info.get("target_coords")
                                    target_theta = move_info.get("target_theta")
                                    
                                    if target_coords:
                                        x, y, z = target_coords
                                        # ✅ PERFORMANCE FIX: KHÔNG cập nhật GUI khi đang JOGGING để tránh đơ máy
                                        should_update_gui = (self.state != C.RobotState.JOGGING)
                                        self._update_virtual_coords(x, y, z, theta=target_theta, update_gui=should_update_gui)
                                    
                                    if target_theta:
                                        with self.state_lock:
                                            self.current_theta = list(target_theta)
                                    
                                    # Cập nhật servo và pump state
                                    self.current_servo_angle = move_info.get("servo", self.current_servo_angle)
                                    self.current_pump_state = move_info.get("pump", self.current_pump_state)
                            
                            if done_id in self.active_command_ids:
                                self.active_command_ids.remove(done_id)
                            
                            # ✅ Đơn giản: cập nhật state dựa trên state hiện tại
                            with self.state_lock:
                                if self.state == C.RobotState.MOVING:
                                    with self.operation_lock:
                                        is_last_block = done_id in self.active_move_info
                                        move_info = {}
                                        if is_last_block:
                                            move_info = self.active_move_info[done_id]
                                            del self.active_move_info[done_id]
                                    
                                    if is_last_block:
                                        self.state = C.RobotState.IDLE
                                        total = move_info.get("total_blocks", "?")
                                        target = move_info.get("target")
                                        if target:
                                            self.app.after(0, lambda t=total, pos=target: 
                                                self.app.log_message(f"✅ Đã đến {pos} (block {t}/{t})", "received"))
                                        else:
                                            self.app.after(0, lambda: 
                                                self.app.log_message("✅ Di chuyển hoàn tất", "received"))
                                elif self.state == C.RobotState.JOGGING:
                                    if self.jog_pending_count > 0:
                                        self.jog_pending_count -= 1
                                    
                                    if not self.jog_stop_requested and not self.jog_at_limit:
                                        if self.jog_pending_count < self.MAX_JOG_PENDING:
                                            self._send_jog_command()
                                    else:
                                        if self.jog_pending_count == 0:
                                            self.state = C.RobotState.IDLE
        
                            # [AUTO V3] Xử lý DONE trong chế độ AUTO
                            if self.is_running_auto:
                                self.auto_controller.handle_done_message(done_id)
        
                    elif msg_type == "HOME_DONE":
                        try:
                            print("[DEBUG HOME] Received HOME_DONE from STM32!")
                            with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                                f.write(f"[{time.strftime('%H:%M:%S')}] HOME_DONE received\n")
                        except Exception as log_err:
                            print(f"[ERROR HOME] Failed to write log: {log_err}")
                        
                        try:
                            self.homing_state = "COMPLETED"
                            
                            # ✅ Update state
                            with self.state_lock:
                                # Nếu đang ESTOP thì giữ nguyên, ngược lại mới về IDLE
                                if self.state != C.RobotState.ESTOP:
                                    self.state = C.RobotState.IDLE
                                self.current_theta = [C.HOME_DEFAULT_THETA_DEG, C.HOME_DEFAULT_THETA_DEG, C.HOME_DEFAULT_THETA_DEG]
                                self.current_pump_state = 0
                                self.current_servo_angle = C.HOME_DEFAULT_SERVO_ANGLE_DEG
                            
                            # ✅ FIX: Calculate HOME position with fallback
                            home_alpha_deg = self._get_kinematics_alpha(C.HOME_DEFAULT_SERVO_ANGLE_DEG)
                            home_coords_fk = self.kinematics.forward_kinematics_tool(
                                C.HOME_DEFAULT_THETA_DEG, C.HOME_DEFAULT_THETA_DEG, C.HOME_DEFAULT_THETA_DEG, home_alpha_deg
                            )
                            
                            if home_coords_fk:
                                x_home, y_home, z_home = home_coords_fk
                                print(f"[DEBUG HOME] FK tính được: ({x_home:.2f}, {y_home:.2f}, {z_home:.2f})")
                            else:
                                # ⚠️ FALLBACK: Nếu FK thất bại, dùng tọa độ cứng (đo thực tế)
                                x_home, y_home, z_home = 0.0, 15.0, -357.0
                                print(f"[WARNING HOME] FK Home failed, using fallback: ({x_home:.2f}, {y_home:.2f}, {z_home:.2f})")
                                self.app.after(0, lambda: self.app.log_message("⚠️ FK Home thất bại - dùng tọa độ fallback", "warning"))
                            
                            home_theta = [C.HOME_DEFAULT_THETA_DEG, C.HOME_DEFAULT_THETA_DEG, C.HOME_DEFAULT_THETA_DEG]
                            
                            # ✅ Update virtual coords and GUI (QUAN TRỌNG: update_gui=True)
                            self._update_virtual_coords(x_home, y_home, z_home, theta=home_theta, update_gui=True)
                            print(f"[DEBUG HOME] _update_virtual_coords called with update_gui=True")

                            # ✅ Update other GUI elements
                            home_coord_angle = C.HOME_DEFAULT_SERVO_ANGLE_DEG # Đã là góc tọa độ (-90)
                            
                            # Update messages
                            self.app.after(0, lambda: self.app.log_message("✅ HOME hoàn thành - Robot đã về vị trí gốc", "received"))
                            self.app.after(0, lambda: self.app.log_message(f"🏠 Home position: X={x_home:.2f}, Y={y_home:.2f}, Z={z_home:.2f}", "info"))
                            
                            # Update indicators
                            self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, True))
                            self.app.after(0, lambda: self.app.set_servo_angle(home_coord_angle))
                            
                            # Update pump switch
                            if hasattr(self.app, 'pump_switch'):
                                self.app.after(0, lambda: setattr(self.app.pump_switch, 'state', False))
                            
                            # Update hardware button LEDs
                            self.app.after(0, self.update_hardware_button_leds)
                            
                        except Exception as e:
                            print(f"[ERROR HOME] HOME_DONE handler failed: {e}")
                            import traceback
                            traceback.print_exc()
        
                    elif msg_type == "SYS_READY":
                        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_STM_CONNECTED, True))
                        self.homing_state = "NOT_HOMED"
                        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, False))
                        with self.slots_lock:
                            self.available_slots = self.MAX_QUEUE_SIZE
                        self._start_status_polling()
                        self.app.after(0, self.update_hardware_button_leds)
        
                    if msg_type == "PONG":
                        self.app.after(0, lambda: self.app.log_message("✓ Kết nối thành công (phản hồi PONG)", "received"))
                        
                        # ✅ FIX: Bắt đầu polling STATUS ngay khi kết nối lại (quan trọng!)
                        self._start_status_polling()
                        with self.slots_lock:
                            self.available_slots = self.MAX_QUEUE_SIZE
                        
                        # ✅ NEW: Xử lý trạng thái E-Stop gửi kèm PONG
                        if len(parts) > 1 and parts[1] == "ESTOP":
                            with self.state_lock:
                                self.state = C.RobotState.ESTOP
                            
                            # ✅ Reset toàn bộ hàng đợi giống như ESTOP_TRIGGERED
                            with self.pending_moves_lock: self.pending_moves.clear()
                            with self.pending_blocks_lock: 
                                self.pending_blocks_queue.clear()
                                self.blocks_in_flight = 0
                                self.current_trajectory_block_index = 0
                            with self.operation_lock:
                                self.active_move_info.clear()
                                self.active_jog_info = None
                            with self.slots_lock:
                                self.available_slots = self.MAX_QUEUE_SIZE
                                
                            self.set_auto_mode(False)

                            self.app.after(0, lambda: self.app.log_message("⚠️ Cảnh báo: Robot đang ở trạng thái E-STOP!", "warning"))
                            
                            # ✅ Tự động bật Manual Mode để người dùng xử lý sự cố
                            self.app.after(0, lambda: self.app.set_manual_mode(True))
                            self.app.after(0, lambda: self.app.log_message("⚠️ Chuyển sang chế độ MANUAL để xử lý.", "info"))
                            
                            # Không bật đèn ERROR để tránh gây hiểu nhầm là hệ thống bị khóa
                            self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_ERROR, False))
                            # ✅ Bật đèn E-STOP để báo hiệu trạng thái nguy hiểm
                            self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_E_STOP, True))
                        
                        self.app.after(0, self.update_hardware_button_leds)
        
                    elif msg_type == "ESTOP_OFF":
                        with self.state_lock:
                            self.state = C.RobotState.IDLE
                            self.homing_state = "NOT_HOMED"
                        
                        # ✅ FIX: Reset toàn bộ hàng đợi khi nhả E-Stop để tránh kẹt lệnh cũ
                        with self.pending_moves_lock: self.pending_moves.clear()
                        with self.pending_blocks_lock: 
                            self.pending_blocks_queue.clear()
                            self.blocks_in_flight = 0
                            self.current_trajectory_block_index = 0
                        with self.operation_lock:
                            self.active_move_info.clear()
                            self.active_jog_info = None
                        with self.slots_lock:
                             self.available_slots = self.MAX_QUEUE_SIZE

                        self.app.after(0, lambda: self.app.set_manual_mode_enabled(True))
                        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_ERROR, False))
                        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_E_STOP, False)) # ✅ Tắt đèn E-STOP
                        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, False))
                        self.app.after(0, lambda: self.app.log_message("✅ E-Stop đã nhả - Robot sẵn sàng (Cần Home lại)", "info"))
                        self.app.after(0, self.update_hardware_button_leds)
        
                    elif msg_type == "ERROR":
                        if len(parts) > 1:
                            error_message = parts[1]
                            self.app.after(0, lambda: self.app.log_message(f"STM32 Error: {error_message}", "error"))
                            
                            # 1. Xử lý Homing Timeout (Cũ)
                            if error_message.startswith("HOMING_TIMEOUT"):
                                self.homing_state = "NOT_HOMED"
                                with self.state_lock:
                                    self.state = C.RobotState.IDLE
                                self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, False))
                                self.app.after(0, self.update_hardware_button_leds)
                            
                            # 2. ✅ Xử lý Lỗi Nghiêm Trọng (CRITICAL) - Ví dụ: Chạm LS khi đang chạy
                            elif "CRITICAL" in error_message:
                                self.app.after(0, lambda: self.app.log_message("🛑 LỖI NGHIÊM TRỌNG: Dừng hệ thống!", "error"))
                                
                                # Dừng Auto và Xóa hàng đợi PC
                                self.set_auto_mode(False)
                                
                                # Reset queues PC để đồng bộ với việc STM32 đã flush
                                with self.pending_moves_lock: self.pending_moves.clear()
                                with self.pending_blocks_lock: 
                                    self.pending_blocks_queue.clear()
                                    self.blocks_in_flight = 0
                                    self.current_trajectory_block_index = 0
                                with self.slots_lock:
                                     self.available_slots = self.MAX_QUEUE_SIZE
                                
                                # Chuyển về trạng thái an toàn (IDLE nhưng chưa Home)
                                with self.state_lock:
                                    self.state = C.RobotState.IDLE
                            
                            # 3. ✅ Xử lý Yêu cầu Homing Lại (REQUIRE_HOMING)
                            if "REQUIRE_HOMING" in error_message:
                                self.homing_state = "NOT_HOMED"
                                self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, False))
                                self.app.after(0, lambda: self.app.log_message("⚠️ YÊU CẦU HOMING LẠI: Vị trí robot không còn tin cậy.", "warning"))
                                self.app.after(0, self.update_hardware_button_leds)
                                
                    elif msg_type == "BTN_PRESS":
                        if len(parts) > 1:
                            try:
                                btn_id = int(parts[1])
                                self.app.after(0, lambda: self._handle_hard_button_press(btn_id))
                            except ValueError:
                                pass
        
                    elif msg_type == "EVENT":
                        if len(parts) > 1:
                            event_type = parts[1].strip()
                            
                            if event_type == "TRAY_LOST":
                                self.is_tray_present = False
                                self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_TRAY_SENSOR, False))
                                self.app.after(0, lambda: self.app.log_message("⚠️ Mất tín hiệu khay hứng!", "warning"))
                                
                                # Nếu đang chạy Auto mà mất khay -> DỪNG KHẨN CẤP
                                if self.is_running_auto:
                                    self.app.after(0, lambda: self.app.log_message("🛑 DỪNG KHẨN CẤP: Mất khay khi đang chạy Auto!", "error"))
                                    self.auto_controller.stop()
                                    
                            elif event_type == "TRAY_FOUND":
                                self.is_tray_present = True
                                self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_TRAY_SENSOR, True))
                                self.app.after(0, lambda: self.app.log_message("✅ Đã nhận khay hứng", "info"))

                    elif msg_type == "ACK":
                        print(f"[DEBUG ACK RAW] Received ACK | parts={parts}")
                        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                            f.write(f"[{time.strftime('%H:%M:%S')}] ACK RAW: parts={parts}\n")
                        
                        if len(parts) > 1:
                            ack_cmd = parts[1].strip().upper()
                            print(f"[DEBUG ACK] Parsed ack_cmd='{ack_cmd}'")
                            with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                                f.write(f"[{time.strftime('%H:%M:%S')}] Parsed ack_cmd='{ack_cmd}'\n")
                            
                            if "FLUSH" in ack_cmd:  # ✅ REMOVED: ABORT - Không còn xử lý ACK:ABORT
                                try:
                                    print(f"[DEBUG ACK] Entering FLUSH handler | homing_state={self.homing_state}")
                                    with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                                        f.write(f"[{time.strftime('%H:%M:%S')}] Entering FLUSH handler | homing_state={self.homing_state}\n")
                                    
                                    with self.slots_lock:
                                        self.available_slots = self.MAX_QUEUE_SIZE
                                    
                                    with self.pending_moves_lock:
                                        self.pending_moves.clear()
                                    
                                    with self.pending_blocks_lock:
                                        self.pending_blocks_queue.clear()
                                        self.blocks_in_flight = 0
                                        self.current_trajectory_block_index = 0
                                    
                                    with self.operation_lock:
                                        self.active_move_info.clear()
                                        self.active_jog_info = None
                                    
                                    # ✅ NEW: Clear block tracking
                                    with self.sequence_lock:
                                        self.sent_block_ids.clear()
                                        self.received_done_ids.clear()
                                    
                                    with self.state_lock:
                                        if self.state != C.RobotState.ESTOP:
                                            self.state = C.RobotState.IDLE
                                    self.active_command_ids.clear()
                                    with self.pending_moves_lock:
                                        self.pending_moves.clear()
                                    
                                    with self.state_lock:
                                        if self.state != C.RobotState.ESTOP:
                                            self.state = C.RobotState.IDLE
                                        self.jog_stop_requested = False
                                        self.jog_at_limit = False
                                        self.jog_pending_count = 0
                                    
                                    print(f"[DEBUG ACK] After cleanup, before log message | homing_state={self.homing_state}")
                                    self.app.after(0, lambda msg=ack_cmd: self.app.log_message(f"ACK:{msg} nhận được - Queue cleared.", "received"))
                                    
                                    print(f"[DEBUG ACK] Before HOME check | homing_state={self.homing_state}, 'FLUSH' in ack_cmd={'FLUSH' in ack_cmd}")
                                    with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                                        f.write(f"[{time.strftime('%H:%M:%S')}] Before HOME check | homing_state={self.homing_state}\n")
                                    
                                    # ✅ FIX: Kiểm tra homing_state thay vì waiting_flag
                                    if "FLUSH" in ack_cmd and self.homing_state == "IN_PROGRESS":
                                        print("[DEBUG ACK] Condition TRUE -> Calling _send_home_command_after_flush()")
                                        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                                            f.write(f"[{time.strftime('%H:%M:%S')}] Condition TRUE -> Calling HOME\n")
                                        
                                        self.app.after(0, lambda: self.app.log_message("✅ ACK:FLUSH OK -> Gửi lệnh HOME ngay!", "sent"))
                                        self.waiting_for_flush_ack_before_homing = False
                                        # ✅ FIX: Khôi phục state HOMING trước khi gửi lệnh HOME
                                        with self.state_lock:
                                            # Nếu đang ESTOP thì giữ nguyên, không chuyển sang HOMING
                                            if self.state != C.RobotState.ESTOP:
                                                self.state = C.RobotState.HOMING
                                        self._send_home_command_after_flush()
                                    else:
                                        # ✅ NEW: Nếu Flush không phải để chuẩn bị Home, thì có nghĩa là Stop/Abort
                                        # Khi đó STM32 đã reset Home, PC cũng phải reset theo
                                        # ⚠️ CHÚ Ý: Chỉ reset Home nếu là lệnh FLUSH cứng (STOP), 
                                        # không reset nếu là FLUSH_AFTER_CURRENT (JOG STOP)
                                        if ack_cmd == "FLUSH" and self.homing_state == "COMPLETED":
                                            self.homing_state = "NOT_HOMED"
                                            self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, False))
                                            self.app.after(0, lambda: self.app.log_message("⚠️ Đã dừng & Reset Home", "warning"))
                                        
                                        print(f"[DEBUG ACK] Condition FALSE -> homing_state={self.homing_state}, skipping HOME")
                                        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                                            f.write(f"[{time.strftime('%H:%M:%S')}] Condition FALSE -> homing_state={self.homing_state}\n")
                                except Exception as e:
                                    print(f"[DEBUG ACK] EXCEPTION in FLUSH handler: {e}")
                                    import traceback
                                    traceback.print_exc()
                                    with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                                        f.write(f"[{time.strftime('%H:%M:%S')}] EXCEPTION: {e}\n")
                            
                            elif ack_cmd == "FLUSH_AFTER_CURRENT":
                                with self.state_lock:
                                    if self.jog_pending_count > 1:
                                        self.jog_pending_count = 1
                                self.app.after(0, lambda: self.app.log_message("🛑 JOG dừng nhanh (Queue flushed)", "received"))
                            
                            # ✅ REMOVED: Xử lý ACK:MOTORS ENABLED/DISABLED - Không dùng (PING tự động ENABLE)
                            # elif "MOTORS ENABLED" in ack_cmd:
                            #     self.homing_state = "NOT_HOMED"
                            #     self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, False))
                            #     self.app.after(0, lambda: self.app.log_message("✓ Motors kích hoạt - Cần HOME để đồng bộ", "received"))
        
                    elif msg_type == "STATUS":
                        if len(parts) > 1:
                            try:
                                # Format Updated (V3): STATUS:RS:HOMED:ESTOP:BTN_START:BTN_STOP:CONV:SPEED:PUMP:SERVO:TRAY:S1,S2,S3
                                rest = parts[1]
                                p = rest.split(':')
                                
                                if len(p) >= 11:
                                    cache = self._last_status_cache
                                    
                                    # 1. Robot State (RS) - Reference only
                                    rs_int = int(p[0])
                                    
                                    # ✅ SELF-CORRECTION: Nếu STM32 báo IDLE mà PC vẫn nghĩ là MOVING -> Force Reset
                                    # Giúp thoát khỏi trạng thái kẹt "Robot Busy" do mất gói tin DONE
                                    if rs_int == 0 and self.state == C.RobotState.MOVING:
                                        # Double check: Chỉ reset nếu đã trôi qua ít nhất 0.5s kể từ lệnh cuối
                                        # để tránh race condition khi lệnh vừa mới gửi đi
                                        if time.time() - self.last_move_command_time > 0.5:
                                            self.app.after(0, lambda: self.app.log_message("⚠️ Sync: STM32 đã xong nhưng mất DONE -> Force IDLE", "warning"))
                                            with self.state_lock:
                                                self.state = C.RobotState.IDLE
                                            # Clean up
                                            with self.operation_lock:
                                                self.active_move_info.clear()
                                            self.active_command_ids.clear()
                                            # Refill slots if needed (assume full if idle)
                                            with self.slots_lock:
                                                 self.available_slots = self.MAX_QUEUE_SIZE
                                    
                                    # 2. Homed State
                                    is_homed_stm = (int(p[1]) == 1)
                                    if is_homed_stm != cache["is_homed"]:
                                        is_initial_sync = (cache["is_homed"] is None)
                                        cache["is_homed"] = is_homed_stm
                                        self.homing_state = "COMPLETED" if is_homed_stm else "NOT_HOMED"
                                        self.app.after(0, lambda s=is_homed_stm: self.app.update_indicator(C.INDICATOR_HOME_OK, s))
                                        
                                        if is_homed_stm:
                                            msg = "✅ Đồng bộ: Robot đã HOME"
                                        else:
                                            msg = "⚠️ Đồng bộ: Robot chưa HOME" if is_initial_sync else "⚠️ Đồng bộ: Mất HOME"
                                        
                                        self.app.after(0, lambda m=msg: self.app.log_message(m, "info"))

                                    # 3. E-Stop
                                    estop_val = int(p[2])
                                    is_estop = (estop_val == 1)
                                    if is_estop != cache["is_estop"]:
                                        cache["is_estop"] = is_estop
                                        self.app.after(0, lambda s=is_estop: self.app.update_indicator(C.INDICATOR_E_STOP, s))
                                        with self.state_lock:
                                            if is_estop: 
                                                self.state = C.RobotState.ESTOP
                                                self.app.after(0, lambda: self.app.set_manual_mode(True))

                                    # 4. Buttons (Start/Stop) - CHANGE DETECTION (Rising Edge)

                                    btn_start = (int(p[3]) == 1)
                                    btn_stop = (int(p[4]) == 1)

                                    # ✅ PHÁT HIỆN LỖI ĐẤU DÂY: Nếu lần đầu sync mà đã là 1 -> Cảnh báo
                                    if cache["btn_start"] is None and btn_start == 1:
                                        self.app.after(0, lambda: self.app.log_message("⚠️ Cảnh báo: Nút START đang báo mức 1 (Kẹt hoặc đấu ngược?)", "error"))
                                    if cache["btn_stop"] is None and btn_stop == 1:
                                        self.app.after(0, lambda: self.app.log_message("⚠️ Cảnh báo: Nút STOP đang báo mức 1 (Kẹt hoặc đấu ngược?)", "error"))

                                    # ✅ RISING EDGE DETECTION: 0 (nhả) -> 1 (nhấn)
                                    # Đã chuyển sang dùng Signal hardware_start_pressed/stop_pressed bên dưới
                                    # để tránh gọi set_auto_mode 2 lần (1 ở đây, 1 ở main.py)
                                    # if cache["btn_start"] == 0 and btn_start == 1:
                                    #     self.app.after(0, lambda: self._handle_hard_button_press(0))
                                    # if cache["btn_stop"] == 0 and btn_stop == 1:
                                    #     self.app.after(0, lambda: self._handle_hard_button_press(1))

                                    # --- PHÁT TÍN HIỆU hardware_start_pressed/hardware_stop_pressed ---
                                    # Phát hiện cạnh lên (0->1) cho tín hiệu phần mềm
                                    if self._last_btn_start_state_signal == 0 and btn_start == 1:
                                        print("="*60)
                                        print("🔴 [HARDWARE BUTTON] START PRESSED (Rising Edge 0→1)")
                                        print("="*60)
                                        self.app.after(0, lambda: self.app.log_message("🔴 Nút START cứng đã nhấn", "info"))
                                        self.hardware_start_pressed.emit()
                                    if self._last_btn_stop_state_signal == 0 and btn_stop == 1:
                                        print("="*60)
                                        print("🟢 [HARDWARE BUTTON] STOP PRESSED (Rising Edge 0→1)")
                                        print("="*60)
                                        self.app.after(0, lambda: self.app.log_message("🟢 Nút STOP cứng đã nhấn", "info"))
                                        self.hardware_stop_pressed.emit()
                                    self._last_btn_start_state_signal = btn_start
                                    self._last_btn_stop_state_signal = btn_stop

                                    # DEBUG: In ra console bit nút bấm khi có thay đổi
                                    if btn_start != cache["btn_start"] or btn_stop != cache["btn_stop"]:
                                        if cache["btn_start"] is not None: # Bỏ qua lần đầu sync
                                            print(f"[DEBUG BUTTONS] START: {cache['btn_start']} -> {btn_start} | STOP: {cache['btn_stop']} -> {btn_stop}")
                                            print(f"[DEBUG EDGE] Last START signal: {self._last_btn_start_state_signal}, Current: {btn_start}")
                                            print(f"[DEBUG EDGE] Last STOP signal: {self._last_btn_stop_state_signal}, Current: {btn_stop}")

                                    cache["btn_start"] = btn_start
                                    cache["btn_stop"] = btn_stop

                                    # 5. Conveyor Status
                                    conv_val = int(p[5])
                                    is_conv_running = (conv_val != 0)
                                    if is_conv_running != cache["conv_status"]:
                                        cache["conv_status"] = is_conv_running
                                        self.app.after(0, lambda s=is_conv_running: self.app.update_indicator(C.INDICATOR_CONVEYOR_1, s))

                                    # 6. Conveyor Speed
                                    conv_speed = int(p[6])
                                    if conv_speed != cache["conv_speed"]:
                                        cache["conv_speed"] = conv_speed
                                        self.conveyor_speed_mm_s = float(conv_speed)
                                        self.app.after(0, lambda s=conv_speed: self.app.update_conveyor_speed_display(s) if hasattr(self.app, 'update_conveyor_speed_display') else None)

                                    # 7. Pump
                                    pump_val = int(p[7])
                                    if pump_val != cache["pump_status"]:
                                        cache["pump_status"] = pump_val
                                        is_on = (pump_val == 1)
                                        with self.state_lock: 
                                            self.current_pump_state = pump_val
                                        
                                        # ✅ Cập nhật cả đèn báo và nút gạt trên GUI
                                        self.app.after(0, lambda s=is_on: self.app.update_indicator(C.INDICATOR_PUMP, s))
                                        if hasattr(self.app, 'set_pump_state'):
                                            self.app.after(0, lambda s=is_on: self.app.set_pump_state(s))

                                    # 8. Servo
                                    servo_x100 = int(p[8])
                                    servo_angle = servo_x100 / 100.0
                                    
                                    if abs(servo_angle - (cache["servo_angle"] or -999)) > 0.1:
                                        cache["servo_angle"] = servo_angle
                                        with self.state_lock: self.current_servo_angle = servo_angle
                                        self.app.after(0, lambda a=servo_angle: self.app.set_servo_angle(a))

                                    # 9. Tray Sensor
                                    tray_val = int(p[9])
                                    is_tray = (tray_val == 1)
                                    if is_tray != cache["is_tray"]:
                                        cache["is_tray"] = is_tray
                                        self.is_tray_present = is_tray
                                        self.app.after(0, lambda s=is_tray: self.app.update_indicator(C.INDICATOR_TRAY_SENSOR, s))
                                        
                                        # ✅ XỬ LÝ THAY ĐỔI TRẠNG THÁI KHAY
                                        if is_tray:
                                            # Khay vừa được bỏ vào
                                            self.app.after(0, lambda: self.app.log_message("✅ Đã nhận khay hứng", "info"))
                                            # ✅ UPDATE LED: Có thể sáng nếu đủ điều kiện
                                            self.app.after(0, self.update_hardware_button_leds)
                                        else:
                                            # Khay vừa bị lấy ra
                                            self.app.after(0, lambda: self.app.log_message("⚠️ Mất tín hiệu khay hứng!", "warning"))
                                            
                                            # Nếu đang chạy Auto mà mất khay -> DỪNG KHẨN CẤP
                                            if self.is_running_auto:
                                                self.app.after(0, lambda: self.app.log_message("🛑 DỪNG KHẨN CẤP: Mất khay khi đang chạy Auto!", "error"))
                                                self.auto_controller.stop()
                                            
                                            # ✅ UPDATE LED: Tắt đèn START vì mất khay
                                            self.app.after(0, self.update_hardware_button_leds)

                                    # 10. Steps -> Coordinates (CHANGE DETECTION)
                                    steps_str = p[10]
                                    if ',' in steps_str:
                                        s_parts = steps_str.split(',')
                                        if len(s_parts) == 3:
                                            s1, s2, s3 = int(s_parts[0]), int(s_parts[1]), int(s_parts[2])
                                            
                                            d_theta = self.kinematics.steps_to_angles(s1, s2, s3)
                                            theta = (
                                                C.HOME_DEFAULT_THETA_DEG + d_theta[0], 
                                                C.HOME_DEFAULT_THETA_DEG + d_theta[1], 
                                                C.HOME_DEFAULT_THETA_DEG + d_theta[2]
                                            )
                                            
                                            # Calculate FK
                                            alpha = self._get_kinematics_alpha(servo_angle)
                                            coords = self.kinematics.forward_kinematics_tool(theta[0], theta[1], theta[2], alpha)
                                            
                                            if coords:
                                                # ✅ CHỈ CẬP NHẬT NẾU VỊ TRÍ THAY ĐỔI (> 0.05mm)
                                                last_c = cache["coords"]
                                                dist_sq = (coords[0]-last_c[0])**2 + (coords[1]-last_c[1])**2 + (coords[2]-last_c[2])**2 if last_c[0] is not None else 999
                                                
                                                if dist_sq > 0.0025: # 0.05mm ^ 2
                                                    cache["coords"] = coords
                                                    cache["theta"] = theta
                                                    with self.state_lock: self.current_theta = list(theta)
                                                    
                                                    # Tránh conflict khi đang Jog
                                                    if self.state != C.RobotState.JOGGING:
                                                        self._update_virtual_coords(coords[0], coords[1], coords[2], theta=theta, update_gui=True)
                                
                            except Exception as e:
                                self.app.after(0, lambda: self.app.log_message(f"STATUS parse error: {e}", "error"))
                                
                            except Exception as e:
                                self.app.after(0, lambda: self.app.log_message(f"STATUS parse error: {e}", "error"))
        
                    elif msg_type == "ESTOP_TRIGGERED":
                        if len(parts) > 1:
                            try:
                                data = json.loads(parts[1])
                                steps = data.get("s", [0, 0, 0])
                                angle_x100 = data.get("a", 0)
                                pump_state = data.get("b", 0)
                                
                                d_theta1, d_theta2, d_theta3 = self.kinematics.steps_to_angles(*steps)
                                
                                theta1 = C.HOME_DEFAULT_THETA_DEG + d_theta1
                                theta2 = C.HOME_DEFAULT_THETA_DEG + d_theta2
                                theta3 = C.HOME_DEFAULT_THETA_DEG + d_theta3
                                
                                # STM32 gửi về góc Physical (0-270) -> Convert sang Coordinate (-225 đến 45)
                                servo_angle_phys = angle_x100 / 100.0
                                servo_angle_coord = servo_angle_phys - 225.0
                                
                                alpha = self._get_kinematics_alpha(servo_angle_coord)
                                
                                coords = self.kinematics.forward_kinematics_tool(theta1, theta2, theta3, alpha)
                                
                                
                                with self.state_lock:
                                    # ✅ FIX: Giữ trạng thái ESTOP để chặn AUTO
                                    # Nhưng vẫn cho phép Manual vì can_execute đã được mở khóa cho state ESTOP
                                    self.state = C.RobotState.ESTOP  
                                    self.current_theta = [theta1, theta2, theta3]
                                    self.current_servo_angle = servo_angle_coord # Lưu Coordinate
                                    self.current_pump_state = pump_state
                                    self.jog_stop_requested = False
                                
                                if coords:
                                    x, y, z = coords
                                    estop_theta = [theta1, theta2, theta3]
                                    self._update_virtual_coords(x, y, z, theta=estop_theta)
                                    self.app.after(0, lambda x=x, y=y, z=z: 
                                        self.app.log_message(f"🛑 E-Stop: Dừng khẩn cấp tại ({x:.1f}, {y:.1f}, {z:.1f})", "warning"))
                                else:
                                    self.app.after(0, lambda: self.app.log_message("🛑 E-Stop: Dừng (Lỗi tính toán vị trí)", "warning"))
        
                                # ✅ Reset toàn bộ hàng đợi để tránh lệnh cũ ùa xuống
                                with self.pending_moves_lock: self.pending_moves.clear()
                                with self.pending_blocks_lock: 
                                    self.pending_blocks_queue.clear()
                                    self.blocks_in_flight = 0
                                    self.current_trajectory_block_index = 0
                                with self.operation_lock:
                                    self.active_move_info.clear()
                                    self.active_jog_info = None
                                with self.slots_lock:
                                     self.available_slots = self.MAX_QUEUE_SIZE
                                     
                                self.set_auto_mode(False)
                                
                                # ✅ Tự động bật Manual Mode để người dùng xử lý sự cố
                                self.app.after(0, lambda: self.app.set_manual_mode(True))
                                self.app.after(0, lambda: self.app.log_message("⚠️ Chuyển sang chế độ MANUAL để xử lý.", "info"))
                                
                                # Không bật đèn ERROR để tránh gây hiểu nhầm là hệ thống bị khóa
                                self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_ERROR, False))
                                self.app.after(0, self.update_hardware_button_leds)
                            except Exception as e:
                                self.app.after(0, lambda: self.app.log_message(f"ESTOP data parse error: {e}", "error"))

        except Exception as e:
            print(f"[ERROR handle_stm_message] {e}")
            self.app.after(0, lambda: self.app.log_message(f"ERROR processing STM message: {e}", "error"))
            import traceback
            traceback.print_exc()

    def _send_home_command_after_flush(self):
        """Hàm nội bộ để gửi lệnh HOME và cài đặt timeout."""
        print(f"[DEBUG HOME] _send_home_command_after_flush() called | homing_state={self.homing_state}")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] _send_home_command_after_flush called\n")
        
        self.app.after(0, lambda: self.app.log_message("📤 Gửi lệnh HOME đến STM32...", "sent"))
        print(f"[DEBUG HOME] Calling _send_command('HOME')... | is_connected={self.conn_manager.is_connected()}")
        success = self._send_command("HOME")
        print(f"[DEBUG HOME] _send_command returned: {success}")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] _send_command('HOME') returned: {success}\n")
        
        if not success:
            self.homing_state = "NOT_HOMED"
            with self.state_lock:
                self.state = C.RobotState.IDLE
            self.app.after(0, lambda: self.app.log_message("❌ Không thể gửi lệnh HOME (không kết nối?)", "error"))
        else:
            self.app.after(0, lambda: self.app.log_message("✅ Lệnh HOME đã được gửi tới STM32", "received"))
            self.app.after(0, lambda: self.app.log_message("⏳ Chờ STM32 phản hồi HOME_DONE... (timeout 15s)", "info"))
            print("[DEBUG HOME] Lệnh HOME đã gửi, chờ HOME_DONE...")
            
            # Cài đặt timeout cho HOME_DONE
            def homing_timeout():
                if self.homing_state == "IN_PROGRESS":
                    print("[DEBUG HOME] TIMEOUT! Không nhận được HOME_DONE sau 15s")
                    self.homing_state = "NOT_HOMED"
                    with self.state_lock:
                        self.state = C.RobotState.IDLE
                    self.app.log_message("⏱️ HOME timeout - STM32 không phản hồi HOME_DONE trong 15s", "error")
                    self.app.log_message("💡 Kiểm tra: 1) Endstop có hoạt động? 2) Motor có chạy? 3) Firmware có gửi HOME_DONE?", "error")
                    self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, False))
            
            # Timeout 15 giây cho quá trình Homing
            self.app.after(15000, homing_timeout)
            
            self.app.after(15000, homing_timeout)
            
    def home_robot(self):
        """Bắt đầu quá trình homing."""
        print(f"[DEBUG HOME] home_robot() called | current_state={self.state}, homing_state={self.homing_state}")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] home_robot() called | state={self.state}, homing_state={self.homing_state}\n")
        
        # ✅ Kiểm tra state
        can_exec = self.can_execute("home")
        print(f"[DEBUG HOME] can_execute('home') = {can_exec} | state={self.state}")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] can_execute('home') = {can_exec}\n")
        
        if not can_exec:
            self.app.log_message("⚠️ Robot đang bận, không thể homing", "error")
            print(f"[DEBUG HOME] FAILED: can_execute returned False")
            with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                f.write(f"[{time.strftime('%H:%M:%S')}] FAILED: can_execute returned False\n")
            return False
        
        print(f"[DEBUG HOME] Checking homing_state: {self.homing_state}")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] homing_state check: {self.homing_state}\n")
        
        if self.homing_state == "IN_PROGRESS":
            self.app.log_message("⚠️ Homing đã đang chạy, vui lòng chờ...", "error")
            with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                f.write(f"[{time.strftime('%H:%M:%S')}] FAILED: homing already IN_PROGRESS\n")
            return False
        
        print(f"[DEBUG HOME] Checking waiting_flag: {self.waiting_for_flush_ack_before_homing}")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] waiting_flag check: {self.waiting_for_flush_ack_before_homing}\n")
        
        if self.waiting_for_flush_ack_before_homing:
            self.app.log_message("⚠️ Đang chờ ACK:FLUSH từ lần trước...", "error")
            with open("D:/debug_home.log", "a", encoding="utf-8") as f:
                f.write(f"[{time.strftime('%H:%M:%S')}] FAILED: still waiting for FLUSH ACK\n")
            return False

        print("[DEBUG HOME] All checks passed, proceeding with HOME")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] All checks passed, setting state to HOMING\n")
        
        # ✅ CRITICAL: Đặt cờ chờ NGAY ĐẦU TIÊN - trước cả việc set state
        self.waiting_for_flush_ack_before_homing = True
        print(f"[DEBUG HOME] Set waiting_flag=True (BEFORE state change)")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] Set waiting_flag=True\n")
        
        # ✅ Chuyển state
        with self.state_lock:
            # Nếu đang ESTOP, giữ nguyên trạng thái ESTOP để chặn Auto sau khi Home xong
            if self.state != C.RobotState.ESTOP:
                self.state = C.RobotState.HOMING
        
        self.homing_state = "IN_PROGRESS"
        # ✅ FRIENDLY LOG: Log HOME bắt đầu
        self.app.log_message("🏠 HOME bắt đầu...", "sent")
        
        # ✅ FIX: Reset servo về góc -90° (góc mặc định HOME) TRƯỚC khi gửi FLUSH
        self.set_servo(-90.0)
        
        print(f"[DEBUG HOME] Now sending FLUSH_BUFFER")
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] Sending FLUSH_BUFFER...\n")
        
        # Gửi FLUSH
        self.send_flush_buffer()
        
        with open("D:/debug_home.log", "a", encoding="utf-8") as f:
            f.write(f"[{time.strftime('%H:%M:%S')}] FLUSH_BUFFER sent, returning True\n")
        
        return True

    def start_jogging(self, axis, direction):
        """Bắt đầu jog liên tục theo axis và direction."""
        try:
            if not self.can_execute("jog"):
                # ✅ FIX: Dùng app.after để log từ thread khác
                self.app.after(0, lambda: self.app.log_message("⚠️ Không thể jog lúc này", "error"))
                return False
            
            with self.state_lock:
                self.state = C.RobotState.JOGGING
                self.current_jog_axis = axis
                self.current_jog_direction = direction
                self.jog_stop_requested = False
                self.jog_at_limit = False  # ✅ Reset cờ limit khi bắt đầu JOG mới
                self.jog_limit_logged = False # ✅ Reset cờ log
                self.jog_pending_count = 0 # Reset counter
                
                # ✅ NEW: Khởi tạo tracker từ vị trí hiện tại để tính toán các bước JOG tiếp theo (Queuing)
                current_coords = self.get_virtual_coords()
                if current_coords:
                    self.jog_target_tracker = {
                        "coords": (current_coords['x'], current_coords['y'], current_coords['z']),
                        "theta": tuple(self.current_theta)
                    }
                else:
                    self.jog_target_tracker = None
            
            # ✅ FRIENDLY LOG: Lưu thông tin JOG và log 1 lần duy nhất
            direction_str = "+" if direction > 0 else "-"
            with self.operation_lock:
                self.active_jog_info = {"axis": axis.upper(), "direction": direction_str}
            self.app.after(0, lambda a=axis.upper(), d=direction_str: 
                          self.app.log_message(f"🕹️ JOG {a}{d} bắt đầu (Buffered)", "sent"))            
            
            # ✅ FIX: Pre-fill buffer - Gửi nhiều lệnh liên tiếp để tránh robot bị khựng (Starvation)
            sent_count = 0
            for _ in range(self.MAX_JOG_PENDING):
                if not self._send_jog_command():
                    break
                sent_count += 1
            
            if sent_count == 0:
                # Không gửi được lệnh nào - IK fail ngay từ đầu
                with self.state_lock:
                    self.state = C.RobotState.IDLE
                return False
                
            return True
        except Exception as e:
            # ✅ FIX: Dùng app.after để log từ thread khác
            self.app.after(0, lambda err=str(e): self.app.log_message(f"❌ Lỗi start_jogging: {err}", "error"))
            import traceback
            traceback.print_exc()
            with self.state_lock:
                self.state = C.RobotState.IDLE
            return False

    def stop_jogging(self):
        """Dừng jog - gửi FLUSH_AFTER_CURRENT để xóa queue nhưng giữ lại lệnh đang chạy."""
        # ✅ FRIENDLY LOG: Log khi dừng JOG
        hit_limit = False
        with self.operation_lock:
            if self.active_jog_info:
                axis = self.active_jog_info["axis"]
                direction = self.active_jog_info["direction"]
                with self.state_lock:
                    hit_limit = self.jog_limit_logged
                if hit_limit:
                    self.app.after(0, lambda a=axis, d=direction: 
                                  self.app.log_message(f"⚠️ JOG {a}{d} đạt giới hạn workspace", "error"))
                else:
                    self.app.after(0, lambda a=axis, d=direction: 
                                  self.app.log_message(f"🛑 JOG {a}{d} dừng", "received"))
                self.active_jog_info = None
        
        with self.state_lock:
            if self.state == C.RobotState.JOGGING:
                self.jog_stop_requested = True
                self.jog_at_limit = False  # ✅ Reset cờ limit
                self.jog_limit_logged = False # ✅ Reset cờ log
                self.jog_pending_count = 0  # ✅ Reset pending count
                self.jog_target_tracker = None # ✅ Reset tracker
                self.command_id_counter = 0 # ✅ Reset ID lệnh khi dừng JOG theo yêu cầu
        
        # ✅ PERFORMANCE FIX: Cập nhật tọa độ chỉ 1 lần thay vì 3 lần riêng lẻ
        coords = self.get_virtual_coords()
        if coords:
            self._update_virtual_coords(coords['x'], coords['y'], coords['z'], update_gui=True)
        
        # ✅ NEW: Gửi FLUSH_AFTER_CURRENT thay vì FLUSH
        # Lệnh này xóa queue nhưng để lệnh hiện tại chạy nốt
        self._send_command("FLUSH_AFTER_CURRENT")

    def _send_jog_command(self):
        """Gửi một lệnh jog đơn lẻ."""
        # ✅ Kiểm tra xem có nên dừng không
        with self.state_lock:
            if self.jog_stop_requested:
                # Nếu đã request stop, không gửi thêm lệnh nữa
                return False
            
            # ✅ Kiểm tra có đang ở giới hạn workspace không
            if self.jog_at_limit:
                # Đã đạt giới hạn, không gửi lệnh mới
                return False
            
            axis = self.current_jog_axis
            direction = self.current_jog_direction
        
        if not axis or not direction:
            return False
        
        # ✅ FIX: Thử gửi lệnh, nếu thất bại (queue đầy) thì return False
        # Không crash, chỉ bỏ qua và thử lại sau
        return self.jog_step(axis, direction)

    def jog_step(self, axis, direction):
        try:
            # ✅ USE TRACKER: Dùng vị trí dự kiến (future) thay vì vị trí hiện tại (current)
            # Để đảm bảo các lệnh trong queue nối tiếp nhau chính xác
            start_coords = None
            start_theta = None
            
            with self.state_lock:
                if self.jog_target_tracker:
                    start_coords = self.jog_target_tracker["coords"]
                    start_theta = self.jog_target_tracker["theta"]
                else:
                    # Fallback (chỉ xảy ra ở lệnh đầu tiên nếu tracker chưa init - không nên xảy ra)
                    virtual = self.get_virtual_coords()
                    if virtual:
                        start_coords = (virtual['x'], virtual['y'], virtual['z'])
                        start_theta = tuple(self.current_theta)

            if start_coords is None:
                # ✅ FIX: Log chỉ 1 lần khi bắt đầu JOG, không log trong mỗi jog_step
                # self.app.after(0, lambda: self.app.log_message("❌ JOG: Cần HOME trước", "error"))
                with self.state_lock:
                    self.state = C.RobotState.IDLE
                return False

            # ✅ PERFORMANCE FIX: Tăng quãng đường mỗi bước để giảm tần suất gửi lệnh
            # 5mm/bước với 1000Hz -> 75ms/lệnh -> quá nhanh, dễ gây starvation
            # 10mm/bước -> 150ms/lệnh -> an toàn hơn
            JOG_DISTANCE_MM = 5.0

            current_xt, current_yt, current_zt = start_coords
            
            # print(f"DEBUG JOG: Planning from ({current_xt:.1f}, {current_yt:.1f}, {current_zt:.1f})")

            target_xt = current_xt + (JOG_DISTANCE_MM * direction if axis == 'x' else 0)
            target_yt = current_yt + (JOG_DISTANCE_MM * direction if axis == 'y' else 0)
            target_zt = current_zt + (JOG_DISTANCE_MM * direction if axis == 'z' else 0)

            with self.state_lock:
                current_servo_angle = self.current_servo_angle
                current_alpha_deg = self._get_kinematics_alpha(current_servo_angle)
                
                # Sử dụng start_theta từ tracker
                current_angles = start_theta
            
            current_steps = self.kinematics.angles_to_steps(*current_angles)
            
            # Tính target angles
            target_angles = self.kinematics.inverse_kinematics_tool(
                target_xt, target_yt, target_zt, current_alpha_deg
            )
            
            if target_angles is None:
                # ✅ FIX: Khi IK fail (ra ngoài vùng làm việc), KHÔNG dừng JOG hoàn toàn
                with self.state_lock:
                    if not self.jog_at_limit:
                        # ✅ PERFORMANCE: Chỉ set flag, log sẽ được xử lý trong stop_jogging
                        self.jog_limit_logged = True
                        self.jog_at_limit = True
                return False

            target_steps = self.kinematics.angles_to_steps(*target_angles)

            # ✅ CORRECT REALITY: Tính lại tọa độ thực tế từ số bước xung đã làm tròn
            # Giúp hiển thị GUI trung thực giống như lệnh MOVE
            actual_theta = self.kinematics.steps_to_angles(*target_steps)
            actual_coords = self.kinematics.forward_kinematics_tool(
                actual_theta[0], actual_theta[1], actual_theta[2], current_alpha_deg
            )
            
            # Fallback nếu FK fail (hiếm gặp)
            if actual_coords is None:
                actual_coords = (target_xt, target_yt, target_zt)

            delta_steps = [
                target_steps[0] - current_steps[0],
                target_steps[1] - current_steps[1],
                target_steps[2] - current_steps[2]
            ]
            
            # Tính thời gian dựa trên tốc độ jog (RPM)
            max_steps = max(abs(s) for s in delta_steps)
            if max_steps == 0:
                return False
            
            # Hz = (RPM * Steps/Rev) / 60
            # [UPDATE] Làm tròn step/s theo yêu cầu
            jog_speed_hz = round((C.JOG_SPEED_RPM * C.MOTOR_STEPS_PER_REV) / 60.0)
            jog_time = max_steps / jog_speed_hz
            
            with self.state_lock:
                # ✅ OPTIMIZATION: Gửi thẳng góc tọa độ (Coordinate System)
                angle_coord = self.current_servo_angle
                # ✅ VALIDATION: Giới hạn góc tọa độ [-225, 45]
                angle_coord = max(-225.0, min(45.0, angle_coord))
                params = {
                    "t": jog_time,
                    "s": delta_steps,
                    "a": angle_coord,
                    "b": self.current_pump_state
                }
                
                # ✅ UPDATE TRACKER: Cập nhật vị trí THỰC TẾ (Quantized) cho lệnh tiếp theo
                self.jog_target_tracker = {
                    "coords": actual_coords,
                    "theta": tuple(actual_theta)
                }
            
            # Target position để cập nhật GUI khi DONE (Hiển thị số lẻ thực tế)
            target_position = {
                "coords": actual_coords,
                "theta": list(actual_theta)
            }
            
            block_id = self.get_next_command_id()
            # ✅ FIX: blocking=False để tránh treo GUI khi JOG nhanh
            send_result = self.send_add_block(params, block_id=block_id, target_position=target_position, blocking=False)
            
            if send_result is None:
                # Gửi thất bại (queue đầy) -> KHÔNG update tracker?
                # Thực tế nếu queue đầy, ta nên giữ tracker ở vị trí cũ để thử lại
                # Nhưng logic hiện tại là return False và caller sẽ quyết định
                # Nếu return False, start_jogging sẽ break loop
                return False
            
            # Thành công - tăng pending count
            with self.state_lock:
                self.jog_pending_count += 1
            return True
        except Exception as e:
            # ✅ PERFORMANCE: Chỉ print error, không gọi app.after để tránh đơ GUI
            error_msg = f"❌ Lỗi jog_step: {str(e)}"
            print(error_msg)
            import traceback
            traceback.print_exc()
            with self.state_lock:
                self.state = C.RobotState.IDLE
            return False

    def append_sliced_segment(self, start_pos, end_pos, start_angles, servo_angle_coord, pump_state):
        """
        Helper: Tính toán và thêm các blocks cắt lát vào hàng đợi pending.
        Trả về (final_coords, final_angles) để dùng cho đoạn tiếp theo.
        """
        # Validate góc servo
        servo_angle_coord = max(-225.0, min(45.0, servo_angle_coord))
        alpha_deg = self._get_kinematics_alpha(servo_angle_coord)
        
        # Plan trajectory
        plan = self.planner_trapezoidal.plan_cartesian_move_time_sliced(
            start_cartesian=start_pos,
            end_cartesian=end_pos,
            alpha_deg=alpha_deg,
            kinematics=self.kinematics,
            current_angles=start_angles
        )
        
        if plan is None: return None, None
        if not plan: return start_pos, start_angles
        
        # Lấy metadata từ block cuối
        last_block = plan[-1]
        if "_final_angles_actual" in last_block:
            target_angles = last_block["_final_angles_actual"]
        else:
            # Fallback IK
            target_angles = self.kinematics.inverse_kinematics_tool(end_pos[0], end_pos[1], end_pos[2], alpha_deg)
            if target_angles is None: target_angles = start_angles
            
        # Tính tọa độ thực tế từ góc thực tế
        actual_final_coords = self.kinematics.forward_kinematics_tool(
            target_angles[0], target_angles[1], target_angles[2], alpha_deg
        )
        if actual_final_coords is None: actual_final_coords = end_pos
        
        # Đưa vào queue
        with self.pending_blocks_lock:
            for idx, block_params in enumerate(plan):
                block_params = block_params.copy()
                block_params.pop("_final_angles_actual", None)
                block_params.pop("_final_steps_actual", None)
                
                block_params["a"] = servo_angle_coord
                block_params["b"] = pump_state
                
                is_last_block = (idx == len(plan) - 1)
                
                target_position = None
                move_info = None
                
                if is_last_block:
                    target_position = {
                        "coords": actual_final_coords,
                        "theta": list(target_angles)
                    }
                    move_info = {
                        "target": actual_final_coords,
                        "total_blocks": len(plan) # Thông tin này chỉ đúng cho segment lẻ
                    }
                
                block_info = {
                    "params": block_params,
                    "target_position": target_position,
                    "is_last": is_last_block,
                    "move_info": move_info
                }
                self.pending_blocks_queue.append(block_info)
                
        return actual_final_coords, tuple(target_angles)

    def move_to_coords(self, xt, yt, zt):
        """Di chuyển đến tọa độ chỉ định theo đường THẲNG với S-Curve mượt mà.
        
        Sử dụng Time-based Cartesian Slicing:
        - Chia thời gian thành các time slices nhỏ (15ms)
        - Tại mỗi slice: Tính S(t) từ S-curve → Tọa độ (x,y,z) → IK → Steps
        - Đảm bảo quỹ đạo thực tế là đường thẳng + chuyển động mượt mà
        
        Args:
            xt, yt, zt: Tọa độ đích
        
        Returns:
            True nếu lệnh được gửi thành công, False nếu có lỗi
        """
        try:
            # ✅ Kiểm tra state TRƯỚC
            if not self.can_execute("move"):
                self.app.log_message("⚠️ Robot đang bận hoặc chưa sẵn sàng để di chuyển", "error")
                return False
            
            current_coords = self.get_virtual_coords()
            if current_coords is None:
                self.app.after(0, lambda: self.app.log_message("Lỗi: Cần Homing trước khi di chuyển đến tọa độ.", "error"))
                return False
            
            x0, y0, z0 = current_coords['x'], current_coords['y'], current_coords['z']
            
            # ✅ DEBUG: Log vị trí hiện tại
            self.app.log_message(
                f"📍 Vị trí hiện tại: ({x0:.1f}, {y0:.1f}, {z0:.1f})", 
                "info")
            
            with self.state_lock:
                current_servo_angle = self.current_servo_angle
                current_angles = tuple(self.current_theta)
                current_pump = self.current_pump_state

            # ✅ Chuyển state sang MOVING
            with self.state_lock:
                self.state = C.RobotState.MOVING
                self.last_move_command_time = time.time()
            
            # ✅ SLIDING WINDOW: Reset queue
            with self.pending_blocks_lock:
                self.pending_blocks_queue.clear()
                self.blocks_in_flight = 0
                self.current_trajectory_block_index = 0
            
            # ✅ Generate blocks using Helper
            final_coords, final_angles = self.append_sliced_segment(
                (x0, y0, z0), (xt, yt, zt), 
                current_angles, current_servo_angle, current_pump
            )
            
            if final_coords is None:
                self.app.log_message(f"Lỗi IK: Không thể đến ({xt:.1f}, {yt:.1f}, {zt:.1f})", "error")
                with self.state_lock: self.state = C.RobotState.IDLE
                return False

            total_distance = math.sqrt((xt-x0)**2 + (yt-y0)**2 + (zt-z0)**2)
            self.app.log_message(
                f"🎯 MOVE đến ({xt:.1f}, {yt:.1f}, {zt:.1f}) - {total_distance:.1f}mm", 
                "sent")
            
            # Gửi batch đầu tiên (32 blocks)
            self._send_blocks_batch(is_initial=True)
            
            return True
            
        except Exception as e:
            # ✅ CATCH ALL: Log mọi lỗi xảy ra để debug
            import traceback
            traceback.print_exc()
            error_msg = f"❌ Critical Error in move_to_coords: {str(e)}"
            self.app.log_message(error_msg, "error")
            
            # Reset state để tránh treo
            with self.state_lock:
                self.state = C.RobotState.IDLE
            return False

    def set_pump(self, state):
        # ✅ LOGIC ĐÚNG cho Relay Active Low:
        # GUI: True (Bật) → Gửi b=1 → STM32 ghi GPIO_PIN_RESET (0V) → Relay ON
        # GUI: False (Tắt) → Gửi b=0 → STM32 ghi GPIO_PIN_SET (3.3V) → Relay OFF
        # Code STM32: block->pump_state ? GPIO_PIN_RESET : GPIO_PIN_SET
        
        # ✅ DEBOUNCE: Tránh spam pump commands
        current_time = time.time() * 1000  # Convert to milliseconds
        if current_time - self.last_pump_command_time < self.PUMP_DEBOUNCE_MS:
            self.app.log_message(
                f"⚠️ Pump: Ấn quá nhanh! Vui lòng đợi {self.PUMP_DEBOUNCE_MS}ms giữa các lần bật/tắt.", 
                "warning")
            return False
        self.last_pump_command_time = current_time
        
        new_pump_state = 1 if state else 0
        
        with self.state_lock:
            old_state = self.current_pump_state
            self.current_pump_state = new_pump_state
            
            # ✅ OPTIMIZATION: Gửi thẳng góc tọa độ (Coordinate System)
            angle_coord = self.current_servo_angle
            # ✅ VALIDATION: Giới hạn góc tọa độ [-225, 45]
            angle_coord = max(-225.0, min(45.0, angle_coord))
            params = {
                "t": 0.1,
                "s": [0, 0, 0],
                "a": angle_coord,
                "b": self.current_pump_state
            }
        
        state_str = "BẬT" if state else "TẮT"
        self.app.log_message(f"💨 Pump: {state_str} (b={self.current_pump_state})", "sent")
        
        # ✅ Cập nhật indicator bơm trên GUI
        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_PUMP, state))
        
        result = self.send_add_block(params, block_id=0)
        
        if result is None:
            self.app.log_message("❌ Pump: Gửi lệnh thất bại (queue đầy?)", "error")
            with self.state_lock:
                self.current_pump_state = old_state
            return False
        return True

    def set_servo(self, angle_coord, block_id=None):
        """Đặt góc servo theo HỆ TỌA ĐỘ. 
        
        Args:
            angle_coord: Góc servo theo hệ tọa độ (-225 đến 45)
            block_id: ID của block (optional, default=0 = fire-and-forget)
        
        Returns:
            True nếu thành công, False nếu có lỗi
        """
        # ✅ VALIDATION: Giới hạn góc tọa độ [-225, 45]
        angle_coord = max(-225.0, min(45.0, angle_coord))
        
        # ✅ OPTIMIZATION: Gửi góc tọa độ trực tiếp, STM32 sẽ chuyển đổi
        # Không cần chuyển đổi ở PC nữa: physical = coord + 225 (được làm ở STM32)
        
        # ✅ FIX BUG PUMP: Servo KHÔNG được thay đổi pump state
        # Luôn gửi current_pump_state (giữ nguyên trạng thái pump hiện tại)
        with self.state_lock:
            params = { "t": 0.1, "s": [0, 0, 0], "a": angle_coord, "b": self.current_pump_state }
            # Cập nhật current_servo_angle (Góc tọa độ) để tracking
            self.current_servo_angle = angle_coord
        
        # ✅ Mặc định dùng block_id=0 (fire-and-forget) nếu không chỉ định
        effective_block_id = block_id if block_id is not None else 0
        
        # ✅ FRIENDLY LOG: Log góc tọa độ
        self.app.log_message(f"🔧 Servo: Set Coord {angle_coord:.1f}°", "sent")
        
        send_result = self.send_add_block(params, block_id=effective_block_id)
        
        if send_result is None:
            # Gửi thất bại
            self.app.log_message(f"❌ Servo: Gửi lệnh thất bại (queue đầy?)", "error")
            return False

        # ✅ UPDATE STATE & GUI:
        # Khi servo xoay, đầu hút (Tool Tip) di chuyển theo cung tròn (thay đổi X, Y)
        # Cần tính lại FK để cập nhật tọa độ hiển thị chính xác
        
        # Lưu góc TỌA ĐỘ vào state nội bộ
        with self.state_lock:
            self.current_servo_angle = angle_coord
            current_theta = self.current_theta # Lấy góc khớp hiện tại
            
        # Tính alpha mới từ góc tọa độ
        new_alpha = self._get_kinematics_alpha(angle_coord)
        
        # Tính FK mới cho đầu hút
        new_coords = self.kinematics.forward_kinematics_tool(
            current_theta[0], current_theta[1], current_theta[2], new_alpha
        )
        
        if new_coords:
            x, y, z = new_coords
            self._update_virtual_coords(x, y, z, theta=current_theta)
            # ✅ Cập nhật servo angle lên GUI (theo hệ tọa độ)
            self.app.after(0, lambda a=angle_coord: self.app.set_servo_angle(a))

        return True

    def set_auto_mode(self, is_running):
        print(f"[DEBUG] set_auto_mode called: is_running={is_running}")
        if is_running:
            self.last_move_command_time = time.time() # ✅ NEW: Đánh dấu thời điểm bắt đầu để tránh Sync lỗi
            self.auto_controller.start()
        else:
            self.auto_controller.stop()
        print(f"[DEBUG] set_auto_mode completed")