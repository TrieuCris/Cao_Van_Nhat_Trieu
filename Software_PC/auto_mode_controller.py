import threading
import queue
import time
import math
from enum import Enum, auto
import constants as C

class AutoState(Enum):
    IDLE = auto()              # Robot chờ, chưa chạy AUTO
    MOVING_TO_WAIT = auto()    # Đang di chuyển đến vị trí chờ
    SCANNING = auto()          # Băng tải chạy, camera quét vật
    INTERCEPTING = auto()      # Robot đang di chuyển đến điểm gắp
    PICKING = auto()           # Robot đang hạ xuống hút + nhấc lên
    DROPPING = auto()          # Robot đang di chuyển đến vị trí thả + thả

class AutoModeController:
    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.app = self.robot.app
        
        # --- AUTO MODE STATE ---
        self.auto_state = AutoState.IDLE
        self.is_running_auto = False
        self.last_auto_sequence_id = None
        
        # --- TIME-BASED SYNCHRONIZATION ---
        # Thời điểm (perf_counter) mà robot dự kiến sẽ hoàn thành xong nhiệm vụ hiện tại
        # Nếu robot đang rảnh, giá trị này <= time.perf_counter()
        self.robot_next_free_perf = 0.0
        
        # Vị trí (x, y, z) mà robot sẽ ở tại thời điểm robot_next_free_perf
        self.future_robot_pos = self.robot.wait_position
        
        # --- QUEUES & THREADS ---
        self.auto_objects_queue = queue.Queue(maxsize=2)
        self.auto_thread = threading.Thread(target=self._auto_thread_loop, daemon=True)
        
        self.pick_execution_queue = queue.Queue()
        self.pick_execution_thread = threading.Thread(target=self._pick_execution_loop, daemon=True)
        
        # --- TRACKING & SCHEDULING ---
        self.processed_ids = set()
        self.triggered_objects = set()
        self.pick_candidates = []           # List chứa các vật đã trigger
        self.candidate_lock = threading.Lock()
        self.schedule_lock = threading.RLock()
        
        self.current_picking_obj_id = None
        
        # --- CONFIG ---
        self.trigger_robot_y = -240.0  # Vị trí trigger trên hệ tọa độ robot
        # self.conveyor_speed_mm_s = 40.0 # REMOVED: Dùng property trỏ sang robot_controller
        self.conveyor_direction = 1    # 1: Y tăng, -1: Y giảm
        
        # Grid Search Parameters (Vùng gắp cho phép)
        self.y_pick_min = -90
        self.y_pick_max = 75
        self.y_pick_step = 2
        
        # Start threads
        self.auto_thread.start()
        self.pick_execution_thread.start()

    @property
    def conveyor_speed_mm_s(self):
        return self.robot.conveyor_speed_mm_s

    @conveyor_speed_mm_s.setter
    def conveyor_speed_mm_s(self, value):
        self.robot.conveyor_speed_mm_s = float(value)
        
    def start(self):
        """Khởi động chế độ Auto"""
        if self.is_running_auto:
            self.app.log_message("⚠️ Chế độ Auto đã đang chạy", "warning")
            return
        
        # Kiểm tra kết nối STM32 trước
        if not self.robot.conn_manager.is_connected():
            self.app.log_message("❌ Chưa kết nối STM32", "error")
            self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_AUTO, False))
            return
        
        # Thêm delay nhỏ để tránh dính lệnh với thao tác trước đó
        time.sleep(0.1)

        # ✅ FORCE UPDATE TRAY STATUS
        # Chủ động hỏi STM32 trạng thái hiện tại (trường hợp khay đã có sẵn từ trước)
        self.robot._request_status()
        # Chờ phản hồi STATUS từ STM32 (chứa bit tray_sensor)
        # Cần wait cứng ở đây vì start() chạy trên MainThread, 
        # nhưng 0.3s là chấp nhận được để đảm bảo an toàn.
        start_wait = time.time()
        while time.time() - start_wait < 0.3:
            self.app.process_events() # Giữ GUI không bị đơ
            time.sleep(0.01)

        # ✅ CHECK TRAY SENSOR (Kiểm tra 2 lần để chắc chắn)
        if not self.robot.is_tray_present:
            self.app.log_message("⚠️ Vui lòng bỏ khay vào để chạy Auto!", "warning")
            self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_AUTO, False))
            return

        # ✅ BẬT AUTO MODE
        self.is_running_auto = True
        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_AUTO, True))
        
        # ✅ UPDATE LED: Tắt đèn START vì auto đã bắt đầu
        self.robot.update_hardware_button_leds()
        
        # ✅ DOUBLE-CHECK TRAY SAU KHI BẬT (Phòng trường hợp race condition)
        time.sleep(0.05)
        if not self.robot.is_tray_present:
            self.app.log_message("❌ Mất khay ngay sau khi bật Auto!", "error")
            self.is_running_auto = False
            self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_AUTO, False))
            self.robot.update_hardware_button_leds()  # Update lại đèn
            return
        
        self.app.log_message("🚀 BẮT ĐẦU AUTO (TIME-BASED): Về vị trí chờ...", "sent")
        # self.robot._send_command("AUTO_MODE:1") # Firmware không hỗ trợ lệnh này
        
        # Reset State
        self.auto_state = AutoState.IDLE
        self.processed_ids.clear()
        self.triggered_objects.clear()
        with self.candidate_lock:
            self.pick_candidates.clear()
        with self.auto_objects_queue.mutex:
            self.auto_objects_queue.queue.clear()
        with self.pick_execution_queue.mutex:
            self.pick_execution_queue.queue.clear()
            
        self.current_picking_obj_id = None
        
        # Reset Time Base
        self.robot_next_free_perf = time.perf_counter()
        self.future_robot_pos = self.robot.wait_position
        
        # Move to Wait Position
        self.auto_state = AutoState.MOVING_TO_WAIT
        x, y, z = self.robot.wait_position
        if not self.robot.move_to_coords(x, y, z):
            self.app.log_message("❌ Lỗi: Không thể về vị trí chờ", "error")
            self.stop()

    def stop(self):
        """Dừng chế độ Auto"""
        if not self.is_running_auto:
            return

        self.is_running_auto = False
        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_AUTO, False))
        self.app.log_message("🛑 AUTO STOP REQUESTED", "sent")
        
        self.auto_state = AutoState.IDLE
        self.robot_next_free_perf = 0
        
        # ✅ Clear Internal Queues immediately
        with self.auto_objects_queue.mutex:
            self.auto_objects_queue.queue.clear()
        with self.pick_execution_queue.mutex:
            self.pick_execution_queue.queue.clear()
        with self.candidate_lock:
            self.pick_candidates.clear()
        self.triggered_objects.clear()
        self.processed_ids.clear()
        
        # self.robot._send_command("AUTO_MODE:0") # Firmware không hỗ trợ
        self.robot.send_conveyor_stop()
        self.robot.send_flush_buffer()
        # ✅ REMOVED: CLEAR_QUEUE - Trùng chức năng với FLUSH_BUFFER
        # FLUSH_BUFFER đã được gọi ở dòng trên, không cần CLEAR_QUEUE nữa
        
        # ✅ RESET SERVO về góc -90° (góc mặc định) khi dừng Auto
        self.robot.set_servo(-90.0)

        # ✅ FORCE RE-HOME: Yêu cầu Home lại sau khi Stop để tránh sai số vị trí
        self.robot.homing_state = "NOT_HOMED"
        self.app.after(0, lambda: self.app.update_indicator(C.INDICATOR_HOME_OK, False))
        self.app.log_message("⚠️ STOP: Yêu cầu HOME lại để đảm bảo chính xác vị trí.", "warning")
        
        # ✅ UPDATE LED: Cập nhật đèn nút START (tắt đi vì auto đã stop)
        self.robot.update_hardware_button_leds()

    def _auto_thread_loop(self):
        """Thread xử lý nhận diện vật từ Camera"""
        while True:
            try:
                tracked_objects = self.auto_objects_queue.get(timeout=1.0)
                if self.is_running_auto:
                    self.process_objects(tracked_objects)
            except queue.Empty:
                continue
            except Exception as e:
                self.app.after(0, lambda err=e: self.app.log_message(f"❌ AutoThread Error: {err}", "error"))
                time.sleep(1)

    def _pick_execution_loop(self):
        """Thread thực thi lệnh gắp (Busy Wait & Send Command)"""
        while True:
            try:
                task = self.pick_execution_queue.get(timeout=1.0)
                if self.is_running_auto:
                    self._execute_time_based_task(task)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"CRITICAL ERROR in _pick_execution_loop: {e}")
                time.sleep(1)

    def process_objects(self, tracked_objects):
        """
        Nhận diện vật và ghi lại thời gian Trigger chính xác (Perf Counter).
        Đây là bước quan trọng nhất để đồng bộ thời gian.
        """
        if not self.is_running_auto or not tracked_objects:
            return
        if self.auto_state != AutoState.SCANNING:
            return
        
        # --- MEMORY CLEANUP ---
        # Giới hạn kích thước triggered_objects để tránh memory leak
        if len(self.triggered_objects) > 1000:
            # Giữ lại 800 ID mới nhất (giả sử ID tăng dần)
            sorted_ids = sorted(list(self.triggered_objects))
            self.triggered_objects = set(sorted_ids[-800:])
        
        # 1. LẤY MỐC THỜI GIAN NGAY LẬP TỨC
        # ✅ FIX: Không lấy thời gian hiện tại nữa, dùng thời gian từ Capture Time
        # current_perf = time.perf_counter() 

        for obj in tracked_objects:
            # Format mới: (obj_id, (bbox, timestamp, metadata))
            # Format cũ: (obj_id, (bbox, timestamp)) - tương thích ngược
            if len(obj[1]) == 3:
                obj_id, (bbox, obj_capture_time, metadata) = obj
            else:
                obj_id, (bbox, obj_capture_time) = obj
                metadata = {}
            
            if obj_id in self.triggered_objects:
                continue
            
            # Convert px -> mm
            try:
                # Nếu có center từ metadata, dùng nó (chính xác hơn)
                if metadata.get('center'):
                    x_center_px, y_center_px = metadata['center']
                else:
                    x_center_px = int(bbox[0] + bbox[2] / 2)
                    y_center_px = int(bbox[1] + bbox[3] / 2)
                
                result = self._convert_pixel_to_robot_coords(x_center_px, y_center_px)
                if not result: continue
                robot_x, robot_y = result
            except:
                continue
            
            # TRIGGER LOGIC UPDATE: Sử dụng Trigger Window (Cửa sổ Trigger)
            # Chỉ chấp nhận vật nằm trong khoảng hẹp ngay sau vạch Trigger
            # Để đảm bảo vật được detect tại nơi có ánh sáng tốt nhất (Trigger Line)
            TRIGGER_WINDOW = 30.0 # mm - Độ rộng vùng chấp nhận trigger
            
            trigger_condition = False
            if self.conveyor_direction > 0:
                # Băng tải đi lên: Trigger <= Y <= Trigger + Window
                trigger_condition = (self.trigger_robot_y <= robot_y <= self.trigger_robot_y + TRIGGER_WINDOW)
            else:
                # Băng tải đi xuống: Trigger - Window <= Y <= Trigger
                trigger_condition = (self.trigger_robot_y - TRIGGER_WINDOW <= robot_y <= self.trigger_robot_y)
            
            if trigger_condition:
                print(f"[DEBUG AUTO] Trigger condition met for Obj {obj_id} at Y={robot_y:.1f}")
                # Check Duplicate (Debounce) - CHỐNG RUNG/NHIỄU
                # Nếu camera rung, tracker có thể mất dấu cũ và tạo ID mới cho cùng 1 vật.
                # Ta kiểm tra khoảng cách giữa vật mới này với vị trí dự đoán của các vật đang track.
                is_duplicate = False
                with self.candidate_lock:
                    for cand in self.pick_candidates:
                        # Dự đoán vị trí hiện tại của cand dựa trên thời gian trôi qua
                        # ✅ FIX: Dùng obj_capture_time thay vì current_perf
                        dt = obj_capture_time - cand['trigger_perf']
                        pred_y = cand['trigger_y'] + (self.conveyor_speed_mm_s * dt * self.conveyor_direction)
                        
                        # Tính khoảng cách Euclidean (Khoảng cách tâm)
                        dist = math.sqrt((cand['robot_x'] - robot_x)**2 + (pred_y - robot_y)**2)
                        
                        # Nếu gần hơn 30mm thì coi là nhiễu/trùng lặp
                        if dist < 30.0:
                            is_duplicate = True
                            # self.app.log_message(f"⚠️ Ignored Ghost Obj{obj_id} (Dist={dist:.1f}mm to Obj{cand['obj_id']})", "info")
                            break
                
                if is_duplicate:
                    # ✅ BUG FIX: Thêm lock để tránh race condition
                    if obj_id not in self.triggered_objects:
                        self.triggered_objects.add(obj_id)
                    continue

                # ✅ BUG FIX: Thêm lock để tránh race condition
                # Kiểm tra lại sau khi ra khỏi candidate_lock để tránh duplicate
                if obj_id in self.triggered_objects:
                    continue
                self.triggered_objects.add(obj_id)
                
                # ✅ KIỂM TRA: Chỉ gắp nếu thuộc 4 loại được hỗ trợ
                class_name = metadata.get('class_name', "Unknown")
                drop_pos = self._get_drop_position(class_name)
                if drop_pos is None:
                    # Không thuộc 4 loại -> bỏ qua, không gắp
                    # ✅ Đánh dấu status "discarded" trong tracker
                    if hasattr(self.robot, 'tracker') and self.robot.tracker:
                        self.robot.tracker.set_status(obj_id, "discarded")
                    # self.app.log_message(f"⏭️ Obj{obj_id} ({class_name}) không thuộc 4 loại hỗ trợ - Bỏ qua", "info")
                    continue
                
                # TẠO CANDIDATE VỚI TIME BASE CHUẨN + METADATA HÌNH CHỮ NHẬT
                candidate = {
                    'obj_id': obj_id,
                    'robot_x': robot_x,
                    'trigger_y': robot_y,       # Y tại thời điểm trigger
                    'trigger_perf': obj_capture_time, # ✅ FIX: Dùng Capture Time chính xác
                    'scheduled': False,
                    'angle': metadata.get('angle', 0),  # Góc lệch (degrees)
                    'class_name': class_name,
                    'confidence': metadata.get('confidence', 0.0)
                }
                
                with self.candidate_lock:
                    self.pick_candidates.append(candidate)
                
                # self.app.log_message(f"👁️ Detect Obj{obj_id} ({candidate['class_name']}) tại T={current_perf:.2f}", "info")
                
                # Gọi Scheduler lập lịch ngay
                threading.Thread(target=self._schedule_pick_candidates, daemon=True).start()

    def _schedule_pick_candidates(self):
        """
        LẬP LỊCH DỰA TRÊN THỜI GIAN (TIME-BASED SCHEDULER)
        Mục tiêu: Tìm Y_PICK sao cho Robot đến nơi ĐÚNG lúc vật trôi tới.
        """
        if not self.schedule_lock.acquire(blocking=False):
            return
        
        try:
            while True:
                # 1. Lấy candidate chưa được schedule
                candidate = None
                with self.candidate_lock:
                    # Lọc sạch các candidate cũ nát hoặc lỗi
                    self.pick_candidates = [c for c in self.pick_candidates if not c.get('discard', False)]
                    
                    for cand in self.pick_candidates:
                        if not cand.get('scheduled', False):
                            candidate = cand
                            break
                
                if not candidate:
                    break # Hết việc
                
                # ✅ BUG FIX: Kiểm tra lại scheduled sau khi ra khỏi lock
                # (Tránh race condition nếu có thread khác đã schedule candidate này)
                with self.candidate_lock:
                    if candidate.get('scheduled', False):
                        # Candidate đã được schedule bởi thread khác, quay lại tìm candidate khác
                        continue
                
                # 2. Chuẩn bị dữ liệu tính toán
                obj_id = candidate['obj_id']
                t_trigger = candidate['trigger_perf']
                trigger_y = candidate['trigger_y']
                robot_x = candidate['robot_x']
                
                # Thời điểm hiện tại
                now_perf = time.perf_counter()
                
                # Thời điểm sớm nhất robot có thể bắt đầu hành động mới
                # Là MAX của (Bây giờ, Thời điểm robot xong việc cũ)
                robot_available_at = max(now_perf, self.robot_next_free_perf)
                
                best_plan = None
                
                # 3. Grid Search: Quét các điểm Y từ Min đến Max để tìm điểm khớp thời gian
                # Chúng ta ưu tiên điểm nào gắp sớm nhất có thể (để giải phóng robot sớm)
                
                # Xác định hướng duyệt loop tùy theo hướng băng tải
                y_range = range(self.y_pick_min, self.y_pick_max + 1, self.y_pick_step)
                
                # Góc servo cho giai đoạn PICK: -90 độ (coord)
                pick_servo_coord = -90.0
                
                for pick_y in y_range:
                    # A. Tính Time Arrival (Vật đến pick_y lúc nào?)
                    dist_obj_travel = pick_y - trigger_y
                    
                    # Nếu điểm gắp ngược chiều băng tải -> Bỏ
                    if (self.conveyor_direction > 0 and dist_obj_travel < 0) or \
                       (self.conveyor_direction < 0 and dist_obj_travel > 0):
                        continue
                        
                    time_travel = abs(dist_obj_travel) / self.conveyor_speed_mm_s
                    t_arrival = t_trigger + time_travel
                    
                    # Nếu vật đã trôi qua điểm này trong quá khứ -> Bỏ
                    if t_arrival < now_perf:
                        continue
                    
                    # B. Tính Robot Time (Robot mất bao lâu để đến pick_y?)
                    # Xuất phát từ vị trí tương lai (nơi robot kết thúc task trước)
                    start_pos = self.future_robot_pos 
                    target_pos_safe = (robot_x, pick_y, self.robot.z_safe)
                    target_pos_pick = (robot_x, pick_y, self.robot.z_pick)
                    
                    # Quyết định có đi tắt (Shortcut) hay qua Wait
                    # Logic: Kiểm tra an toàn độ cao Z
                    is_shortcut = self._is_shortcut_safe(start_pos, target_pos_safe)
                    
                    t_robot_move = 0.0
                    
                    # Tính toán thời gian Hạ xuống (Pick Down)
                    # Di chuyển từ Z_Safe -> Z_Pick với góc pick_servo_coord
                    t_down = self._simulate_robot_move_time(target_pos_safe, target_pos_pick, pick_servo_coord)
                    if t_down is None:
                        self.app.log_message("❌ Critical: Motion Plan Failed! Cannot calculate path time (Pick Down).", "error")
                        self.stop()
                        break
                    
                    if is_shortcut:
                        # Shortcut: Bay thẳng từ vị trí cũ đến vị trí gắp (Safe Z)
                        t_move = self._simulate_robot_move_time(start_pos, target_pos_safe, pick_servo_coord)
                        if t_move is None:
                            self.app.log_message("❌ Critical: Motion Plan Failed! Cannot calculate path time (Shortcut).", "error")
                            self.stop()
                            break
                        t_robot_move = t_move + t_down
                        via_wait = False
                    else:
                        # Via Wait: Về Home/Wait rồi mới ra gắp
                        t_to_wait = self._simulate_robot_move_time(start_pos, self.robot.wait_position, pick_servo_coord)
                        t_wait_to_pick = self._simulate_robot_move_time(self.robot.wait_position, target_pos_safe, pick_servo_coord)
                        
                        if t_to_wait is None or t_wait_to_pick is None:
                            self.app.log_message("❌ Critical: Motion Plan Failed! Cannot calculate path time (Via Wait).", "error")
                            self.stop()
                            break
                            
                        t_robot_move = t_to_wait + t_wait_to_pick + t_down
                        via_wait = True
                    
                    # C. Tính Required Start Time (Lúc nào robot phải xuất phát?)
                    LATENCY_COMPENSATION = 0.25
                    t_required_start = t_arrival - t_robot_move - LATENCY_COMPENSATION
                    
                    # D. KIỂM TRA TÍNH KHẢ THI (Crucial Step)
                    # Robot phải xuất phát SAU khi nó rảnh
                    # Cho phép sai số nhỏ (0.0s) để bù trừ lag
                    if t_required_start >= robot_available_at - 0.1:
                        
                        # Tính thời gian phải chờ (Idle time)
                        wait_duration = t_required_start - now_perf
                        
                        # Chấp nhận plan này!
                        best_plan = {
                            'pick_y': pick_y,
                            'via_wait': via_wait,
                            't_arrival': t_arrival,         # Target Clock
                            't_start_action': t_required_start, # Start Clock
                            't_robot_move': t_robot_move,
                            'wait_duration': wait_duration
                        }
                        break # Tìm thấy điểm hợp lệ đầu tiên -> Chốt luôn (First Fit)
                
                # 4. Xử lý kết quả lập lịch
                if best_plan:
                    # ✅ BUG FIX: Kiểm tra lại scheduled trước khi đánh dấu (double-check)
                    with self.candidate_lock:
                        if candidate.get('scheduled', False):
                            # Candidate đã được schedule bởi thread khác, bỏ qua plan này
                            continue
                        candidate['scheduled'] = True
                    
                    # ✅ Đánh dấu status trong tracker
                    if hasattr(self.robot, 'tracker') and self.robot.tracker:
                        self.robot.tracker.set_status(obj_id, "scheduled")
                    
                    candidate['plan'] = best_plan
                    
                    # --- TÍNH TOÁN CYCLE DURATION CHÍNH XÁC ---
                    # Cycle: Start -> Pick Down (đã tính trong t_robot_move) -> Pick Up -> Move Drop -> Drop Down -> Release -> Drop Up
                    
                    # Xác định vị trí thả dựa trên phân loại
                    drop_pos = self._get_drop_position(candidate.get('class_name', "Unknown"))
                    if drop_pos is None:
                        # Không thuộc 4 loại -> bỏ qua (Phòng thủ - Không nên xảy ra vì đã check ở process_objects)
                        with self.candidate_lock:
                            candidate['discard'] = True
                        # ✅ Đánh dấu status trong tracker
                        if hasattr(self.robot, 'tracker') and self.robot.tracker:
                            self.robot.tracker.set_status(obj_id, "discarded")
                        continue
                    drop_x, drop_y, _ = drop_pos
                    
                    # Xác định góc Servo cho giai đoạn thả (DROP)
                    # Nếu vật bị xoay, robot sẽ xoay servo ngay trong lúc Pick Up
                    # Góc servo khi thả = góc đã xoay vật (nếu có), giữ nguyên góc vật khi thả
                    rectangle_angle = candidate.get('angle', 0)
                    target_servo_coord = self._calculate_servo_angle_for_rectangle(rectangle_angle)
                    servo_angle_diff = abs(target_servo_coord - pick_servo_coord)
                    drop_servo_coord = pick_servo_coord  # Mặc định = -90°
                    if servo_angle_diff > 0.1:  # Có xoay servo đáng kể
                        drop_servo_coord = target_servo_coord  # Dùng góc đã xoay

                    # 1. Pick Down: Thêm thời gian pump hút vật (0.3s)
                    t_pump_suck = 0.3
                    
                    # 2. Pick Up: Từ Z_Pick lên Z_Safe
                    pick_up_start = (robot_x, best_plan['pick_y'], self.robot.z_pick)
                    pick_up_end = (robot_x, best_plan['pick_y'], self.robot.z_safe)
                    pick_up_dist = abs(pick_up_end[2] - pick_up_start[2])  # Vertical move
                    
                    t_up = 0.0
                    if pick_up_dist >= 3.0:  # Distance Filter
                        t_up = self._simulate_robot_move_time(pick_up_start, pick_up_end, pick_servo_coord)
                        if t_up is None:
                            self.app.log_message("❌ Critical: Motion Plan Failed! Cannot calculate path time (Pick Up).", "error")
                            self.stop()
                            continue
                        if t_up < 0.1: t_up = 0.1  # Min time clamp

                    # 3. Move to Drop: Từ vị trí gắp sang vị trí thả (servo xoay đồng thời)
                    to_drop_start = (robot_x, best_plan['pick_y'], self.robot.z_safe)
                    to_drop_end = (drop_x, drop_y, self.robot.z_safe)
                    to_drop_dist = math.sqrt((to_drop_end[0]-to_drop_start[0])**2 + (to_drop_end[1]-to_drop_start[1])**2)
                    is_servo_rotate = (servo_angle_diff > 1.0)
                    
                    t_to_drop = 0.0
                    if to_drop_dist >= 3.0 or is_servo_rotate:  # Distance Filter + Servo exception
                        t_to_drop = self._simulate_robot_move_time(to_drop_start, to_drop_end, drop_servo_coord)
                        if t_to_drop is None:
                            self.app.log_message("❌ Critical: Motion Plan Failed! Cannot calculate path time (To Drop).", "error")
                            self.stop()
                            continue
                        if t_to_drop < 0.1: t_to_drop = 0.1  # Min time clamp
                        
                    # 5. Drop Down: Từ Z_Safe xuống Z_Pick (giữ nguyên drop_servo_coord)
                    drop_down_start = (drop_x, drop_y, self.robot.z_safe)
                    drop_down_end = (drop_x, drop_y, self.robot.z_pick)
                    drop_down_dist = abs(drop_down_end[2] - drop_down_start[2])  # Vertical move
                    
                    t_drop_down = 0.0
                    if drop_down_dist >= 3.0:  # Distance Filter
                        t_drop_down = self._simulate_robot_move_time(drop_down_start, drop_down_end, drop_servo_coord)
                        if t_drop_down is None:
                            self.app.log_message("❌ Critical: Motion Plan Failed! Cannot calculate path time (Drop Down).", "error")
                            self.stop()
                            continue
                        if t_drop_down < 0.1: t_drop_down = 0.1  # Min time clamp
                    
                    # 6. Release Pump (Cố định do giới hạn vật lý bơm)
                    t_release = 0.3 
                    
                    # 7. Drop Up: Từ Z_Pick lên Z_Safe (giữ nguyên góc servo đã xoay)
                    drop_up_start = (drop_x, drop_y, self.robot.z_pick)
                    drop_up_end = (drop_x, drop_y, self.robot.z_safe)
                    drop_up_dist = abs(drop_up_end[2] - drop_up_start[2])  # Vertical move
                    
                    t_drop_up = 0.0
                    if drop_up_dist >= 3.0:  # Distance Filter
                        t_drop_up = self._simulate_robot_move_time(drop_up_start, drop_up_end, drop_servo_coord)
                        if t_drop_up is None:
                            self.app.log_message("❌ Critical: Motion Plan Failed! Cannot calculate path time (Drop Up).", "error")
                            self.stop()
                            continue
                        if t_drop_up < 0.1: t_drop_up = 0.1  # Min time clamp

                    # Tổng thời gian robot bận (servo xoay đồng thời khi di chuyển, không cần thời gian riêng)
                    # ✅ SYNC: Distance Filter đã được áp dụng giống như execution
                    cycle_duration = best_plan['t_robot_move'] + t_pump_suck + t_up + t_to_drop + t_drop_down + t_release + t_drop_up
                    
                    # Cập nhật Global State cho vật tiếp theo biết
                    self.robot_next_free_perf = best_plan['t_start_action'] + cycle_duration
                    self.future_robot_pos = (drop_x, drop_y, self.robot.z_safe)
                    
                    # msg = f"📅 Plan Obj{obj_id}: Y={best_plan['pick_y']}, Start T={best_plan['t_start_action']:.2f} (Wait {best_plan['wait_duration']:.2f}s)"
                    # self.app.log_message(msg, "info")
                    
                    # Đẩy sang hàng đợi thực thi
                    self.pick_execution_queue.put(candidate)
                    
                else:
                    # Không tìm thấy điểm gắp nào khả thi (vật trôi quá nhanh hoặc robot quá bận)
                    # self.app.log_message(f"⏭️ Obj{obj_id} quá hạn/không kịp gắp. Bỏ qua.", "warning")
                    candidate['discard'] = True # Đánh dấu để xóa
                    # ✅ Đánh dấu status trong tracker
                    if hasattr(self.robot, 'tracker') and self.robot.tracker:
                        self.robot.tracker.set_status(obj_id, "discarded")
        
        finally:
            self.schedule_lock.release()

    def _execute_time_based_task(self, candidate):
        """
        Thực thi nhiệm vụ với độ chính xác thời gian cao (Precision Wait)
        Đã tối ưu CPU Usage.
        """
        plan = candidate['plan']
        t_start = plan['t_start_action']
        obj_id = candidate['obj_id']
        
        # 1. PRECISION WAIT LOOP (Optimized)
        while True:
            if not self.is_running_auto: return
            
            now = time.perf_counter()
            remaining = t_start - now
            
            if remaining <= 0:
                break # Đã đến giờ G!
            
            # Nếu còn > 5ms thì sleep nhẹ để nhường CPU cho Serial Thread
            if remaining > 0.005: 
                time.sleep(0.001) 
            else:
                pass # Busy wait trong 5ms cuối để chính xác tuyệt đối
        
        # 2. CHECK TRỄ
        latency = time.perf_counter() - t_start
        if latency > 0.15:
            self.app.log_message(f"⚠️ High Latency: Start trễ {latency*1000:.1f}ms", "warning")
            
        # 3. EXECUTE
        self.current_picking_obj_id = obj_id
        pick_y = plan['pick_y']
        pick_x = candidate['robot_x']
        via_wait = plan['via_wait']
        
        # Log thông tin
        # log_msg = f"🚀 Execute Obj{obj_id} tại Y={pick_y}"
        # if candidate.get('angle') is not None:
        #      log_msg += f" (Góc lệch: {candidate.get('angle'):.1f}°)"
        # self.app.log_message(log_msg, "sent")
        
        self._send_pick_commands(pick_x, pick_y, via_wait, plan, candidate)

    def _send_pick_commands(self, pick_x, pick_y, via_wait, plan, candidate=None):
        """
        Gửi chuỗi lệnh block xuống RobotController.
        
        ✅ CẢI TIẾN MỚI:
        - Di chuyển NGANG (INTERCEPT, TO_DROP): Cắt lát + nội suy quỹ đạo (nhiều blocks, mượt)
        - Di chuyển ĐỨNG (PICK_DOWN/UP, DROP_DOWN/UP): 1 block constant velocity (chậm, an toàn)
        """
        try:
            # Xác định vị trí thả dựa trên phân loại
            drop_pos = self._get_drop_position(candidate.get('class_name', "Unknown") if candidate else "Unknown")
            if drop_pos is None:
                return
            drop_x, drop_y, _ = drop_pos
            
            # ===== ĐỊNH NGHĨA GÓC SERVO =====
            default_pick_servo_coord = -90.0
            rectangle_angle = candidate.get('angle', 0.0) if candidate else 0.0
            target_servo_coord = self._calculate_servo_angle_for_rectangle(rectangle_angle)
            
            servo_angle_diff = abs(target_servo_coord - default_pick_servo_coord)
            final_servo_coord = default_pick_servo_coord
            if servo_angle_diff > 0.1:
                final_servo_coord = target_servo_coord
            
            # ===== EXECUTION =====
            final_block_id = None
            
            with self.robot.state_lock:
                current_theta = list(self.robot.current_theta)
                current_servo_angle = self.robot.current_servo_angle
                current_pump = self.robot.current_pump_state
            
            # Tính tọa độ hiện tại (FK)
            current_pos = self.robot.kinematics.forward_kinematics_tool(
                current_theta[0], current_theta[1], current_theta[2], 
                self.robot._get_kinematics_alpha(current_servo_angle))
            
            if current_pos is None:
                self.app.log_message("❌ Critical: FK Start Error", "error")
                self.stop()
                return
            
            # ===== SEGMENT 1: TO_WAIT (nếu cần) =====
            if via_wait:
                current_pos, current_theta = self._send_horizontal_sliced(
                    current_pos, self.robot.wait_position, 
                    current_theta, current_servo_angle, 0,
                    AutoState.MOVING_TO_WAIT, "TO_WAIT"
                )
                if current_pos is None:
                    return
            
            # ===== SEGMENT 2: INTERCEPT (Ngang - Cắt lát) =====
            intercept_pos = (pick_x, pick_y, self.robot.z_safe)
            current_pos, current_theta = self._send_horizontal_sliced(
                current_pos, intercept_pos,
                current_theta, default_pick_servo_coord, 0,
                AutoState.INTERCEPTING, "INTERCEPT"
            )
            if current_pos is None:
                return
            
            # ===== SEGMENT 3: PICK_DOWN (Đứng - 1 block chậm) =====
            pick_down_pos = (pick_x, pick_y, self.robot.z_pick)
            blk_id = self._send_vertical_single(
                current_pos, pick_down_pos,
                current_theta, default_pick_servo_coord, 1,  # pump ON
                AutoState.PICKING, "PICK_DOWN"
            )
            if blk_id:
                final_block_id = blk_id
                current_pos = pick_down_pos
                # Cập nhật theta
                alpha = self.robot._get_kinematics_alpha(default_pick_servo_coord)
                angles = self.robot.kinematics.inverse_kinematics_tool(*pick_down_pos, alpha)
                if angles:
                    current_theta = list(angles)
            
            # ===== SEGMENT 4: PICK_UP (Đứng - 1 block chậm) =====
            pick_up_servo = final_servo_coord if servo_angle_diff > 0.1 else default_pick_servo_coord
            pick_up_pos = (pick_x, pick_y, self.robot.z_safe)
            blk_id = self._send_vertical_single(
                current_pos, pick_up_pos,
                current_theta, pick_up_servo, 1,  # pump ON
                AutoState.PICKING, "PICK_UP"
            )
            if blk_id:
                final_block_id = blk_id
                current_pos = pick_up_pos
                # Cập nhật theta
                alpha = self.robot._get_kinematics_alpha(pick_up_servo)
                angles = self.robot.kinematics.inverse_kinematics_tool(*pick_up_pos, alpha)
                if angles:
                    current_theta = list(angles)
            
            # ===== SEGMENT 5: TO_DROP (Ngang - Cắt lát) =====
            drop_safe_pos = (drop_x, drop_y, self.robot.z_safe)
            current_pos, current_theta = self._send_horizontal_sliced(
                current_pos, drop_safe_pos,
                current_theta, final_servo_coord, 1,  # pump ON
                AutoState.DROPPING, "TO_DROP"
            )
            if current_pos is None:
                return
            
            # ===== SEGMENT 6: DROP_DOWN (Đứng - 1 block chậm) =====
            drop_down_pos = (drop_x, drop_y, self.robot.z_pick)
            blk_id = self._send_vertical_single(
                current_pos, drop_down_pos,
                current_theta, final_servo_coord, 0,  # pump OFF
                AutoState.DROPPING, "DROP_DOWN"
            )
            if blk_id:
                final_block_id = blk_id
                current_pos = drop_down_pos
                # Cập nhật theta
                alpha = self.robot._get_kinematics_alpha(final_servo_coord)
                angles = self.robot.kinematics.inverse_kinematics_tool(*drop_down_pos, alpha)
                if angles:
                    current_theta = list(angles)
            
            # ===== SEGMENT 7: DROP_UP (Đứng - 1 block chậm) =====
            drop_up_pos = (drop_x, drop_y, self.robot.z_safe)
            blk_id = self._send_vertical_single(
                current_pos, drop_up_pos,
                current_theta, default_pick_servo_coord, 0,  # pump OFF
                AutoState.SCANNING, "DROP_UP"
            )
            if blk_id:
                final_block_id = blk_id
            
            self.last_auto_sequence_id = str(final_block_id)
            
        except Exception as e:
            self.app.log_message(f"❌ Send Command Error: {e}", "error")
    
    def _send_horizontal_sliced(self, start_pos, end_pos, start_theta, servo_coord, pump_state, state, desc):
        """
        Gửi lệnh di chuyển NGANG với cắt lát + nội suy quỹ đạo (nhiều blocks).
        
        Returns:
            (final_pos, final_theta) nếu thành công, (None, None) nếu lỗi
        """
        # Distance filter
        dist = math.sqrt((end_pos[0]-start_pos[0])**2 + (end_pos[1]-start_pos[1])**2 + (end_pos[2]-start_pos[2])**2)
        if dist < 3.0:
            return start_pos, start_theta  # Bỏ qua micro-move
        
        # Dùng append_sliced_segment để cắt lát
        final_pos, final_theta = self.robot.append_sliced_segment(
            start_pos, end_pos, start_theta, servo_coord, pump_state
        )
        
        if final_pos is None:
            self.app.log_message(f"❌ IK Error at {desc}", "error")
            self.stop()
            return None, None
        
        # Gửi batch blocks
        with self.robot.pending_blocks_lock:
            # Đánh dấu block cuối cùng để update state
            if self.robot.pending_blocks_queue:
                last_block = self.robot.pending_blocks_queue[-1]
                last_block['auto_state'] = state
                last_block['auto_desc'] = desc
        
        # Gửi batch (không delay giữa blocks trong AUTO - đã có delay trong _send_blocks_batch)
        num_blocks = len(self.robot.pending_blocks_queue) if hasattr(self.robot, 'pending_blocks_queue') else 0
        with self.robot.pending_blocks_lock:
            blocks_to_send = list(self.robot.pending_blocks_queue)
            
        for block_info in blocks_to_send:
            # ✅ STOP CHECK: Nếu đã stop, không gửi thêm block nào nữa
            if not self.is_running_auto:
                return None, None

            params = block_info["params"]
            target_position = block_info["target_position"]
            
            block_id = self.robot.get_next_command_id()
            blk_id = self.robot.send_add_block(params, block_id=block_id, target_position=target_position, source="AUTO")
            
            if blk_id and block_info.get('auto_state'):
                self.auto_state = block_info['auto_state']
        
        # Clear pending queue
        with self.robot.pending_blocks_lock:
            self.robot.pending_blocks_queue.clear()
        
        return final_pos, final_theta
    
    def _send_vertical_single(self, start_pos, end_pos, start_theta, servo_coord, pump_state, state, desc):
        """
        Gửi lệnh di chuyển ĐỨNG với 1 block duy nhất (constant velocity chậm).
        
        Returns:
            block_id nếu thành công, None nếu lỗi
        """
        # Distance filter
        dist = abs(end_pos[2] - start_pos[2])
        if dist < 1.0:
            return None  # Bỏ qua micro-move
        
        # Tính IK
        alpha = self.robot._get_kinematics_alpha(servo_coord)
        target_angles = self.robot.kinematics.inverse_kinematics_tool(
            end_pos[0], end_pos[1], end_pos[2], alpha
        )
        
        if target_angles is None:
            self.app.log_message(f"❌ IK Error at {desc}", "error")
            self.stop()
            return None
        
        # Tính delta steps
        current_steps = self.robot.kinematics.angles_to_steps(*start_theta)
        target_steps = self.robot.kinematics.angles_to_steps(*target_angles)
        delta_steps = [target_steps[k] - current_steps[k] for k in range(3)]
        
        # Tính thời gian - CHẬM hơn (giảm RPM cho constant velocity)
        # Dùng 100 RPM thay vì 300 RPM (giảm 3 lần)
        VERTICAL_SPEED_RPM = 100.0
        max_steps = max(abs(s) for s in delta_steps)
        if max_steps == 0:
            return None
        
        speed_hz = (VERTICAL_SPEED_RPM * self.robot.kinematics.steps_per_revolution) / 60.0
        t_segment = max_steps / speed_hz
        
        if t_segment < 0.1:
            t_segment = 0.1
        
        # Tạo params
        params = {
            "t": t_segment,
            "s": delta_steps,
            "a": servo_coord,
            "b": pump_state
        }
        
        target_pos_dict = {"coords": end_pos, "theta": target_angles}
        
        # Gửi block
        block_id = self.robot.get_next_command_id()
        blk_id = self.robot.send_add_block(params, block_id=block_id, target_position=target_pos_dict, source="AUTO")
        
        if blk_id:
            self.auto_state = state
        
        return blk_id

    # --- HELPERS ---

    def handle_done_message(self, done_id):
        """Xử lý khi robot báo hoàn thành Block"""
        if not self.is_running_auto: return

        if self.auto_state == AutoState.MOVING_TO_WAIT:
             self.start_conveyor_scanning()
        
        elif self.last_auto_sequence_id and done_id == self.last_auto_sequence_id:
            # Hoàn thành cycle gắp
            self.last_auto_sequence_id = None
            current_count = self.app.get_product_count()
            self.app.set_product_count(current_count + 1)
            self.app.log_message(f"✅ Gắp xong SP #{current_count + 1}", "received")
            
            # Dọn dẹp queue
            completed_id = self.current_picking_obj_id
            if completed_id is not None:
                # Tìm và cập nhật số lượng chi tiết trước khi xóa
                with self.candidate_lock:
                    found_cls = None
                    for c in self.pick_candidates:
                        if c['obj_id'] == completed_id:
                            found_cls = c.get('class_name')
                            break
                    
                    # Update GUI count
                    if found_cls:
                        name = str(found_cls).lower().strip()
                        idx = -1
                        if "chuoi" in name: idx = 0
                        elif "dau" in name: idx = 1
                        elif "kiwi" in name: idx = 2
                        elif "socola" in name: idx = 3
                        
                        if idx != -1:
                            # Run on main thread to be safe with GUI updates
                            # Although set_detail_count calls setText which is mostly thread-safe in PyQt signal slots?
                            # The current code calls self.app.set_product_count directly.
                            # self.app methods use signals? No, set_product_count calls setText directly.
                            # But standard PyQt usage from thread requires signals.
                            # In DeltaRobotGUI, set_product_count calls self.lbl_prod_count.setText.
                            # If auto_mode_controller runs in thread, this might be unsafe.
                            # However, existing code does it. I will follow existing pattern.
                            try:
                                cur_d = self.app.get_detail_count(idx)
                                self.app.set_detail_count(idx, cur_d + 1)
                            except: pass

                    # Xóa khỏi pick_candidates
                    self.pick_candidates = [c for c in self.pick_candidates if c['obj_id'] != completed_id]
                
                # KHÔNG Xóa khỏi triggered_objects để tránh gắp lại vật cũ (Double Pick)
                # Tracker sử dụng ID tăng dần (Unique), nên việc giữ ID trong set là đúng đắn
                # để đảm bảo mỗi vật chỉ được trigger 1 lần duy nhất trong đời.
                # if completed_id in self.triggered_objects:
                #     self.triggered_objects.remove(completed_id)
            
            self.current_picking_obj_id = None
            self.auto_state = AutoState.SCANNING
            
            # Trigger scheduler lần nữa để fill chỗ trống nếu có
            threading.Thread(target=self._schedule_pick_candidates, daemon=True).start()

    def start_conveyor_scanning(self):
        # self.app.log_message("🔄 Băng tải CHẠY. Đang quét vật...", "info")
        self.robot.send_conveyor_start(True)
        self.auto_state = AutoState.SCANNING

    def _get_drop_position(self, class_name):
        """
        Xác định vị trí thả dựa trên class_name.
        Mapping (4 class):
          - chuoi:  X=60, Y=-60
          - dau:    X=60, Y=-20
          - kiwi:   X=60, Y=40
          - socola: X=60, Y=60
        Z giữ nguyên theo robot default.
        
        Returns:
            (x, y, z) nếu thuộc 4 loại, None nếu không thuộc (không gắp)
        """
        # Default Z from robot config
        default_z = self.robot.drop_position_default[2]
        
        # Mapping logic
        # Class names: chuoi, dau, kiwi, socola
        target_x =60.0
        
        name = str(class_name).lower().strip()
        
        if "chuoi" in name:
            target_y = -92.0
        elif "dau" in name:
            target_y = -38
        elif "kiwi" in name:
            target_y = 20
        elif "socola" in name:
            target_y = 77
        else:
            # Không thuộc 4 loại -> không gắp
            return None
            
        return (target_x, target_y, default_z)

    def _convert_pixel_to_robot_coords(self, pixel_x, pixel_y):
        camera_config = getattr(self.app, 'camera_config', None)
        if camera_config and camera_config.is_calibrated():
            return camera_config.pixel_to_mm(pixel_x, pixel_y)
        return None

    def _calculate_servo_angle_for_rectangle(self, rectangle_angle):
        """
        Tính toán góc servo để xoay hình chữ nhật sao cho chiều dài trùng với trục Y.
        Hỗ trợ input rectangle_angle từ -90 đến 90 độ.
        """
        # rectangle_angle: Góc lệch từ camera (-90 đến 90)
        # 0: Ngang
        # 90 / -90: Dọc
        
        # Mục tiêu: Đưa về +/- 90 độ (Dọc)
        # Tính khoảng cách đến 90 và -90
        diff_to_90 = 90.0 - rectangle_angle
        diff_to_neg_90 = -90.0 - rectangle_angle
        
        # Chọn đường xoay ngắn nhất
        if abs(diff_to_90) < abs(diff_to_neg_90):
            target_rotation = diff_to_90
        else:
            target_rotation = diff_to_neg_90
        
        # Servo mặc định (-90 độ coord)
        default_servo_coord = -90.0
        
        # Công thức: Servo = Default + Rotation (Thử lại phép cộng sau khi fix detection)
        # Nếu detection trả về đúng góc âm/dương, phép cộng có thể mới là logic đúng
        servo_coord_angle = default_servo_coord + target_rotation
        
        # Giới hạn an toàn (-225 đến 45)
        servo_coord_angle = max(-225.0, min(45.0, servo_coord_angle))
        
        return servo_coord_angle

    def _is_shortcut_safe(self, start_pos, end_pos):
        # Kiểm tra nếu bay thẳng có bị va chạm không (dựa vào chiều cao Z)
        # Nếu đang ở thấp (Z < -400) mà bay ngang là nguy hiểm
        # Robot z_safe hiện tại là -395, nên cần hạ threshold xuống -400 để cho phép shortcut.
        MIN_SAFE_Z = -400.0
        if start_pos[2] < MIN_SAFE_Z or end_pos[2] < MIN_SAFE_Z:
            return False
        return True

    def _simulate_robot_move_time(self, from_pos, to_pos, servo_angle_coord=None):
        """
        Tính toán thời gian di chuyển dựa trên Trapezoidal Profile của Robot.
        Sử dụng hàm calc_travel_time (Cartesian + Numba) để chính xác tuyệt đối.
        
        Args:
            from_pos: Vị trí bắt đầu (x, y, z)
            to_pos: Vị trí kết thúc (x, y, z)
            servo_angle_coord: Góc servo hệ tọa độ (-225 đến 45). Nếu None, dùng góc mặc định.
        
        Returns:
            Thời gian di chuyển (giây) hoặc None nếu lỗi
        """
        try:
            # Lấy góc servo: nếu không có thì dùng góc mặc định
            if servo_angle_coord is None:
                servo_angle_coord = self.robot.servo_angle_fixed
            
            # Sử dụng hàm tính toán mới trong kinematics.py
            # Hàm này đã bao gồm logic: Trapezoidal, Motor Hz Limit, Segment Quantization
            duration = self.robot.planner_trapezoidal.calc_travel_time(from_pos, to_pos, servo_angle_coord)
            
            return duration
        except:
            return None