"""
Connection Manager - Handles asynchronous communication with STM32
Prevents UI freezing by offloading all blocking operations to worker threads
"""
import serial
import serial.tools.list_ports
import threading
import queue
import time
from constants import log_dedupe
from enum import Enum
from typing import Callable, Optional

def list_ports():
    """Liệt kê tất cả các cổng COM có sẵn."""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return ports

class STM32Communicator:
    def __init__(self):
        self.ser = None
        self.is_running = False
        self.read_thread = None
        self.listener_thread = None
        self.write_thread = None  # NEW: Thread riêng để ghi dữ liệu
        self.listeners = []
        self.listener_lock = threading.Lock()  # ✅ FIX: Thread safety for listeners list
        self.message_queue = queue.Queue()
        self.command_queue = queue.Queue()  # NEW: Queue cho các lệnh cần gửi
        self.handshake_queue = queue.Queue()  # Queue riêng cho PONG/SYS_READY khi kết nối

    def add_message_listener(self, listener):
        """Thêm một hàm callback để nhận dữ liệu từ STM32."""
        with self.listener_lock:
            if listener not in self.listeners:
                self.listeners.append(listener)

    def remove_message_listener(self, listener):
        """Xóa một hàm callback."""
        with self.listener_lock:
            if listener in self.listeners:
                self.listeners.remove(listener)

    def _notify_listeners(self, message):
        """Thông báo cho tất cả các listener đã đăng ký."""
        self.message_queue.put(message)

    def connect(self, port, baudrate=115200, timeout=2.0, max_retries=2):
        """Thiết lập kết nối serial và xác nhận STM32 sẵn sàng."""
        if self.is_connected():
            return True, "Đã kết nối."

        last_error = None
        
        # Thử kết nối với retry
        for attempt in range(max_retries):
            try:
                # Tạo kết nối serial
                self.ser = serial.Serial(
                    port, baudrate, timeout=1, write_timeout=5  # Tăng write_timeout để tránh timeout
                )
                self.is_running = True
                self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self.read_thread.start()
                
                # NEW: Bắt đầu luồng ghi dữ liệu
                self.write_thread = threading.Thread(target=self._write_loop, daemon=True)
                self.write_thread.start()
                
                # Bắt đầu luồng xử lý listener
                self.listener_thread = threading.Thread(target=self._listener_loop, daemon=True)
                self.listener_thread.start()

                # Đợi USB CDC khởi tạo xong (STM32 cần thời gian sau khi kết nối)
                time.sleep(0.5)  # Tăng lên 0.5s để chắc chắn
                
                # Flush buffers ngay để xóa dữ liệu cũ
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
                # Xóa hàng đợi cũ trước khi ping
                while not self.message_queue.empty():
                    try:
                        self.message_queue.get_nowait()
                    except queue.Empty:
                        break

                # Chờ SYS_READY hoặc gửi PING
                # Bước 1: Đợi xem có SYS_READY tự động không (trong 1 giây)
                sys_ready_received = False
                try:
                    response = self.message_queue.get(timeout=1.0)
                    print(f"[DEBUG] Nhận được từ STM32 ngay sau kết nối: '{response}'")
                    if response in ["SYS_READY", "PONG"]:
                        return True, f"✓ Kết nối thành công đến {port} (nhận {response})"
                except queue.Empty:
                    print("[DEBUG] Không nhận được SYS_READY tự động, thử gửi PING...")

                # Bước 2: Nếu không có SYS_READY, gửi PING
                ping_attempts = 5  # Tăng số lần thử
                pong_received = False
                
                for ping_try in range(ping_attempts):
                    # Flush buffer trước mỗi lần thử
                    self.ser.reset_input_buffer()
                    while not self.message_queue.empty():
                        try:
                            self.message_queue.get_nowait()
                        except queue.Empty:
                            break
                    
                    # Gửi PING trực tiếp qua serial (không qua queue để đảm bảo gửi ngay)
                    print(f"[DEBUG] Gửi PING lần {ping_try + 1}...")
                    self.ser.write(b"PING\n")
                    self.ser.flush()  # Đảm bảo dữ liệu được gửi ngay
                    
                    try:
                        # Chờ phản hồi với timeout ngắn hơn cho mỗi lần thử
                        response = self.message_queue.get(timeout=1.0)
                        print(f"[DEBUG] Nhận được phản hồi: '{response}'")
                        if response in ["PONG", "SYS_READY"]:
                            pong_received = True
                            break
                        else:
                            last_error = f"Phản hồi không đúng: {response}"
                    except queue.Empty:
                        print(f"[DEBUG] Timeout lần thử {ping_try + 1}")
                        last_error = f"Timeout chờ PONG (lần thử {ping_try + 1}/{ping_attempts})"
                        continue
                
                if pong_received:
                    # ✅ REMOVED: Không cần gửi ENABLE - STM32 tự động ENABLE khi nhận PING
                    return True, f"✓ Kết nối thành công đến {port}"
                else:
                    last_error = f"Không nhận được PONG sau {ping_attempts} lần thử"
                
                # Nếu thất bại, ngắt kết nối và thử lại
                self.disconnect()
                if attempt < max_retries - 1:
                    time.sleep(0.5)  # Đợi trước khi thử lại

            except serial.SerialException as e:
                last_error = str(e)
                self.ser = None
                if attempt < max_retries - 1:
                    time.sleep(0.5)
        
        return False, f"✗ Kết nối thất bại sau {max_retries} lần thử: {last_error}"

    def disconnect(self):
        """Ngắt kết nối serial."""
        if self.ser and self.ser.is_open:
            # ✅ FIX: Dừng thread TRƯỚC khi gửi lệnh để tránh xung đột
            self.is_running = False
            
            try:
                # Gửi lệnh RESET_SYSTEM để STM32 tự dọn dẹp (Reset Home, Stop Motor, Stop Conveyor)
                print("[DEBUG] Đang gửi RESET_SYSTEM...")
                self.ser.write(b"RESET_SYSTEM\n")
                self.ser.flush()
                time.sleep(0.1) # Đợi STM32 nhận và xử lý
                
                # Flush buffers để đảm bảo lệnh được gửi
                self.ser.reset_output_buffer()
                self.ser.reset_input_buffer()
            except Exception as e:
                print(f"[WARNING] Lỗi khi gửi lệnh cleanup: {e}")
            
            # Chờ read_thread chết (timeout 1s)
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=1.0)
            
            # Chờ write_thread chết (timeout 1s)
            if self.write_thread and self.write_thread.is_alive():
                self.write_thread.join(timeout=1.0)
            
            # Chờ listener_thread chết (timeout 1s)
            if self.listener_thread and self.listener_thread.is_alive():
                self.listener_thread.join(timeout=1.0)
            
            # Kiểm tra nếu còn thread nào chưa chết
            alive_threads = []
            if self.read_thread and self.read_thread.is_alive():
                alive_threads.append("read_thread")
            if self.write_thread and self.write_thread.is_alive():
                alive_threads.append("write_thread")
            if self.listener_thread and self.listener_thread.is_alive():
                alive_threads.append("listener_thread")
            
            if alive_threads:
                # Log warning nhưng vẫn tiếp tục đóng cổng
                print(f"[WARNING] Thread chưa chết hẳn: {', '.join(alive_threads)}")
            
            # Đóng cổng COM
            try:
                self.ser.close()
                print("[DEBUG] Đã đóng cổng COM")
            except Exception as e:
                print(f"[ERROR] Lỗi khi đóng cổng COM: {e}")
            
            self.ser = None
            
            self._notify_listeners("SYS_DISCONNECTED") 
            return True, "Đã ngắt kết nối và dọn dẹp."
        return False, "Không có kết nối nào đang hoạt động."

    def send_command(self, command):
        """Queue command to send to STM32 (async, non-blocking)."""
        if self.is_connected():
            # ✅ DEBUG: Log lệnh được queue (trừ ADD_BLOCK để tránh spam)
            # is_add_block = command.startswith("ADD_BLOCK:") or (len(command) > 0 and command[0].isdigit() and ':' in command)
            # if not is_add_block:
            #     print(f"[DEBUG send_command] Queuing command: {command}")
            # Queue command instead of blocking write
            self.command_queue.put(command)
            return True
        else:
            # print(f"[DEBUG send_command] FAILED - Not connected! Command: {command}")
            return False

    def is_connected(self):
        """Kiểm tra trạng thái kết nối."""
        return self.ser is not None and self.ser.is_open

    def _read_loop(self):
        """Vòng lặp đọc dữ liệu từ STM32 và thông báo cho các listener."""
        while self.is_running and self.is_connected():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        self.message_queue.put(line)
            except serial.SerialException:
                self._notify_listeners("ERROR:Mất kết nối phần cứng.")
                self.disconnect()
                break
            except Exception as e:
                self._notify_listeners(f"ERROR:Lỗi đọc dữ liệu: {e}")
                pass
            time.sleep(0.01) # Giảm độ trễ

    def _write_loop(self):
        """NEW: Thread loop to handle non-blocking writes."""
        while self.is_running:
            try:
                command = self.command_queue.get(timeout=0.1)
                if self.is_connected():
                    try:
                        # ✅ OPTIMIZATION: ADD_BLOCK có thể có prefix "ADD_BLOCK:" hoặc short format "id:json"
                        is_add_block = command.startswith("ADD_BLOCK:") or (len(command) > 0 and command[0].isdigit() and ':' in command)
                        
                        # ✅ DEBUG: Log lệnh trước khi gửi (trừ ADD_BLOCK để tránh spam)
                        # if not is_add_block:
                        #     print(f"[DEBUG _write_loop] Sending to serial: {command}")
                        
                        self.ser.write((command + '\n').encode('ascii'))
                        self.ser.flush()  # ✅ Đảm bảo lệnh được gửi ngay
                        
                        # ✅ DEBUG: Confirm đã gửi (trừ ADD_BLOCK)
                        # if not is_add_block:
                        #     print(f"[DEBUG _write_loop] Sent successfully: {command}")
                        
                        # ✅ FIX: Thêm delay nhỏ giữa các lệnh để tránh tràn buffer USB CDC
                        # Đặc biệt quan trọng cho ADD_BLOCK (lệnh dài)
                        if is_add_block:
                            time.sleep(0.010)  # ⚠️ 10ms delay cho ADD_BLOCK (tăng lên để an toàn với buffer 512)
                        else:
                            time.sleep(0.002)  # 2ms delay cho lệnh khác
                    except serial.SerialException as e:
                        print(f"[ERROR _write_loop] SerialException: {e}")
                        self._notify_listeners(f"ERROR:Lỗi khi gửi lệnh: {e}")
                        self.disconnect()
            except queue.Empty:
                pass
            except Exception as e:
                print(f"[ERROR _write_loop] Exception: {e}")
                pass
                # print(f"Lỗi trong write loop: {e}")

    def _listener_loop(self):
        """Vòng lặp xử lý queue và gọi các listener."""
        while self.is_running:
            try:
                message = self.message_queue.get(timeout=0.5)
                
                # Nếu đang trong quá trình handshake, chuyển PONG/SYS_READY sang handshake_queue
                # (Kiểm tra xem handshake_queue có đang được theo dõi không)
                if message.startswith("PONG") or message.startswith("SYS_READY"):
                    # Đưa vào cả 2 queue để đảm bảo cả handshake và listener đều nhận được
                    self.handshake_queue.put(message)
                
                # ✅ DEBUG: Log số lượng listener
                # with self.listener_lock:
                #     num_listeners = len(self.listeners)
                #     if message.startswith("ACK:"):
                #         print(f"[DEBUG _listener_loop] Message: '{message}', Số listeners: {num_listeners}")
                
                # ✅ FIX: Gọi các listener bình thường với thread safety
                with self.listener_lock:
                    for idx, listener in enumerate(self.listeners):
                        try:
                            # if message.startswith("ACK:"):
                            #     print(f"[DEBUG _listener_loop] Gọi listener #{idx}")
                            listener(message)
                        except Exception as e:
                            print(f"[ERROR _listener_loop] Lỗi gọi listener #{idx}: {e}")
            except queue.Empty:
                pass
            except Exception as e:
                pass
                # print(f"Lỗi trong listener loop: {e}")

class ConnectionState(Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    DISCONNECTING = "DISCONNECTING"
    ERROR = "ERROR"


class ConnectionManager:
    """
    Manages STM32 connection asynchronously to prevent UI freezing.
    All connection operations happen on worker threads with callbacks.
    """
    
    def __init__(self):
        self.stm_comm = STM32Communicator()
        self.state = ConnectionState.DISCONNECTED
        self.connection_callbacks = []  # (success, message) callbacks
        self.state_change_callbacks = []  # (new_state) callbacks
        self.worker_thread = None
        self.command_queue = queue.Queue()
        self.is_running = False
        self._last_connect_attempt_time = 0.0  # Thời điểm cuối cùng gọi connect_async
        self._connect_debounce_sec = 1.0       # Khoảng cách tối thiểu giữa các lần thử kết nối
        
        # Add message listener for non-blocking logging
        self.stm_comm.add_message_listener(self._on_stm_message)
    
    def add_connection_callback(self, callback: Callable[[bool, str], None]):
        """Add callback for connection attempt results: (success, message)"""
        if callback not in self.connection_callbacks:
            self.connection_callbacks.append(callback)
    
    def remove_connection_callback(self, callback: Callable):
        """Remove connection callback"""
        if callback in self.connection_callbacks:
            self.connection_callbacks.remove(callback)
    
    def add_state_change_callback(self, callback: Callable[[ConnectionState], None]):
        """Add callback for state changes: (new_state)"""
        if callback not in self.state_change_callbacks:
            self.state_change_callbacks.append(callback)
    
    def remove_state_change_callback(self, callback: Callable):
        """Remove state change callback"""
        if callback in self.state_change_callbacks:
            self.state_change_callbacks.remove(callback)
    
    def _set_state(self, new_state: ConnectionState):
        """Set connection state and notify callbacks"""
        if self.state != new_state:
            print(f"[DEBUG] Connection State Change: {self.state} -> {new_state}")
            self.state = new_state
            for callback in self.state_change_callbacks:
                try:
                    callback(new_state)
                except Exception as e:
                    pass
                    # print(f"Error in state change callback: {e}")
    
    def _notify_connection_callbacks(self, success: bool, message: str):
        """Notify all connection callbacks with dedupe for identical rapid messages."""
        # Không in ra console nữa, chỉ gọi callbacks để GUI hiển thị
        # if success and message.startswith("✓ Kết nối"):
        #     log_dedupe(message)
        # else:
        #     print(message)
        for callback in self.connection_callbacks:
            try:
                callback(success, message)
            except Exception as e:
                pass
                # print(f"Error in connection callback: {e}")
    
    def _on_stm_message(self, message: str):
        """Handle incoming messages from STM32"""
        # This is called from STM32Communicator's listener thread
        if message == "SYS_DISCONNECTED":
            self._set_state(ConnectionState.DISCONNECTED)
            # Notify callbacks so GUI can update (e.g. show Red indicator)
            self._notify_connection_callbacks(False, "Đã ngắt kết nối (Tín hiệu hệ thống)")
        elif message.startswith("ERROR:"):
             # Optional: Log error but don't disconnect yet unless fatal
             print(f"[CONN ERROR] {message}")
    
    def connect_async(self, port: str, baudrate: int = 115200,
                     timeout: float = 5.0, max_retries: int = 2):
        """
        Connect to STM32 asynchronously. 
        Does not block the calling thread.
        
        Args:
            port: COM port (e.g., 'COM3')
            baudrate: Connection speed (default 115200)
            timeout: Time to wait for PONG response (default 5 seconds)
            max_retries: Number of retry attempts (default 2)
        
        Callback will be called with (success, message)
        """
        if self.state in [ConnectionState.CONNECTING, ConnectionState.CONNECTED]:
            self._notify_connection_callbacks(
                False, "Đã kết nối hoặc đang kết nối"
            )
            return
        # Debounce: tránh spam nút kết nối liên tục
        now = time.time()
        if (now - self._last_connect_attempt_time) < self._connect_debounce_sec:
            self._notify_connection_callbacks(False, "Debounce: chờ 1s giữa các lần kết nối")
            return
        self._last_connect_attempt_time = now
        
        # Start worker thread if not running
        if not self.is_running:
            self.is_running = True
            self.worker_thread = threading.Thread(
                target=self._worker_loop, daemon=True, name="ConnectionWorker"
            )
            self.worker_thread.start()
        
        # Queue the connection task
        self._set_state(ConnectionState.CONNECTING)
        self.command_queue.put({
            'action': 'connect',
            'port': port,
            'baudrate': baudrate,
            'timeout': timeout,
            'max_retries': max_retries
        })
    
    def disconnect_async(self):
        """
        Disconnect from STM32 asynchronously.
        Does not block the calling thread.
        """
        if self.state == ConnectionState.DISCONNECTED:
            return
        
        self._set_state(ConnectionState.DISCONNECTING)
        self.command_queue.put({'action': 'disconnect'})
    
    def send_command(self, command: str) -> bool:
        """Send command to STM32 (non-blocking). Returns True if queued."""
        if not self.is_connected():
            return False
        return self.stm_comm.send_command(command)
    
    def is_connected(self) -> bool:
        """Check if currently connected"""
        return self.state == ConnectionState.CONNECTED
    
    def get_state(self) -> ConnectionState:
        """Get current connection state"""
        return self.state
    
    def add_message_listener(self, listener: Callable):
        """Add listener for STM32 messages"""
        self.stm_comm.add_message_listener(listener)
    
    def remove_message_listener(self, listener: Callable):
        """Remove message listener"""
        self.stm_comm.remove_message_listener(listener)
    
    def _worker_loop(self):
        """Worker thread main loop - handles connection tasks"""
        while self.is_running:
            try:
                # Get next command with timeout to allow graceful shutdown
                task = self.command_queue.get(timeout=1.0)
                
                if task['action'] == 'connect':
                    self._do_connect(
                        task['port'],
                        task['baudrate'],
                        task['timeout'],
                        task['max_retries']
                    )
                
                elif task['action'] == 'disconnect':
                    self._do_disconnect()
                
            except queue.Empty:
                # Check if should shut down
                if not self.is_running:
                    break
            except Exception as e:
                # print(f"Error in connection worker: {e}")
                self._set_state(ConnectionState.ERROR)
    
    def _do_connect(self, port: str, baudrate: int,
                   timeout: float, max_retries: int):
        """Perform actual connection (runs in worker thread) - PHIÊN BẢN HYBRID"""
        
        # Chúng ta không dùng max_retries nữa, vì logic hybrid sẽ xử lý trong 1 lần
        try:
            # 1. Khởi tạo kết nối
            self.stm_comm.ser = None  # Reset
            self.stm_comm.is_running = False
            
            import serial
            self.stm_comm.ser = serial.Serial(
                port, baudrate, timeout=1, write_timeout=1
            )
            
            # Chờ 2 giây để STM32 khởi động xong sau khi DTR reset
            time.sleep(2.0)
            
            self.stm_comm.is_running = True
            
            # Flush buffers ngay để xóa dữ liệu cũ
            self.stm_comm.ser.reset_input_buffer()
            self.stm_comm.ser.reset_output_buffer()
            
            # 2. Khởi động các luồng đọc và ghi
            self.stm_comm.read_thread = threading.Thread(
                target=self.stm_comm._read_loop, daemon=True
            )
            self.stm_comm.read_thread.start()
            
            # Khởi động write_thread để gửi lệnh
            self.stm_comm.write_thread = threading.Thread(
                target=self.stm_comm._write_loop, daemon=True
            )
            self.stm_comm.write_thread.start()
            
            self.stm_comm.listener_thread = threading.Thread(
                target=self.stm_comm._listener_loop, daemon=True
            )
            self.stm_comm.listener_thread.start()
            
            # Chờ một chút để các thread khởi động
            time.sleep(0.1)
            
            # 3. Xóa tin nhắn rác (từ bootloader, v.v.)
            while not self.stm_comm.message_queue.empty():
                try:
                    self.stm_comm.message_queue.get_nowait()
                except queue.Empty:
                    break
            
            # Xóa handshake queue cũ
            while not self.stm_comm.handshake_queue.empty():
                try:
                    self.stm_comm.handshake_queue.get_nowait()
                except queue.Empty:
                    break
            
            # 4. Gửi PING ngay lập tức
            # Kịch bản 1: STM32 đã chạy, nó sẽ nhận và gửi PONG.
            # Kịch bản 2: STM32 đang reset, nó sẽ bỏ lỡ PING.
            if not self.stm_comm.send_command("PING"):
                 self._set_state(ConnectionState.ERROR)
                 self._notify_connection_callbacks(
                     False,
                     "Lỗi Serial: Không thể gửi PING ban đầu."
                 )
                 return

            # 5. Chờ PONG (Kịch bản 1) HOẶC SYS_READY (Kịch bản 2) từ handshake_queue
            start_time = time.time()
            pong_received = False
            
            while time.time() - start_time < timeout:
                try:
                    # Lấy tin nhắn từ handshake_queue (chờ 0.2 giây mỗi lần để responsive)
                    response = self.stm_comm.handshake_queue.get(timeout=0.2)
                    
                    if response.startswith("PONG"):
                        # Kịch bản 1: STM32 đã chạy và phản hồi PING
                        # ✅ REMOVED: Không cần gửi ENABLE - STM32 tự động ENABLE khi nhận PING
                        pong_received = True
                        self._set_state(ConnectionState.CONNECTED)
                        
                        self._notify_connection_callbacks(
                            True, 
                            f"✓ Kết nối {port} (phản hồi {response})"
                        )
                        return
                    
                    if response.startswith("SYS_READY"):
                        # Kịch bản 2: STM32 vừa reset và gửi SYS_READY
                        # ✅ REMOVED: Không cần gửi ENABLE - STM32 sẽ tự ENABLE khi nhận PING tiếp theo
                        self._set_state(ConnectionState.CONNECTED)
                        
                        self._notify_connection_callbacks(
                            True, 
                            f"✓ Kết nối {port} (nhận {response})"
                        )
                        return
                    
                    # Nếu nhận được tin nhắn khác, cứ bỏ qua và tiếp tục chờ
                    
                except queue.Empty:
                    # Không có tin nhắn nào, tiếp tục vòng lặp chờ
                    pass
            
            # Kịch bản 3: Hết thời gian chờ
            self._do_disconnect() # Dọn dẹp
            self._set_state(ConnectionState.ERROR)
            self._notify_connection_callbacks(
                False,
                f"✗ Kết nối thất bại: Không nhận được PONG hoặc SYS_READY (Timeout)."
            )

        except Exception as e:
            # Xử lý lỗi nghiêm trọng (ví dụ: cổng COM không tồn tại)
            self._do_disconnect()
            self._set_state(ConnectionState.ERROR)
            self._notify_connection_callbacks(
                False,
                f"✗ Lỗi kết nối Serial nghiêm trọng: {e}"
            )
    
    def _do_disconnect(self):
        """Perform actual disconnection (runs in worker thread)"""
        try:
            if self.stm_comm.ser and self.stm_comm.ser.is_open:
                try:
                    # Gửi lệnh RESET_SYSTEM để STM32 tự dọn dẹp
                    self.stm_comm.ser.write(b"RESET_SYSTEM\n")
                    time.sleep(0.1) # Đợi STM32 xử lý
                except:
                    pass
                
                self.stm_comm.is_running = False
                
                # Wait for threads
                if self.stm_comm.read_thread and self.stm_comm.read_thread.is_alive():
                    self.stm_comm.read_thread.join(timeout=1)
                if self.stm_comm.listener_thread and self.stm_comm.listener_thread.is_alive():
                    self.stm_comm.listener_thread.join(timeout=1)
                
                self.stm_comm.ser.close()
                self.stm_comm.ser = None
        except Exception as e:
            pass
            # print(f"Error during disconnect: {e}")
        
        self._set_state(ConnectionState.DISCONNECTED)
        self._notify_connection_callbacks(
            True, "✓ Ngắt kết nối thành công"
        )
    
    def shutdown(self):
        """Shutdown the connection manager and worker thread"""
        self.is_running = False
        if self.stm_comm.is_connected():
            self.disconnect_async()
        
        # Wait for worker thread
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2)