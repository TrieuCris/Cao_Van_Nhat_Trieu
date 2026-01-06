# -*- coding: utf-8 -*-
import sys
import io
import os
import threading
import time
from PIL import Image, ImageTk
import tkinter.messagebox as messagebox 


os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
os.environ["QT_SCALE_FACTOR"] = "1"


sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')


def thread_exception_hook(args):
    print(f"\n{'='*60}")
    print(f"THREAD EXCEPTION: {args.exc_type.__name__}: {args.exc_value}")
    print(f"Thread: {args.thread.name}")
    print(f"{'='*60}")
    import traceback
    traceback.print_exception(args.exc_type, args.exc_value, args.exc_traceback)
    print(f"{'='*60}\n")

threading.excepthook = thread_exception_hook


from PyQt6.QtWidgets import QApplication, QMessageBox
from PyQt6.QtCore import Qt, QtMsgType, qInstallMessageHandler, QTimer


def qt_message_handler(mode, context, message):
    if "QFont::setPointSize" in message and "Point size <= 0" in message: return
    if mode == QtMsgType.QtWarningMsg: print(f"Qt Warning: {message}")
    elif mode == QtMsgType.QtCriticalMsg: print(f"Qt Critical: {message}")
    elif mode == QtMsgType.QtFatalMsg: print(f"Qt Fatal: {message}")

qInstallMessageHandler(qt_message_handler)

qt_app = None


from vision_system import (
    CameraConfig, 
    DeepLearningProcessor, 
    ObjectTracker, 
    VideoThread, 
    get_camera_config
)
from connection_manager import ConnectionManager
from robot_controller import RobotController
import constants as C

# =============================================================================
# --- MAIN CALLBACKS (Đã chuyển từ main_callbacks.py) ---
# =============================================================================

def send_stm_command(app, command):
    """Gửi một lệnh tùy chỉnh đến STM32."""
    if app.robot_controller and app.robot_controller.conn_manager.is_connected():
        app.robot_controller.conn_manager.send_command(command)
        app.log_message(f"Sent: {command}", "sent")

def on_move(app):
    """Callback khi người dùng nhấn nút MOVE."""
    app.log_message("🔍 MOVE: Kiểm tra điều kiện...", "info")
    
    if not app.conn_manager.is_connected():
        app.log_message("❌ MOVE: Chưa kết nối STM32", "error")
        return

    if not app.is_manual_mode():
        app.log_message("❌ MOVE: Chỉ dùng được ở chế độ MANUAL (bật toggle Manual)", "error")
        return
    
    hs = app.robot_controller.homing_state if app.robot_controller else "None"
    if not app.robot_controller or app.robot_controller.homing_state != "COMPLETED":
        app.log_message("❌ MOVE: Cần HOME trước khi di chuyển", "error")
        return
    
    try:
        x = app.get_coordinate('X')
        y = app.get_coordinate('Y')
        z = app.get_coordinate('Z')
    except Exception as e:
        app.log_message(f"❌ Lỗi lấy tọa độ: {e}", "error")
        return
    
    def move_thread():
        try:
            success = app.robot_controller.move_to_coords(x, y, z)
            if not success:
                app.after(0, lambda: app.log_message(f"❌ MOVE thất bại"))
        except Exception as e:
            import traceback
            traceback.print_exc()
            app.after(0, lambda: app.log_message(f"❌ MOVE Thread Crash: {e}", "error"))
    
    threading.Thread(target=move_thread, daemon=True, name="MoveThread").start()

def on_home(app):
    """Callback khi người dùng nhấn nút HOME."""
    if not app.conn_manager.is_connected():
        app.log_message("❌ HOME: Chưa kết nối STM32", "error")
        return

    if not app.is_manual_mode():
        app.log_message("❌ HOME: Cần bật chế độ MANUAL trước!", "error")
        return
    
    app.log_message("🏠 Bắt đầu homing...", "sent")
    success = app.robot_controller.home_robot()
    if not success:
        app.log_message("❌ Không thể homing (robot đang bận hoặc lỗi)", "error")

def on_conveyor_start(app, forward):
    """Callback khi người dùng bắt đầu chạy băng tải."""
    if not app.conn_manager.is_connected():
        app.log_message("❌ Chưa kết nối STM32", "error")
        return

    if not app.is_manual_mode():
        app.log_message("❌ Băng tải chỉ dùng được ở chế độ MANUAL", "error")
        return
    
    app.robot_controller.send_conveyor_start(forward)

def on_stop_demo(app):
    if hasattr(app, 'demo_stop_flag'):
        app.demo_stop_flag = True
        app.log_message("⏹️ Yêu cầu dừng DEMO", "info")

def on_conveyor_stop(app):
    """Callback khi người dùng dừng băng tải."""
    if not app.conn_manager.is_connected():
        app.log_message("❌ Chưa kết nối STM32", "error")
        return

    if not app.is_manual_mode():
        app.log_message("❌ Băng tải chỉ dùng được ở chế độ MANUAL", "error")
        return
    
    app.robot_controller.send_conveyor_stop()

def on_conveyor_set_speed(app):
    """Callback khi người dùng đặt tốc độ băng tải."""
    if not app.conn_manager.is_connected():
        app.log_message("❌ Chưa kết nối STM32", "error")
        return

    if not app.is_manual_mode():
        app.log_message("❌ Băng tải chỉ dùng được ở chế độ MANUAL", "error")
        return
    
    try:
        speed = int(app.get_conveyor_speed())
        if not (0 <= speed <= 255):
            messagebox.showerror("Lỗi Nhập Liệu", "Tốc độ băng tải phải là một số nguyên từ 0 đến 255.")
            return
    except ValueError:
        messagebox.showerror("Lỗi Nhập Liệu", "Tốc độ băng tải phải là một số nguyên.")
        return
    
    app.robot_controller.send_conveyor_set_speed(speed)

def on_pump(app, state):
    """Callback khi người dùng bật/tắt pump."""
    if not app.conn_manager.is_connected():
        app.log_message("❌ Chưa kết nối STM32", "error")
        app.pump_switch.state = not state
        return

    if not app.is_manual_mode():
        app.log_message("❌ Pump chỉ dùng được ở chế độ MANUAL", "error")
        app.pump_switch.state = not state
        return
    
    if not app.robot_controller or app.robot_controller.homing_state != "COMPLETED":
        app.log_message("❌ Cần HOME trước khi điều khiển pump", "error")
        app.pump_switch.state = not state
        return
    
    success = app.robot_controller.set_pump(state)
    if not success:
        app.pump_switch.state = not state

def on_servo_set_angle(app):
    """Callback khi thả chuột khỏi slider."""
    def reset_to_coord_angle():
        if hasattr(app.robot_controller, 'current_servo_angle'):
            coord_angle = app.robot_controller.current_servo_angle - 225.0
            app.set_servo_angle(coord_angle)

    if not app.conn_manager.is_connected():
        reset_to_coord_angle()
        app.log_message("❌ Chưa kết nối STM32", "error")
        return

    if not app.is_manual_mode():
        reset_to_coord_angle()
        app.log_message("❌ Servo chỉ dùng được ở chế độ MANUAL", "error")
        return
    
    try:
        coord_angle_val = app.get_servo_angle()
    except (ValueError, TypeError):
        reset_to_coord_angle()
        messagebox.showerror("Lỗi Giá Trị", "Giá trị góc servo không hợp lệ.")
        return
    
    success = app.robot_controller.set_servo(coord_angle_val)
    if not success:
        reset_to_coord_angle()

def on_servo_set_from_entry(app):
    """Callback khi nhấn SET hoặc Enter trên ô nhập liệu."""
    def reset_to_coord_angle():
        if hasattr(app.robot_controller, 'current_servo_angle'):
            coord_angle = app.robot_controller.current_servo_angle - 225.0
            app.set_servo_angle(coord_angle)

    if not app.conn_manager.is_connected():
        reset_to_coord_angle()
        app.log_message("❌ Chưa kết nối STM32", "error")
        return

    if not app.is_manual_mode():
        reset_to_coord_angle()
        app.log_message("❌ Servo chỉ dùng được ở chế độ MANUAL", "error")
        return
    
    try:
        coord_angle_val = float(app.get_servo_entry_value())
        if not (-225 <= coord_angle_val <= 45):
            reset_to_coord_angle()
            messagebox.showerror("Lỗi Nhập Liệu", "Góc tọa độ của servo phải là một số từ -225 đến 45.")
            return
    except (ValueError, TypeError):
        reset_to_coord_angle()
        messagebox.showerror("Lỗi Nhập Liệu", "Góc tọa độ của servo phải là một số hợp lệ.")
        return
    
    app.set_servo_angle(coord_angle_val)
    success = app.robot_controller.set_servo(coord_angle_val)
    if not success:
        reset_to_coord_angle()

def on_servo_nudge_plus(app):
    """Callback khi nhấn nút servo '+'."""
    def reset_to_coord_angle():
        if hasattr(app.robot_controller, 'current_servo_angle'):
            coord_angle = app.robot_controller.current_servo_angle - 225.0
            app.set_servo_angle(coord_angle)

    if not app.conn_manager.is_connected():
        reset_to_coord_angle()
        app.log_message("❌ Chưa kết nối STM32", "error")
        return
    if not app.is_manual_mode():
        reset_to_coord_angle()
        app.log_message("❌ Servo chỉ dùng được ở chế độ MANUAL", "error")
        return

    try:
        current_coord_angle = app.get_servo_angle()
        new_coord_angle = min(45.0, current_coord_angle + C.SERVO_NUDGE_AMOUNT)
        app.set_servo_angle(new_coord_angle)
    except (ValueError, TypeError) as e:
        reset_to_coord_angle()
        return

    success = app.robot_controller.set_servo(new_coord_angle)
    if not success:
        reset_to_coord_angle()

def on_servo_nudge_minus(app):
    """Callback khi nhấn nút servo '-'."""
    def reset_to_coord_angle():
        if hasattr(app.robot_controller, 'current_servo_angle'):
            coord_angle = app.robot_controller.current_servo_angle - 225.0
            app.set_servo_angle(coord_angle)

    if not app.conn_manager.is_connected():
        reset_to_coord_angle()
        app.log_message("❌ Chưa kết nối STM32", "error")
        return
    if not app.is_manual_mode():
        reset_to_coord_angle()
        app.log_message("❌ Servo chỉ dùng được ở chế độ MANUAL", "error")
        return
        
    try:
        current_coord_angle = app.get_servo_angle()
        new_coord_angle = max(-225.0, current_coord_angle - C.SERVO_NUDGE_AMOUNT)
        app.set_servo_angle(new_coord_angle)
    except (ValueError, TypeError) as e:
        reset_to_coord_angle()
        return

    success = app.robot_controller.set_servo(new_coord_angle)
    if not success:
        reset_to_coord_angle()

# =============================================================================
# --- MAIN APPLICATION LOGIC ---
# =============================================================================

def on_connect_stm(app):
    port = app.get_selected_port()
    if not port:
        QMessageBox.warning(app, "Lỗi", "Vui lòng chọn một cổng COM.")
        return
    
    if app.conn_manager.is_connected():
        app.log_message("⚠️ Đã kết nối rồi", "warning")
        return
    
    app.log_message(f"Đang kết nối tới {port}...", "sent")
    
    def on_connection_result(success, message):
        def _ui_update():
            app.log_message(message, "received")
            app.update_indicator(C.INDICATOR_STM_CONNECTED, success)
            if success:
                app._update_toggle_switches_state(True)
        app.after(0, _ui_update)
    
    if hasattr(app, '_on_connection_result_callback'):
        app.conn_manager.remove_connection_callback(app._on_connection_result_callback)
    app._on_connection_result_callback = on_connection_result
    app.conn_manager.add_connection_callback(on_connection_result)
    
    app.conn_manager.connect_async(port, timeout=2.0, max_retries=2)

def on_disconnect_stm(app):
    if not app.conn_manager.is_connected():
        app.log_message("⚠️ Đã ngắt kết nối rồi", "warning")
        return

    if hasattr(app, 'robot_controller'):
        if app.robot_controller.is_running_auto:
            app.log_message("❌ Không thể ngắt kết nối khi đang ở chế độ AUTO", "error")
            return
        if app.robot_controller.state == C.RobotState.ESTOP:
            app.log_message("❌ Không thể ngắt kết nối khi đang E-STOP", "error")
            return

    app.log_message("Đang ngắt kết nối...", "sent")
    
    def on_disconnection_result(success, message):
        def _ui_update():
            app.log_message(message, "received")
            if success:
                app.update_indicator(C.INDICATOR_STM_CONNECTED, False)
                app._update_toggle_switches_state(False)
                if hasattr(app, 'robot_controller'):
                    app.robot_controller.reset_system_state()
                if app.is_manual_mode(): app.set_manual_mode(False)
                app.update_indicator(C.INDICATOR_HOME_OK, False)
                app.update_indicator(C.INDICATOR_AUTO, False)
                app.set_product_count(0)
        app.after(0, _ui_update)
    
    if hasattr(app, '_on_disconnection_result_callback'):
        app.conn_manager.remove_connection_callback(app._on_disconnection_result_callback)
    app._on_disconnection_result_callback = on_disconnection_result
    app.conn_manager.add_connection_callback(on_disconnection_result)
    
    app.conn_manager.disconnect_async()

def update_gui_frame(app):
    if not getattr(app, 'is_video_loop_active', False): return
    try:
        if not app.isVisible():
            app.is_video_loop_active = False 
            return
        if hasattr(app, 'video_thread') and app.video_thread:
            label_height = app.camera_label.height()
            if label_height > 1: app.video_thread.set_display_height(label_height)
            frame = app.video_thread.get_frame()
            if frame is not None: app.update_camera_feed(frame)
        
        # ✅ PERIODIC: Check block timeout (mỗi 10s tự động cleanup)
        if hasattr(app, 'robot_controller') and app.robot_controller:
            app.robot_controller.check_block_timeout()
            
    except Exception as e:
        print(f"[GUI UPDATE ERROR] {e}")
    QTimer.singleShot(30, lambda: update_gui_frame(app))

def start_video_feed(app):
    try:
        if hasattr(app, 'video_thread') and app.video_thread:
            app.log_message("Stopping old camera thread...", "info")
            app.video_thread.stop_capture()
            app.video_thread = None

        camera_config = get_camera_config(C.CAMERA_CONFIG_PATH)
        # Force reload config from file to pick up changes (e.g. index change)
        camera_config.load()
        app.camera_config = camera_config
        
        if camera_config.is_calibrated():
            app.log_message(f"✅ Đã load camera calibration config", "info")
            app.robot_controller.compute_trigger_line_pixel(camera_config)
        else:
            app.log_message("⚠️ chưa calibrate camera!", "error")
        
        dl_processor = DeepLearningProcessor(camera_config)
        tracker = ObjectTracker()
        
        def on_cam_error(msg):
            app.after(0, lambda: app.handle_camera_error(msg))

        app.video_thread = VideoThread(
            camera_index=camera_config.camera_index,
            camera_config=camera_config,
            dl_processor=dl_processor,
            tracker=tracker,
            robot_controller=app.robot_controller,
            error_callback=on_cam_error
        )
        app.video_thread.start_capture()
        
        app.update_indicator(C.INDICATOR_CAMERA_ON, True)
        app.log_message("Camera feed started (threaded).", "info")
        
        if hasattr(app, 'robot_controller') and app.robot_controller:
            if app.robot_controller.state == C.RobotState.ESTOP:
                app.log_message("⚠️ Camera đã kết nối lại, nhưng Robot vẫn đang E-STOP!", "warning")
            else:
                app.update_indicator(C.INDICATOR_ERROR, False)
        else:
            app.update_indicator(C.INDICATOR_ERROR, False)
        
        if not getattr(app, 'is_video_loop_active', False):
            app.is_video_loop_active = True
            update_gui_frame(app)
            
        # Cập nhật đèn nút bấm ngay khi camera sẵn sàng
        if hasattr(app, 'robot_controller') and app.robot_controller:
            app.robot_controller.update_hardware_button_leds()

    except Exception as e:
        error_msg = f"Lỗi Camera: {e}"
        app.log_message(error_msg, "error")
        app.update_indicator(C.INDICATOR_CAMERA_ON, False)

def cleanup_video(app):
    if hasattr(app, 'video_thread') and app.video_thread:
        print("[CLEANUP] Stopping video thread...")
        app.video_thread.stop_capture()
        app.video_thread = None

def main():
    os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    
    global qt_app
    qt_app = QApplication(sys.argv)
    
    from pyqt_delta_gui import DeltaRobotGUI

    try:
        app = DeltaRobotGUI()
        app.video_update_id = None
        app.video_thread = None
        
        def manual_toggle_callback(state):
            is_stm_connected = app.conn_manager.is_connected()
            if state and not is_stm_connected:
                app.log_message("❌ Chưa kết nối STM32", "error")
                app.set_manual_mode(False)
                return
            
            if state and app.robot_controller.is_running_auto:
                 app.log_message("❌ Tắt AUTO trước khi bật MANUAL", "error")
                 app.set_manual_mode(False)
                 return

            app.update_indicator(C.INDICATOR_MANUAL, state)
            if state: app.log_message("✅ Chế độ MANUAL đã được BẬT", "info")
            else: app.log_message("ℹ️ Chế độ MANUAL đã được TẮT", "info")
            
            if hasattr(app, 'robot_controller'):
                app.robot_controller.update_hardware_button_leds()
        
        app._on_manual_toggle_callback = manual_toggle_callback

        app.set_button_commands({
            'run': app.on_run if hasattr(app, 'on_run') else lambda: print("Run pressed"),
            'stop': app.on_stop if hasattr(app, 'on_stop') else lambda: print("Stop pressed"),
            'move': lambda: on_move(app),
            'home': lambda: on_home(app),
            'pump': lambda state: on_pump(app, state),
            'conveyor_fwd_press': lambda: on_conveyor_start(app, forward=True),
            'conveyor_rev_press': lambda: on_conveyor_start(app, forward=False),
            'conveyor_release': lambda: on_conveyor_stop(app),
            'conveyor_set_speed': lambda: on_conveyor_set_speed(app),
            'servo_set_angle': lambda: on_servo_set_angle(app),
            'servo_set_from_entry': lambda: on_servo_set_from_entry(app),
            'servo_nudge_plus': lambda: on_servo_nudge_plus(app),
            'servo_nudge_minus': lambda: on_servo_nudge_minus(app),
            'connect_stm': lambda: on_connect_stm(app),
            'disconnect_stm': lambda: on_disconnect_stm(app)
        })
        
        app.btn_reload_camera.clicked.connect(lambda: start_video_feed(app))

        def on_run_logic():
            if not app.conn_manager.is_connected():
                app.log_message("❌ Chưa kết nối STM32", "error"); return
            if app.robot_controller.state == C.RobotState.ESTOP:
                app.log_message("❌ Hệ thống đang E-STOP.", "error"); return
            if app.robot_controller.homing_state != "COMPLETED":
                app.log_message("❌ Cần HOME trước", "error"); return
            if app.is_manual_mode():
                app.log_message("❌ Tắt MANUAL trước", "error"); return
            
            # ✅ CHECK CAMERA STATUS
            if not app.video_thread or not app.video_thread.running or not app.video_thread.cap or not app.video_thread.cap.isOpened():
                app.log_message("❌ Camera chưa sẵn sàng!", "error"); return
            
            print("[DEBUG] GUI START button: Starting Auto Mode")
            app.robot_controller.set_auto_mode(True)

        def on_stop_logic():
            if app.robot_controller.state == C.RobotState.ESTOP:
                app.log_message("⚠️ Hệ thống đang E-STOP.", "warning"); return
            app.robot_controller.set_auto_mode(False)

        # ✅ TẠO ROBOT CONTROLLER TRƯỚC (để có signals)
        app.robot_controller = RobotController(app, app.conn_manager)

        # Kết nối tín hiệu nút cứng với Auto Mode Controller
        def on_hardware_start():
            print("[DEBUG] Hardware START button callback triggered")
            # Kiểm tra điều kiện an toàn trước khi chạy
            if not app.robot_controller.homing_state == "COMPLETED":
                print("Cannot start: Robot not homed!")
                app.log_message("❌ Nút cứng: Robot chưa HOME!", "error")
                return
            if app.robot_controller.state == C.RobotState.ESTOP:
                print("Cannot start: E-Stop active!")
                app.log_message("❌ Nút cứng: E-Stop đang kích hoạt!", "error")
                return
            if app.is_manual_mode():
                print("Cannot start: Manual mode active!")
                app.log_message("❌ Nút cứng: Tắt MANUAL trước!", "error")
                return
            
            # ✅ CHECK CAMERA STATUS
            if not app.video_thread or not app.video_thread.running or not app.video_thread.cap or not app.video_thread.cap.isOpened():
                print("Cannot start: Camera not ready!")
                app.log_message("❌ Nút cứng: Camera chưa sẵn sàng!", "error")
                return

            print(">>> Hardware Start Triggered: Starting Auto Mode")
            app.robot_controller.set_auto_mode(True)
            if hasattr(app, 'update_auto_mode_ui'):
                app.update_auto_mode_ui(True)

        def on_hardware_stop():
            print("[DEBUG] Hardware STOP button callback triggered")
            print(">>> Hardware Stop Triggered: Stopping Auto Mode")
            app.robot_controller.set_auto_mode(False)
            if hasattr(app, 'update_auto_mode_ui'):
                app.update_auto_mode_ui(False)

        # ✅ KẾT NỐI SIGNALS (SAU KHI ĐÃ TẠO ROBOT CONTROLLER)
        app.robot_controller.hardware_start_pressed.connect(on_hardware_start)
        app.robot_controller.hardware_stop_pressed.connect(on_hardware_stop)
        
        # Kết nối nút GUI
        app.btn_run.clicked.connect(on_run_logic)
        app.btn_stop.clicked.connect(on_stop_logic)
        
        initial_speed = app.get_conveyor_speed()
        try:
            speed_value = float(initial_speed)
            app.robot_controller.conveyor_speed_mm_s = speed_value
        except ValueError: pass
        
        app.conn_manager.add_message_listener(app.robot_controller.handle_stm_message)
        
        def stm_message_logger(message):
            if message.startswith(("DEBUG:", "STATUS:", "DONE:", "ACK:")): return
            if any(k in message for k in ["Sent batch:", "Trajectory:", "blocks in plan", "🎯"]): return
            app.log_message(message, "received")
        app.conn_manager.add_message_listener(stm_message_logger)

        def start_jog(axis, direction):
            if not app.conn_manager.is_connected():
                app.log_message("❌ Chưa kết nối STM32", "error"); return
            if not app.is_manual_mode():
                app.log_message("❌ JOG chỉ dùng được ở chế độ MANUAL", "error"); return
            if app.robot_controller:
                app.robot_controller.start_jogging(axis, direction)
        
        def stop_jog():
            if app.is_manual_mode() and app.robot_controller:
                app.robot_controller.stop_jogging()

        app.bind_jog('Y+', lambda: start_jog('y', 1), stop_jog)
        app.bind_jog('Y-', lambda: start_jog('y', -1), stop_jog)
        app.bind_jog('X+', lambda: start_jog('x', 1), stop_jog)
        app.bind_jog('X-', lambda: start_jog('x', -1), stop_jog)
        app.bind_jog('Z+', lambda: start_jog('z', 1), stop_jog)
        app.bind_jog('Z-', lambda: start_jog('z', -1), stop_jog)

        # ✅ NEW: Periodic block timeout check (mỗi 10s)
        def check_block_health():
            if app.robot_controller:
                app.robot_controller.check_block_timeout()
            QTimer.singleShot(10000, check_block_health)  # Repeat every 10s
        
        app.show()
        QTimer.singleShot(100, lambda: start_video_feed(app))
        QTimer.singleShot(10000, check_block_health)  # Start block health check
        qt_app.aboutToQuit.connect(lambda: cleanup_video(app))
        sys.exit(qt_app.exec())

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
