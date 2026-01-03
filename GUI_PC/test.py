import sys
# Sử dụng PyQt5 để đồng bộ với robot_controller.py của bạn
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QFrame, QGridLayout, QLineEdit, 
                             QTextEdit, QComboBox, QGroupBox, QSlider, QCheckBox, QAbstractButton)
from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer, QSize, QPropertyAnimation, QEasingCurve, pyqtProperty, QObject
from PyQt6.QtGui import QImage, QPixmap, QFont, QColor, QIcon, QPainter, QBrush, QPen, QTextCursor
from robot_controller import RobotController

# --- 1. Tạo môi trường giả lập (Mock) ---
class MockConnectionManager(QObject):
    """Giả lập ConnectionManager để không cần kết nối STM32 thật"""
    message_received = pyqtSignal(str) # Signal giả để gửi data
    
    def __init__(self):
        super().__init__()
        self.is_connected = True

    def send_command(self, cmd):
        print(f"[STM32 Mock] Received command: {cmd}")

    def add_message_listener(self, listener):
        # Dummy method for compatibility
        self._listener = listener

# --- 2. Tạo ứng dụng giả lập (Mock Main App) ---
class MockMainApp:
    def __init__(self, controller):
        self.controller = controller
        # Kết nối signal phần cứng vào hàm test
        self.controller.hardware_start_pressed.connect(self.on_hardware_start)
        self.controller.hardware_stop_pressed.connect(self.on_hardware_stop)

    def on_hardware_start(self):
        print("\n>>> [MAIN APP] ✅ ĐÃ NHẬN LỆNH START TỪ PHẦN CỨNG!")
        print(">>> [MAIN APP] -> Đang kiểm tra an toàn... -> Kích hoạt Auto Mode\n")

    def on_hardware_stop(self):
        print("\n>>> [MAIN APP] 🛑 ĐÃ NHẬN LỆNH STOP TỪ PHẦN CỨNG!")
        print(">>> [MAIN APP] -> Dừng hệ thống ngay lập tức\n")

# --- 3. Kịch bản Test ---
if __name__ == "__main__":
    print("--- BẮT ĐẦU TEST MÔ PHỎNG NÚT BẤM ---")
    
    # Khởi tạo
    mock_connection = MockConnectionManager()
    mock_app = MockMainApp  # Định nghĩa class trước khi truyền vào
    # Tạo một instance của MockMainApp sau khi có robot_ctrl
    # Nhưng RobotController cần app, nên tạo một mock app tạm thời
    class DummyApp:
        def log_message(self, msg, level):
            print(f"[DummyApp][{level}] {msg}")
    dummy_app = DummyApp()
    robot_ctrl = RobotController(dummy_app, mock_connection)
    app = MockMainApp(robot_ctrl)
    
    # Giả lập gói tin STATUS từ STM32 gửi lên
    # Format: STATUS:run:homed:estop:BTN_START:BTN_STOP:conv:pump:angle:sensor:x,y,z
    
    print("1. Tình huống: Nút đang nhả (Start=0, Stop=0)")
    # Giả lập STM32 gửi trạng thái bình thường
    robot_ctrl.handle_stm_message("STATUS:0:1:0:0:0:0:0:0:0:0,0,0") 
    print("(Không có gì xảy ra - Đúng)")
    
    print("-" * 30)
    
    print("2. Tình huống: Người dùng ẤN nút Start (Start=0 -> 1)")
    # Giả lập STM32 gửi trạng thái nút Start được nhấn
    robot_ctrl.handle_stm_message("STATUS:0:1:0:1:0:0:0:0:0:0,0,0")
    # -> KỲ VỌNG: Hiện dòng "[MAIN APP] ✅ ĐÃ NHẬN LỆNH START..."
    
    print("-" * 30)
    
    print("3. Tình huống: Người dùng ĐANG GIỮ nút Start (Start=1 -> 1)")
    # Gói tin tiếp theo vẫn báo Start=1 (do tay chưa nhả ra)
    robot_ctrl.handle_stm_message("STATUS:0:1:0:1:0:0:0:0:0:0,0,0")
    print("(Không được hiện lệnh Start lần nữa - Đúng tính năng chống Spam)")
    
    print("-" * 30)

    print("4. Tình huống: Người dùng NHẢ nút Start (Start=1 -> 0)")
    robot_ctrl.handle_stm_message("STATUS:0:1:0:0:0:0:0:0:0:0,0,0")
    print("(Trạng thái về chờ)")
    
    print("-" * 30)
    
    print("5. Tình huống: Người dùng ẤN nút Stop (Stop=0 -> 1)")
    robot_ctrl.handle_stm_message("STATUS:0:1:0:0:1:0:0:0:0:0,0,0")
    # -> KỲ VỌNG: Hiện dòng "[MAIN APP] 🛑 ĐÃ NHẬN LỆNH STOP..."

    print("\n--- KẾT THÚC TEST ---")