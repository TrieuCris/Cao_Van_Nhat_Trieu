# Đồ Án Tốt Nghiệp: Hệ Thống Robot Delta Phân Loại Sản Phẩm Sử Dụng Thị Giác Máy Tính

![Project Banner](https://img.shields.io/badge/Project-Graduation_Thesis-blue?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.11+-yellow?style=for-the-badge&logo=python)
![STM32](https://img.shields.io/badge/Firmware-STM32-green?style=for-the-badge&logo=stmicroelectronics)
![YOLOv8](https://img.shields.io/badge/AI-YOLOv8_OBB-purple?style=for-the-badge)

## 📖 Giới Thiệu

Dự án này là một hệ thống **Robot Delta** tự động hóa hoàn chỉnh, có khả năng phát hiện, theo dõi và phân loại sản phẩm chạy trên băng tải động. Hệ thống kết hợp giữa xử lý ảnh hiện đại (Deep Learning) và điều khiển robot chính xác thời gian thực.

Dự án bao gồm hai thành phần chính:
1.  **PC Controller (High-Level):** Chạy thuật toán AI, xử lý ảnh, lập kế hoạch quỹ đạo và giao diện người dùng (GUI).
2.  **Robot Controller (Low-Level):** Vi điều khiển STM32 điều khiển động cơ bước, bơm hút và xử lý tín hiệu cảm biến.

## 🎥 Demo Video

Xem hệ thống hoạt động thực tế tại đây:

[![Xem Video Demo](https://img.youtube.com/vi/aaXNv4fIjs0/0.jpg)](https://youtu.be/aaXNv4fIjs0)

## 🚀 Tính Năng Nổi Bật

*   **Nhận diện đối tượng:** Sử dụng mô hình **YOLOv8-OBB** (Oriented Bounding Box) để phát hiện vị trí và góc xoay của vật thể (ví dụ: trái cây, bánh).
*   **Hybrid Tracking (Trigger Line Locking):**
    *   Kết hợp **YOLOv8** và **Dead Reckoning** (tính toán dựa trên vận tốc băng tải).
    *   **Anti-Ghosting:** Cơ chế khóa đối tượng khi qua vạch Trigger, ngăn chặn việc nhận diện trùng lặp (double counting) và đảm bảo gắp chính xác 100% ngay cả khi vật ra khỏi vùng nhìn camera.
*   **Đồng bộ hóa thời gian thực (Time-Based Sync):**
    *   Sử dụng `time.perf_counter()` độ chính xác cao để đồng bộ vị trí vật thể với băng tải.
    *   Loại bỏ độ trễ (latency) giữa xử lý ảnh và hành động của robot.
*   **Lập kế hoạch chuyển động lai (Hybrid Motion Planning):**
    *   **Trapezoidal Planner:** Cho các chuyển động dài (ngang), giúp robot di chuyển mượt mà, giảm rung lắc.
    *   **Direct Send:** Cho các chuyển động ngắn (dọc/Z-axis) để tối đa hóa tốc độ đáp ứng.
*   **Hiệu năng cao:**
    *   **JIT Warmup:** Tối ưu hóa khởi động Numba, loại bỏ giật lag trong lần chạy đầu tiên.
    *   **Advanced Motion Control (DDS & Sliding Window):**
    *   **Sliding Window Protocol:** Giao thức truyền thông nạp trước (Look-ahead), cho phép robot thực thi chuỗi quỹ đạo phức tạp mà không bị gián đoạn.
    *   **DDS (Direct Digital Synthesis):** Thuật toán tạo xung bước siêu chính xác trên STM32, giúp động cơ vận hành êm ái và loại bỏ sai số tích lũy.
*   **Chế độ hoạt động:**
    *   **Auto Mode:** Tự động hoàn toàn, phân loại theo lớp đối tượng.
    *   **Manual Mode:** Giao diện điều khiển Jogging, kiểm tra IO, Home robot.

## 🛠️ Kiến Trúc Hệ Thống & Công Nghệ

### 1. Phần Mềm (PC - `Software_PC`)
*   **Ngôn ngữ:** Python 3.11+
*   **Giao diện:** PyQt6 (Modern GUI)
*   **Xử lý ảnh & AI:** OpenCV, Ultralytics YOLOv8
*   **Tính toán:** NumPy, Numba (High-performance JIT compiler)
*   **Giao tiếp:** PySerial (UART communication với STM32)

### 2. Phần Cứng (Firmware - `STM32_ROBOTDELTA`)
*   **Vi điều khiển:** STM32F103 (Blue Pill hoặc tương đương)
*   **Framework:** STM32 HAL, FreeRTOS
*   **Điều khiển:** Step Motor Drivers, Relay/Mosfet cho bơm khí nén.

## 📂 Cấu Trúc Thư Mục

```
CODE/
├── Software_PC/                # Mã nguồn phần mềm điều khiển trên máy tính
│   ├── main.py                 # Điểm khởi chạy chương trình (Main Entry)
│   ├── vision_system.py        # Xử lý ảnh, Camera, YOLO, Tracking
│   ├── robot_controller.py     # Quản lý kết nối và gửi lệnh xuống STM32
│   ├── auto_mode_controller.py # Logic điều khiển chế độ tự động (Scheduler)
│   ├── kinematics.py           # Tính toán động học nghịch đảo (Inverse Kinematics)
│   ├── pyqt_delta_gui.py       # Giao diện đồ họa (UI Layout)
│   ├── best_obb_traicay_0656.pt # Trọng số mô hình AI (Model Weights)
│   └── camera_calibration.json # File cấu hình tham số Camera
│
├── STM32_ROBOTDELTA/           # Mã nguồn Firmware cho vi điều khiển
│   ├── Core/Src/               # Source code C (main.c, interruptions...)
│   └── STM32_ROBOTDELTA.ioc    # File cấu hình CubeMX
│
└── GEMINI.md                   # Tài liệu ghi chú phát triển (Memory Context)
```

## ⚙️ Cài Đặt & Hướng Dẫn Sử Dụng

### Yêu cầu phần cứng
*   Máy tính chạy Windows (Khuyên dùng có GPU NVIDIA để chạy YOLO mượt hơn).
*   Camera (Webcam USB - Khuyên dùng Camera công nghiệp hỗ trợ MSMF).
*   Robot Delta kết nối qua cổng COM (USB-to-TTL).
*   Băng tải có thể điều chỉnh tốc độ.

### Các bước cài đặt

1.  **Clone repository:**
    ```bash
    git clone https://github.com/TrieuCris/Cao_Van_Nhat_Trieu.git
    cd Cao_Van_Nhat_Trieu/CODE/Software_PC
    ```

2.  **Cài đặt thư viện Python:**
    ```bash
    pip install PyQt6 opencv-python ultralytics numba pyserial numpy pillow
    # Lưu ý: Cài đặt PyTorch với hỗ trợ CUDA nếu có GPU NVIDIA
    ```

3.  **Cấu hình Camera:**
    *   File `camera_calibration.json` chứa thông số Matrix, Distortion.
    *   Đảm bảo `camera_index` trong file JSON trỏ đúng tới Camera ngoài (thường là 1).

4.  **Nạp Firmware:**
    *   Dùng STM32CubeIDE mở project `STM32_ROBOTDELTA`.
    *   Build và nạp code xuống mạch STM32.

### Vận hành

1.  Kết nối USB của Robot và Camera vào máy tính.
2.  Chạy phần mềm điều khiển:
    ```bash
    cd Software_PC
    python main.py
    ```
3.  Trên giao diện:
    *   Chọn cổng COM và nhấn **Connect**.
    *   Nhấn **Home** để đưa robot về vị trí gốc.
    *   Chuyển sang tab **Auto**, nhấn **Start** để bắt đầu phân loại.

## 🤝 Đóng Góp & Liên Hệ

*   **Tác giả:** Cao Văn Nhật Triều
*   **Đề tài:** Đồ án tốt nghiệp đại học
*   **Đề tài:** nhattrieu1772003@gmail.com