import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import time

# Khởi tạo camera
# Dùng CAP_DSHOW để khởi động nhanh
cap = cv2.VideoCapture(2, cv2.CAP_MSMF)  # Thay 2 bằng chỉ số camera của bạn nếu cần

# --- CẤU HÌNH CAMERA (Bắt buộc dùng MJPG cho Full HD 30fps) ---
# Đặt MJPG TRƯỚC khi đặt độ phân giải để tránh DSHOW chọn YUY2 mặc định (chậm)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)

# Font chữ để hiển thị
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.8
font_color = (0, 255, 0) # Màu xanh lá
thickness = 2

print("Bấm 'q' để thoát...")

while True:
    # --- 1. ĐO THỜI GIAN LẤY ẢNH (ACQUISITION) ---
    start_read = time.perf_counter() # Bắt đầu đo
    
    ret, frame = cap.read()
    
    end_read = time.perf_counter() # Kết thúc đo
    # Tính thời gian đọc (đổi sang mili-giây)
    read_time_ms = (end_read - start_read) * 1000 

    if not ret:
        print("Mất kết nối camera.")
        break

    # --- 2. ĐO THỜI GIAN XỬ LÝ (PROCESSING) ---
    start_process = time.perf_counter()

    # [Ví dụ xử lý] Ở đây bạn có thể chèn code AI, convert màu, detect vật thể...
    # Để test, mình sẽ thử convert sang ảnh xám (bỏ comment dòng dưới nếu muốn thử)
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Nếu là ảnh xám thì cần convert lại BGR để vẽ chữ màu
    # frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    # Lấy độ phân giải thực tế của khung hình hiện tại
    height, width = frame.shape[:2]

    # Chuẩn bị nội dung text
    info_resolution = f"Res: {width}x{height}"
    info_read = f"Read Time: {read_time_ms:.2f} ms"
    
    # Tính thời gian xử lý xong phần logic (trước khi vẽ UI cuối cùng)
    end_process = time.perf_counter()
    proc_time_ms = (end_process - start_process) * 1000
    info_proc = f"Process Time: {proc_time_ms:.2f} ms"

    # Tính FPS tức thời (Dựa trên tổng thời gian 1 vòng lặp)
    # Lưu ý: FPS này sẽ biến thiên liên tục
    total_time = read_time_ms + proc_time_ms
    fps = 1000 / total_time if total_time > 0 else 0
    info_fps = f"FPS: {fps:.1f}"

    # --- HIỂN THỊ THÔNG SỐ LÊN MÀN HÌNH ---
    # Vẽ lần lượt từng dòng
    cv2.putText(frame, info_resolution, (20, 40), font, font_scale, font_color, thickness)
    cv2.putText(frame, info_fps, (20, 80), font, font_scale, (0, 255, 255), thickness) # Màu vàng cho FPS
    cv2.putText(frame, info_read, (20, 120), font, font_scale, (100, 100, 255), thickness) # Màu đỏ nhạt
    cv2.putText(frame, info_proc, (20, 160), font, font_scale, (255, 100, 255), thickness)

    cv2.imshow('Camera Statistics Monitor', frame)

    # Bấm 'q' để thoát
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()