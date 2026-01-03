import cv2
import json
import time

# =========================
# CẤU HÌNH
# =========================
JSON_FILE = "camera_calibration.json"

# Load config
try:
    with open(JSON_FILE, "r") as f:
        cfg = json.load(f)
except:
    cfg = {"camera": {}, "roi": {}, "calibration": {}, "distortion": {}}

# Lấy index từ JSON (Mặc định là 0 nếu không tìm thấy)
target_idx = cfg.get("camera", {}).get("index", 0)

# =========================
# KHỞI TẠO CAMERA (DSHOW - AUTO FIX)
# =========================
cap = None
working_idx = -1

print(f"--- Đang thử mở Camera Index: {target_idx} bằng DSHOW ---")
temp_cap = cv2.VideoCapture(target_idx, cv2.CAP_DSHOW)

# Thiết lập MJPG ngay để tránh treo driver (Quan trọng cho 13MP)
temp_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'M','J','P','G'))
temp_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
temp_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if temp_cap.isOpened():
    cap = temp_cap
    working_idx = target_idx
    print(f">>> Mở thành công Camera {working_idx}!")
else:
    print(f"!!! LỖI: Không mở được Camera {target_idx}. Đang dò tìm camera khác...")
    temp_cap.release()
    # Thử quét các index khác từ 0 đến 3
    for i in range(4):
        if i == target_idx: continue
        print(f"  > Đang thử Index {i}...")
        try_cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if try_cap.isOpened():
            # Thử đọc 1 frame xem có sống không
            ret, _ = try_cap.read()
            if ret:
                cap = try_cap
                working_idx = i
                print(f">>> TÌM THẤY CAMERA TẠI INDEX {i}!")
                # Update lại vào config để lần sau chạy đúng
                cfg["camera"]["index"] = i
                break
            else:
                try_cap.release()
        else:
            try_cap.release()

if cap is None or not cap.isOpened():
    print("!!! LỖI NGHIÊM TRỌNG: Không tìm thấy camera nào hoạt động!")
    print("Gợi ý: Hãy rút dây USB ra cắm lại và thử lại.")
    exit()

# =========================
# MỞ BẢNG CÀI ĐẶT
# =========================
print("Đang mở bảng cài đặt gốc...")
cap.set(cv2.CAP_PROP_SETTINGS, 1)

# =========================
# VÒNG LẶP CHÍNH
# =========================
print("---------------------------------------")
print(f"CHẾ ĐỘ CHỈNH MÀU (Camera Index: {working_idx})")
print("---------------------------------------")
print("  [O] : Mở lại bảng cài đặt (Settings)")
print("  [U] : CẬP NHẬT (Lưu thông số vào JSON)")
print("  [Q] : Thoát")
print("---------------------------------------")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Mất kết nối camera!")
        break

    cv2.imshow("TUNER MODE (DSHOW)", frame)
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('q'):
        break
    
    # Mở lại bảng settings nếu lỡ tắt
    elif key == ord('o'):
        cap.set(cv2.CAP_PROP_SETTINGS, 1)
        
    # Lưu thông số
    elif key == ord('u'):
        print("--- Đang đọc thông số từ bảng cài đặt ---")
        params = {
            "brightness": cv2.CAP_PROP_BRIGHTNESS,
            "contrast": cv2.CAP_PROP_CONTRAST,
            "saturation": cv2.CAP_PROP_SATURATION,
            "hue": cv2.CAP_PROP_HUE,
            "gain": cv2.CAP_PROP_GAIN,
            "exposure": cv2.CAP_PROP_EXPOSURE,
            "sharpness": cv2.CAP_PROP_SHARPNESS,
            "gamma": cv2.CAP_PROP_GAMMA,
            "white_balance": cv2.CAP_PROP_WB_TEMPERATURE,
            "backlight": cv2.CAP_PROP_BACKLIGHT,
            "focus": cv2.CAP_PROP_FOCUS
        }

        # Đọc giá trị
        read_count = 0
        for name, prop_id in params.items():
            val = cap.get(prop_id)
            if val != -1:
                cfg["camera"][name] = val
                print(f"  > {name}: {val}")
                read_count += 1
        
        if read_count > 0:
            with open(JSON_FILE, "w") as f:
                json.dump(cfg, f, indent=2)
            print(">>> ĐÃ LƯU CẤU HÌNH THÀNH CÔNG! <<<")
        else:
            print("!!! Cảnh báo: Không đọc được thông số nào (Driver chặn).")

cap.release()
cv2.destroyAllWindows()