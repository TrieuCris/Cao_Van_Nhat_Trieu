# calibrate_distortion_12x9.py
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import json
import glob
import numpy as np

JSON_FILE = "camera_calibration.json"
# Bạn có 13 ô × 9 ô => inner corners = (13-1, 9-1) = (12, 8)
CHESS_SIZE = (12, 8)   # inner corners (cols, rows)
SQUARE_MM = 20         # kích thước cạnh ô (mm)

# Load (or create minimal) JSON
if os.path.exists(JSON_FILE):
    with open(JSON_FILE, "r") as f:
        cfg = json.load(f)
else:
    cfg = {
        "version": "3.0",
        "distortion": {"camera_matrix": [], "dist_coeffs": []}
    }

# prepare object points: (0,0,0), (1,0,0), ... in mm
objp = np.zeros((CHESS_SIZE[0]*CHESS_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESS_SIZE[0], 0:CHESS_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_MM  # multiply by square size to get mm units

objpoints = []  # 3d point in real world space (mm)
imgpoints = []  # 2d points in image plane. 

images = sorted(glob.glob("chess/*.png"))
if len(images) == 0:
    print("Không tìm thấy ảnh trong thư mục 'chess/'. Thêm ảnh bàn cờ rồi chạy lại.")
    raise SystemExit

vis_dir = "chess_detected"
os.makedirs(vis_dir, exist_ok=True)

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print("Bỏ ảnh không đọc được:", fname)
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHESS_SIZE,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        # refine corners
        criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners_refined)

        # draw and save visualization
        vis = img.copy()
        cv2.drawChessboardCorners(vis, CHESS_SIZE, corners_refined, ret)
        base = os.path.basename(fname)
        cv2.imwrite(os.path.join(vis_dir, base), vis)
        print(f"[OK] {fname} -> corners detected, saved vis: {os.path.join(vis_dir, base)}")
    else:
        print(f"[MISS] {fname} -> corners NOT found")

if len(objpoints) < 5:
    print("Cảnh báo: tìm được quá ít ảnh (đề nghị >=10 ảnh, tối thiểu 5).")
    # vẫn có thể thử calibrate, nhưng kết quả kém

# calibrate
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Calibrate result ret:", ret)
print("Camera matrix:\n", mtx)
print("Dist coeffs:\n", dist.ravel())

# Save to JSON (ghi đè phần distortion)
cfg.setdefault("distortion", {})
cfg["distortion"]["camera_matrix"] = mtx.tolist()
cfg["distortion"]["dist_coeffs"] = dist.tolist()
cfg["distortion"]["image_size"] = list(gray.shape[::-1])

with open(JSON_FILE, "w") as f:
    json.dump(cfg, f, indent=2)

print("Saved distortion into", JSON_FILE)
print("Xong. Kiểm tra ảnh trong", vis_dir, "và thử undistort để xác nhận.")
