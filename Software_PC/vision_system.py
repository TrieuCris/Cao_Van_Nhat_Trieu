# -*- coding: utf-8 -*-
"""
Vision System Module
====================
Hệ thống xử lý thị giác máy tính toàn diện cho Robot Delta.
Bao gồm:
1. Cấu hình Camera (CameraConfig)
2. Xử lý AI & Tracking (DeepLearningProcessor, ObjectTracker)
3. Luồng xử lý Video đa luồng (VideoThread)
"""

import json
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import time
import threading
import numpy as np
from queue import Queue, Empty
from typing import Optional, Tuple, Dict, Any, List, TYPE_CHECKING
import torch
from ultralytics import YOLO

# Import cv2 globally with environment settings
try:
    import cv2
except ImportError:
    cv2 = None
    print("[VISION WARNING] OpenCV không khả dụng, một số chức năng sẽ bị giới hạn")

# Đường dẫn mặc định cho file config
DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "camera_calibration.json")


# ==================================================================================================
# PART 1: CAMERA CONFIGURATION
# ==================================================================================================

class CameraConfig:
    """
    Class quản lý cấu hình camera calibration.
    """
    
    def __init__(self, config_path: str = DEFAULT_CONFIG_PATH):
        self.config_path = config_path
        
        # === ROI Settings ===
        self.roi: Optional[Tuple[int, int, int, int]] = None  # (x, y, w, h)
        
        # === Lens Correction ===
        self.distortion_k1: float = 0.0
        self.distortion_k2: float = 0.0
        
        # === Image Processing - Cơ bản ===
        self.brightness_offset: int = 0
        self.brightness_factor: float = 1.0
        self.contrast: float = 1.0
        self.gamma: float = 1.0
        self.saturation: float = 1.0
        
        # === Image Processing - Filters ===
        self.blur_size: int = 0
        self.median_blur: int = 0
        self.bilateral_d: int = 0
        self.sharpen_amount: float = 0.0
        
        # === Object Detection ===
        self.threshold_value: int = 60
        self.threshold_type: int = 0
        self.min_object_area: int = 1500
        self.max_object_area: int = 30000
        
        # === Morphology ===
        self.morph_open: int = 0
        self.morph_close: int = 0
        
        # === Camera Settings ===
        self.hw_brightness: Optional[int] = None
        self.hw_exposure: Optional[int] = None
        self.hw_auto_exposure: Optional[int] = None
        self.hw_gain: Optional[float] = None
        self.hw_focus: Optional[float] = None

        self.camera_index: Optional[int] = None
        self.camera_resolution: Tuple[int, int] = (1920, 1080)
        self.camera_fps: int = 30
        self.rotate_angle: int = 0
        self.flip_horizontal: bool = False
        self.flip_vertical: bool = False
        
        # === Full Distortion Parameters ===
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.map1: Optional[np.ndarray] = None
        self.map2: Optional[np.ndarray] = None

        # === Calibration ===
        self.calibration_points: list = []
        self._pixel_to_mm_matrix: Optional[np.ndarray] = None
        self._mm_to_pixel_matrix: Optional[np.ndarray] = None
        
        # Load config nếu tồn tại
        if os.path.exists(config_path):
            self.load()
    
    def _init_undistort_maps(self):
        if self.camera_matrix is not None and self.dist_coeffs is not None and self.camera_resolution:
            w, h = self.camera_resolution
            try:
                # New camera matrix is optional, here we just use the same one or optimize it
                # Using the same matrix keeps the view consistent with calibration
                new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                    self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
                
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    self.camera_matrix, self.dist_coeffs, None, 
                    new_camera_matrix, (w, h), cv2.CV_16SC2)
                print("[CONFIG] Undistort maps initialized successfully.")
            except Exception as e:
                print(f"[CONFIG] Error initializing undistort maps: {e}")
                self.map1 = None
                self.map2 = None

    def save(self, path: Optional[str] = None) -> bool:
        save_path = path or self.config_path
        config_dict = {
            "version": "2.0",
            "roi": self.roi,
            "distortion_k1": self.distortion_k1,
            "distortion_k2": self.distortion_k2,
            "brightness_offset": self.brightness_offset,
            "brightness_factor": self.brightness_factor,
            "contrast": self.contrast,
            "gamma": self.gamma,
            "saturation": self.saturation,
            "blur_size": self.blur_size,
            "median_blur": self.median_blur,
            "bilateral_d": self.bilateral_d,
            "sharpen_amount": self.sharpen_amount,
            "threshold_value": self.threshold_value,
            "threshold_type": self.threshold_type,
            "min_object_area": self.min_object_area,
            "max_object_area": self.max_object_area,
            "morph_open": self.morph_open,
            "morph_close": self.morph_close,
            "camera_index": self.camera_index,
            "camera_resolution": list(self.camera_resolution),
            "rotate_angle": self.rotate_angle,
            "flip_horizontal": self.flip_horizontal,
            "flip_vertical": self.flip_vertical,
            "calibration_points": self.calibration_points,
            "pixel_to_mm_matrix": self._pixel_to_mm_matrix.tolist() if self._pixel_to_mm_matrix is not None else None,
            "distortion": {
                "camera_matrix": self.camera_matrix.tolist() if self.camera_matrix is not None else None,
                "dist_coeffs": self.dist_coeffs.tolist() if self.dist_coeffs is not None else None,
            }
        }
        try:
            with open(save_path, 'w', encoding='utf-8') as f:
                json.dump(config_dict, f, indent=4, ensure_ascii=False)
            print(f"[CONFIG] Đã lưu cấu hình vào: {save_path}")
            return True
        except Exception as e:
            print(f"[CONFIG] Lỗi khi lưu config: {e}")
            return False
    
    def load(self, path: Optional[str] = None) -> bool:
        load_path = path or self.config_path
        if not os.path.exists(load_path):
            print(f"[CONFIG] File config không tồn tại: {load_path}")
            return False
        
        try:
            with open(load_path, 'r', encoding='utf-8') as f:
                config_dict = json.load(f)
            
            if "roi" in config_dict and isinstance(config_dict["roi"], dict):
                roi_data = config_dict["roi"]
                self.roi = (roi_data.get("x", 0), roi_data.get("y", 0),
                            roi_data.get("w", 0), roi_data.get("h", 0))
            else:
                self.roi = None
            
            if "camera" in config_dict:
                cam_data = config_dict["camera"]
                self.camera_index = cam_data.get("index", None)
                self.camera_resolution = tuple(cam_data.get("resolution", [1920, 1080]))
                self.hw_brightness = cam_data.get("brightness")
                self.hw_exposure = cam_data.get("exposure")
                self.hw_auto_exposure = cam_data.get("auto_exposure")
                self.hw_gain = cam_data.get("gain")
                self.hw_focus = cam_data.get("focus")
            
            if "distortion" in config_dict and isinstance(config_dict["distortion"], dict):
                dist_data = config_dict["distortion"]
                if "camera_matrix" in dist_data and "dist_coeffs" in dist_data:
                    self.camera_matrix = np.array(dist_data["camera_matrix"])
                    self.dist_coeffs = np.array(dist_data["dist_coeffs"])
                    print("[CONFIG] Loaded full distortion parameters.")
                    self._init_undistort_maps()
            else:
                self.distortion_k1 = config_dict.get("distortion_k1", 0.0)

            self.min_object_area = config_dict.get("min_object_area", self.min_object_area)
            self.max_object_area = config_dict.get("max_object_area", self.max_object_area)
            self.threshold_value = config_dict.get("threshold_value", self.threshold_value)

            if "calibration" in config_dict and isinstance(config_dict["calibration"], dict):
                calib_data = config_dict["calibration"]
                img_pts = calib_data.get("image_points_roi_px")
                robot_pts = calib_data.get("robot_points_mm")

                if img_pts and robot_pts and len(img_pts) == len(robot_pts):
                    self.calibration_points = []
                    for i in range(len(img_pts)):
                        px, py = img_pts[i]
                        rx, ry = robot_pts[i]
                        self.calibration_points.append((px, py, rx, ry))
                    print(f"[CONFIG] Loaded {len(self.calibration_points)} calibration points.")
                    self.compute_calibration_matrix()

        except Exception as e:
            print(f"[CONFIG] Lỗi khi load config: {e}")
            return False
        return True
    
    def compute_calibration_matrix(self) -> bool:
        if len(self.calibration_points) < 4:
            return False
        try:
            src_points = np.array([(p[0], p[1]) for p in self.calibration_points[:4]], dtype=np.float32)
            dst_points = np.array([(p[2], p[3]) for p in self.calibration_points[:4]], dtype=np.float32)
            self._pixel_to_mm_matrix, status = cv2.findHomography(src_points, dst_points)
            if self._pixel_to_mm_matrix is not None:
                self._mm_to_pixel_matrix = np.linalg.inv(self._pixel_to_mm_matrix)
                return True
            return False
        except Exception as e:
            print(f"[CONFIG] Lỗi khi tính ma trận: {e}")
            return False
    
    def pixel_to_mm(self, pixel_x: float, pixel_y: float) -> Optional[Tuple[float, float]]:
        if self._pixel_to_mm_matrix is None:
            return None
        try:
            point = np.array([pixel_x, pixel_y, 1.0], dtype=np.float64)
            result = self._pixel_to_mm_matrix @ point
            robot_x = result[0] / result[2]
            robot_y = result[1] / result[2]
            return (robot_x, robot_y)
        except Exception:
            return None
    
    def mm_to_pixel(self, robot_x: float, robot_y: float) -> Optional[Tuple[float, float]]:
        if self._mm_to_pixel_matrix is None:
            return None
        try:
            point = np.array([robot_x, robot_y, 1.0], dtype=np.float64)
            result = self._mm_to_pixel_matrix @ point
            pixel_x = result[0] / result[2]
            pixel_y = result[1] / result[2]
            return (pixel_x, pixel_y)
        except Exception:
            return None
    
    def is_calibrated(self) -> bool:
        return self._pixel_to_mm_matrix is not None
    
    def apply_undistort(self, frame: np.ndarray) -> np.ndarray:
        if cv2 is None: return frame
        
        # Use cached maps if available (Faster)
        if self.map1 is not None and self.map2 is not None:
             return cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)

        # Fallback to slow method if params exist but maps not inited
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            self._init_undistort_maps()
            if self.map1 is not None:
                 return cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
            return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            
        return frame

    def apply_roi(self, frame: np.ndarray) -> np.ndarray:
        if self.roi is None: return frame
        x, y, w_roi, h_roi = self.roi
        frame_h, frame_w = frame.shape[:2]
        x = max(0, min(x, frame_w - 1))
        y = max(0, min(y, frame_h - 1))
        w_roi = min(w_roi, frame_w - x)
        h_roi = min(h_roi, frame_h - y)
        return frame[y:y+h_roi, x:x+w_roi]
    
    def apply_display_transform(self, frame: np.ndarray, display_height: int = 720, extra_rotate: int = 90) -> np.ndarray:
        if cv2 is None: return frame
        h, w = frame.shape[:2]
        rotated_h = w if extra_rotate in [90, 270] else h
        scale = 1.0
        if rotated_h > display_height:
            scale = display_height / rotated_h
            new_w_pre = int(w * scale)
            new_h_pre = int(h * scale)
            frame = cv2.resize(frame, (new_w_pre, new_h_pre), interpolation=cv2.INTER_LINEAR)
            
        if extra_rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif extra_rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif extra_rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame
    
    def apply_image_adjustments(self, frame: np.ndarray) -> np.ndarray:
        return cv2.convertScaleAbs(frame, alpha=self.brightness_factor, beta=self.brightness_offset)


# Singleton instance
_config_instance: Optional[CameraConfig] = None
def get_camera_config(config_path: str = DEFAULT_CONFIG_PATH) -> CameraConfig:
    global _config_instance
    if _config_instance is None:
        _config_instance = CameraConfig(config_path)
    return _config_instance


# ==================================================================================================
# PART 2: AI PROCESSING & TRACKING
# ==================================================================================================

class DeepLearningProcessor:
    def __init__(self, camera_config: Optional[CameraConfig] = None):
        print("[VISION] Khởi tạo YOLOv8 OBB Processor.")
        self.camera_config = camera_config
        self.model = None
        model_path = os.path.join(os.path.dirname(__file__), "best_obb_traicay_0656.pt")
        
        if os.path.exists(model_path):
            try:
                print(f"[VISION] Đang load model từ: {model_path}")
                self.model = YOLO(model_path)
                print("[VISION] Model loaded successfully.")
                
                print("\n" + "="*50)
                print("       TRẠNG THÁI PHẦN CỨNG AI (GPU CHECK)      ")
                print("="*50)
                if torch.cuda.is_available():
                    print(f" [SUCCESS] ✅ Đã tìm thấy GPU (CUDA).")
                    print(f" [DEVICE]  Card màn hình: {torch.cuda.get_device_name(0)}")
                    print(f" [INFO]    Hệ thống sẽ chạy mượt mà (60+ FPS).")
                    # Force move to GPU
                    self.model.to('cuda')
                else:
                    print(f" [WARNING] ⚠️ KHÔNG tìm thấy GPU.")
                    print(f" [DEVICE]  Đang chạy trên CPU (Sẽ chậm).")
                
                # Đảm bảo model được đẩy sang thiết bị đúng (nếu cần thiết)
                # YOLOv8 thường tự động, nhưng in ra để chắc chắn
                print(f" [MODEL]   YOLO Engine đang chạy trên: {self.model.device}")
                print("="*50 + "\n")

            except Exception as e:
                print(f"[VISION ERROR] Không thể load model YOLO: {e}")
        else:
            print(f"[VISION ERROR] File model không tồn tại: {model_path}")

    def process_frame(self, frame: np.ndarray, draw: bool = True) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
        if self.model is None:
            return frame, []

        detections = []
        h_orig, w_orig = frame.shape[:2]
        
        # Crop 640x640 right side
        crop_size = 640
        x_start = max(0, w_orig - crop_size)
        x_end = w_orig
        crop_frame = frame[:, x_start:x_end]
        offset_x = x_start
        offset_y = 0
        
        try:
            # Determine device dynamically
            device_arg = 0 if torch.cuda.is_available() else 'cpu'
            results = self.model.predict(crop_frame, verbose=False, conf=0.9, imgsz=320, device=device_arg)
            result = results[0]
            
            if result.obb is not None:
                for obb in result.obb:
                    r = obb.xywhr[0].cpu().numpy()
                    cx_local, cy_local, w, h, rot_rad = r
                    cx = cx_local + offset_x
                    cy = cy_local + offset_y
                    
                    conf = float(obb.conf)
                    cls_id = int(obb.cls)
                    class_name = result.names[cls_id]
                    
                    angle_deg = np.degrees(rot_rad)
                    if w < h:
                        angle_deg += 90
                        
                    while angle_deg > 90: angle_deg -= 180
                    while angle_deg < -90: angle_deg += 180
                    
                    box_points_local = obb.xyxyxyxy[0].cpu().numpy().astype(np.int32)
                    box_points_global = box_points_local.copy()
                    box_points_global[:, 0] += offset_x
                    box_points_global[:, 1] += offset_y
                    
                    x_min = np.min(box_points_global[:, 0])
                    y_min = np.min(box_points_global[:, 1])
                    x_max = np.max(box_points_global[:, 0])
                    y_max = np.max(box_points_global[:, 1])
                    
                    bbox = [int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)]
                    center_int = (int(cx), int(cy))
                    
                    detection = {
                        'bbox': bbox,
                        'angle': angle_deg,
                        'center': center_int,
                        'rotated_box': box_points_global,
                        'class_name': class_name,
                        'confidence': conf,
                        'info_text': f"{class_name} {conf:.2f}"
                    }
                    detections.append(detection)
                    
                    if draw:
                        cv2.polylines(frame, [box_points_global], True, (0, 255, 0), 2)
                        cv2.circle(frame, center_int, 4, (0, 0, 255), -1)
                        label_text = f"{class_name} ({center_int[0]}, {center_int[1]})"
                        cv2.putText(frame, label_text, (int(x_min), int(y_min)-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if draw:
                cv2.line(frame, (x_start, 0), (x_start, h_orig), (0, 255, 255), 2)
                cv2.putText(frame, "YOLO ZONE >", (x_start + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        except Exception as e:
            print(f"[VISION ERROR] Lỗi khi xử lý frame với YOLO: {e}")

        return frame, detections


class ObjectTracker:
    def __init__(self):
        print("[TRACKER] Khởi tạo Object Tracker (Euclidean).")
        self.next_object_id = 1
        self.tracked_objects = {} 
        self.distance_threshold = 150
        self.min_hits = 3
        self.missed_frames = {}
        self.camera_config = None
        self.max_tracked_objects = 20
        self.robot_controller = None

    def update(self, detections, capture_time):
        current_time = capture_time
        
        if len(detections) == 0:
            for oid in list(self.tracked_objects.keys()):
                self.missed_frames[oid] = self.missed_frames.get(oid, 0) + 1
            self._cleanup_objects(current_time)
            return self._get_active_objects_list(current_time)

        input_centroids = []
        detection_bboxes = []
        detection_metadata = []
        
        for det in detections:
            if isinstance(det, dict):
                bbox = det['bbox']
                metadata = {
                    'angle': det.get('angle', 0),
                    'center': det.get('center', None),
                    'class_name': det.get('class_name', "Unknown"),
                    'confidence': det.get('confidence', 0.0),
                    'rotated_box': det.get('rotated_box', None)
                }
            else:
                bbox = det
                metadata = {'angle': 0, 'center': None, 'class_name': "Unknown", 'confidence': 0.0, 'rotated_box': None}
            
            x, y, w, h = bbox
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            input_centroids.append((cx, cy))
            detection_bboxes.append(bbox)
            detection_metadata.append(metadata)

        if len(self.tracked_objects) == 0:
            for i in range(min(len(detection_bboxes), self.max_tracked_objects)):
                self._register(detection_bboxes[i], capture_time, detection_metadata[i])
        else:
            object_ids = list(self.tracked_objects.keys())
            object_centroids = [self.tracked_objects[oid]["center"] for oid in object_ids]
            
            D = np.zeros((len(object_centroids), len(input_centroids)))
            for i in range(len(object_centroids)):
                for j in range(len(input_centroids)):
                    dist = np.sqrt((object_centroids[i][0] - input_centroids[j][0])**2 + 
                                   (object_centroids[i][1] - input_centroids[j][1])**2)
                    D[i, j] = dist
            
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            
            used_rows = set()
            used_cols = set()
            
            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols: continue
                if D[row, col] > self.distance_threshold: continue
                
                obj_id = object_ids[row]
                self._update_object(obj_id, detection_bboxes[col], capture_time, detection_metadata[col])
                if obj_id in self.missed_frames: del self.missed_frames[obj_id]
                
                used_rows.add(row)
                used_cols.add(col)
            
            for col in range(len(input_centroids)):
                if col not in used_cols:
                    if len(self.tracked_objects) < self.max_tracked_objects:
                        self._register(detection_bboxes[col], capture_time, detection_metadata[col])
            
            for oid in object_ids:
                if oid not in used_rows:
                    self.missed_frames[oid] = self.missed_frames.get(oid, 0) + 1
            
        self._cleanup_objects(current_time)
        return self._get_active_objects_list(current_time)

    def _register(self, bbox, capture_time, metadata=None):
        x, y, w, h = bbox
        center = (int(x + w/2), int(y + h/2))
        if metadata and metadata.get('center'): center = metadata['center']
        
        new_id = self.next_object_id
        self.tracked_objects[new_id] = {
            "bbox": bbox,
            "timestamp": capture_time,
            "center": center,
            "history": [center],
            "first_seen": capture_time,
            "metadata": metadata.copy() if metadata else {},
            "hits": 1,
            "status": "pending"
        }
        self.missed_frames[new_id] = 0
        self.next_object_id += 1

    def _update_object(self, obj_id, bbox, capture_time, metadata=None):
        x, y, w, h = bbox
        center = (int(x + w/2), int(y + h/2))
        if metadata and metadata.get('center'): center = metadata['center']
        
        obj = self.tracked_objects[obj_id]
        obj["bbox"] = bbox
        obj["center"] = center
        obj["timestamp"] = capture_time
        obj["hits"] += 1
        
        if metadata:
            current_status = obj.get("status", "pending")
            obj["metadata"] = metadata.copy()
            obj["status"] = current_status
        
        history = obj.get("history", [])
        history.append(center)
        if len(history) > 10: history = history[-10:]
        obj["history"] = history

    def _cleanup_objects(self, current_time):
        to_delete = []
        for oid, data in self.tracked_objects.items():
            missed_count = self.missed_frames.get(oid, 0)
            time_since_update = current_time - data["timestamp"]
            if time_since_update > 1.5 or missed_count >= 5:
                to_delete.append(oid)
        
        for oid in to_delete:
            if oid in self.tracked_objects: del self.tracked_objects[oid]
            if oid in self.missed_frames: del self.missed_frames[oid]
    
    def set_status(self, obj_id, status):
        if obj_id in self.tracked_objects:
            self.tracked_objects[obj_id]["status"] = status

    def _get_active_objects_list(self, current_time=None):
        if current_time is None: current_time = time.time()
        result = []
        for oid, data in self.tracked_objects.items():
            hits_ok = data["hits"] >= self.min_hits
            time_since_update = current_time - data["timestamp"]
            status = data.get("status", "pending")
            recently_updated = time_since_update < 0.5
            has_final_status = status in ["scheduled", "discarded"]
            
            if hits_ok and (recently_updated or has_final_status):
                metadata = data.get("metadata", {})
                metadata_with_status = metadata.copy()
                metadata_with_status['track_status'] = status
                result.append((oid, (data["bbox"], data["timestamp"], metadata_with_status)))
        return result


# ==================================================================================================
# PART 3: VIDEO PROCESSING THREAD
# ==================================================================================================

class VideoThread(threading.Thread):
    """Thread xử lý camera và detection riêng biệt với GUI."""
    
    def __init__(self, camera_index=None, camera_config: CameraConfig = None,
                 dl_processor=None, tracker=None, robot_controller=None, error_callback=None):
        super().__init__(daemon=True)
        self.camera_config = camera_config
        
        # Ưu tiên lấy index từ Config (JSON) nếu có
        if self.camera_config and self.camera_config.camera_index is not None:
            self.camera_index = self.camera_config.camera_index
        else:
            self.camera_index = camera_index

        self.dl_processor = dl_processor
        self.tracker = tracker
        self.robot_controller = robot_controller
        self.error_callback = error_callback
        
        if tracker and camera_config:
            tracker.camera_config = camera_config
        
        if tracker and robot_controller:
            tracker.robot_controller = robot_controller
        
        if robot_controller and tracker:
            robot_controller.tracker = tracker
        
        self.frame_queue = Queue(maxsize=2)
        self.running = False
        self.stopped = threading.Event()
        self.fps = 0.0
        self._fps_counter = 0
        self._fps_time = time.time()
        self.cap = None
        self.display_height = 720
        self.frame_count = 0
        self.process_interval = 1
        self.last_tracked_objects = []
        
    def start_capture(self):
        self.running = True
        self.stopped.clear()
        self.start()
        
    def stop_capture(self):
        self.running = False
        self.stopped.wait(timeout=2.0)
        if self.cap and self.cap.isOpened():
            self.cap.release()
            
    def set_display_height(self, height):
        if height > 1: self.display_height = height
    
    def _attempt_reconnect(self):
        try:
            if self.cap is not None:
                try: self.cap.release()
                except: pass
                self.cap = None
            time.sleep(1.0)
            print(f"[VideoThread] Reconnecting to camera index {self.camera_index}...")
            
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_MSMF)
            
            if self.cap.isOpened():
                backend_name = self.cap.getBackendName()
                print(f"[VideoThread] Opened camera index {self.camera_index} with backend: {backend_name}")
                if backend_name != "MSMF":
                     print(f"[VideoThread] ❌ Backend mismatch! Expected MSMF, got {backend_name}. Rejecting.")
                     self.cap.release()
                     self.cap = None

            if self.cap and self.cap.isOpened():
                if self.camera_config:
                    w, h = self.camera_config.camera_resolution
                else:
                    w, h = 1920, 1080
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'M','J','P','G'))
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # --- STRICT RESOLUTION CHECK (RECONNECT) ---
                actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                if int(actual_w) != w or int(actual_h) != h:
                    print(f"[VideoThread] ❌ Resolution Mismatch during reconnect! Got {int(actual_w)}x{int(actual_h)}, Expected {w}x{h}. Rejecting.")
                    self.cap.release()
                    self.cap = None
                    return False
                
                if self.camera_config:
                    if self.camera_config.hw_auto_exposure is not None:
                        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.camera_config.hw_auto_exposure)
                    if self.camera_config.hw_exposure is not None:
                        self.cap.set(cv2.CAP_PROP_EXPOSURE, float(self.camera_config.hw_exposure))
                    if self.camera_config.hw_brightness is not None:
                        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, float(self.camera_config.hw_brightness))
                    if self.camera_config.hw_gain is not None:
                        self.cap.set(cv2.CAP_PROP_GAIN, float(self.camera_config.hw_gain))
                    if self.camera_config.hw_focus is not None:
                        self.cap.set(cv2.CAP_PROP_FOCUS, float(self.camera_config.hw_focus))
                
                self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
                self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4000)
                
                print("[VideoThread] ✅ Camera reconnected successfully!")
                return True
            else:
                print("[VideoThread] ❌ Reconnect failed")
                return False
        except Exception as e:
            print(f"[VideoThread] ❌ Lỗi khi reconnect: {e}")
            self.cap = None
            return False
            
    def get_frame(self):
        try: return self.frame_queue.get_nowait()
        except Empty: return None
            
    def run(self):
        if self.camera_index is None:
            print("[VideoThread] ❌ No valid camera index found in config. Stopping to prevent opening laptop camera.")
            if self.error_callback:
                self.error_callback("No Camera Index Configured")
            self.stopped.set()
            return

        # Chỉ dùng MSMF theo yêu cầu
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_MSMF)
        
        if self.cap.isOpened():
            backend_name = self.cap.getBackendName()
            print(f"[VideoThread] Opened camera index {self.camera_index} with backend: {backend_name}")
            if backend_name != "MSMF":
                 print(f"[VideoThread] ❌ Backend mismatch! Expected MSMF, got {backend_name}. Rejecting.")
                 self.cap.release()
                 self.cap = None

        if self.cap is None or not self.cap.isOpened():
            print(f"[VideoThread] ERROR: Cannot open camera {self.camera_index}")
            print(f"[VideoThread] ❌ Failed to open Camera {self.camera_index}. Connection failed strictly. Will NOT fallback to other cameras.")
            if self.error_callback:
                self.error_callback(f"Cannot open camera {self.camera_index}")
            self.stopped.set()
            return
            
        w, h = 1920, 1080
        if self.camera_config:
            w, h = self.camera_config.camera_resolution
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4000)
        
        # --- STRICT RESOLUTION CHECK ---
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"[VideoThread] Camera Resolution: Requested={w}x{h}, Actual={int(actual_w)}x{int(actual_h)}")
        
        if int(actual_w) != w or int(actual_h) != h:
            print(f"[VideoThread] ❌ Resolution Mismatch! Camera does not support {w}x{h}. Likely wrong camera (Laptop?). Closing.")
            self.cap.release()
            self.cap = None
            if self.error_callback:
                self.error_callback(f"Resolution Mismatch: Got {int(actual_w)}x{int(actual_h)}")
            self.stopped.set()
            return

        if self.camera_config:
            if self.camera_config.hw_auto_exposure is not None:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.camera_config.hw_auto_exposure)
            if self.camera_config.hw_exposure is not None:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, float(self.camera_config.hw_exposure))
            if self.camera_config.hw_brightness is not None:
                self.cap.set(cv2.CAP_PROP_BRIGHTNESS, float(self.camera_config.hw_brightness))
            if self.camera_config.hw_gain is not None:
                self.cap.set(cv2.CAP_PROP_GAIN, float(self.camera_config.hw_gain))
            if self.camera_config.hw_focus is not None:
                self.cap.set(cv2.CAP_PROP_FOCUS, float(self.camera_config.hw_focus))
        else:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, -6.0)
        
        consecutive_read_errors = 0
        max_read_errors = 30
        reconnect_attempts = 0
        max_reconnect_attempts = 2
        
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                if reconnect_attempts < max_reconnect_attempts:
                    print(f"[VideoThread] Closed. Retrying... ({reconnect_attempts+1})")
                    if self._attempt_reconnect():
                        reconnect_attempts = 0
                        continue
                    reconnect_attempts += 1
                    time.sleep(0.5)
                    continue
                else:
                    print("[VideoThread] Critical: Camera lost.")
                    if self.error_callback: self.error_callback("Camera Disconnected")
                    self.running = False 
                    break

            ret, frame = self.cap.read()
            
            if not ret:
                consecutive_read_errors += 1
                if consecutive_read_errors >= max_read_errors:
                    if reconnect_attempts < max_reconnect_attempts:
                         print("[VideoThread] Read failed. Reconnecting...")
                         if self._attempt_reconnect():
                             consecutive_read_errors = 0
                             reconnect_attempts = 0
                             continue
                         reconnect_attempts += 1
                         time.sleep(1.0)
                    else:
                        print("[VideoThread] CRITICAL: Connection lost.")
                        if self.error_callback: self.error_callback("Camera Lost Connection")
                        self.running = False
                        break
                continue
            
            consecutive_read_errors = 0
            reconnect_attempts = 0
            
            try:
                self.frame_count += 1
                should_run_ai = (self.frame_count % self.process_interval == 0)
                
                # ✅ FIX: Lấy timestamp ngay lúc chụp ảnh (Capture Time)
                capture_time = time.perf_counter()
                
                # Process frame (AI + Tracking + Display Prep)
                # We pass 'should_run_ai' to control whether YOLO runs.
                # Tracking runs every frame (using predictions if AI is skipped).
                processed_frame = self._process_frame(frame, capture_time, run_ai=should_run_ai)

                self._update_fps()
                cv2.putText(processed_frame, f"FPS: {self.fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
                
                if self.frame_queue.full():
                    try: self.frame_queue.get_nowait()
                    except Empty: pass
                self.frame_queue.put(processed_frame)
                
            except Exception as e:
                print(f"[VideoThread] Processing error: {e}")
                continue
                
        if self.cap is not None:
            try:
                if self.cap.isOpened(): self.cap.release()
            except: pass
            finally: self.cap = None
        self.stopped.set()
        
    def _process_frame(self, frame, capture_time, run_ai=True):
        scale_factor = 1.0
        roi_w_original = None
        roi_h_original = None

        if self.camera_config:
            frame = self.camera_config.apply_undistort(frame)
            frame = self.camera_config.apply_image_adjustments(frame)
            frame = self.camera_config.apply_roi(frame)
        
        detections = []
        if self.dl_processor and run_ai:
            frame, detections = self.dl_processor.process_frame(frame, draw=False)
            
        tracked_objects = []
        if self.tracker:
            # ✅ FIX: Truyền capture_time xuống Tracker
            tracked_objects = self.tracker.update(detections, capture_time)
            if self.robot_controller:
                if self.robot_controller.auto_objects_queue.full():
                    try: self.robot_controller.auto_objects_queue.get_nowait()
                    except Empty: pass
                self.robot_controller.auto_objects_queue.put(tracked_objects)
        
        if self.camera_config:
            h_orig, w_orig = frame.shape[:2]
            roi_w_original = w_orig
            roi_h_original = h_orig
            
            if roi_w_original > self.display_height:
                scale_factor = self.display_height / roi_w_original
            
            frame = self.camera_config.apply_display_transform(frame, display_height=self.display_height, extra_rotate=90)
            
            w_scaled_orig = roi_w_original * scale_factor
            h_scaled_orig = roi_h_original * scale_factor
            
            for obj in tracked_objects:
                obj_id = obj[0]
                data = obj[1]
                bbox = data[0]
                metadata = data[2] if len(data) > 2 else {}
                status = metadata.get('track_status', 'pending')
                
                # Màu box theo status (BGR format)
                if status == "scheduled": 
                    box_color = (0, 255, 0)      # Xanh lá = đã lên lịch
                elif status == "discarded": 
                    box_color = (0, 0, 255)      # Đỏ = bỏ qua
                else: 
                    box_color = (255, 255, 0)    # Cyan = đang chờ
                
                rotated_box = metadata.get('rotated_box')
                if rotated_box is not None:
                    box_trans = []
                    for p in rotated_box:
                        px, py = p
                        new_x = int(h_scaled_orig - (py * scale_factor))
                        new_y = int(px * scale_factor)
                        box_trans.append([new_x, new_y])
                    box_trans = np.array(box_trans, dtype=np.int32)
                    cv2.polylines(frame, [box_trans], True, box_color, 1)
                else:
                    bx, by, bw, bh = bbox
                    corners = [(bx, by), (bx+bw, by), (bx+bw, by+bh), (bx, by+bh)]
                    box_trans = []
                    for px, py in corners:
                        new_x = int(h_scaled_orig - (py * scale_factor))
                        new_y = int(px * scale_factor)
                        box_trans.append([new_x, new_y])
                    box_trans = np.array(box_trans, dtype=np.int32)
                    cv2.polylines(frame, [box_trans], True, box_color, 1)
                
                if metadata.get('center'): cx, cy = metadata['center']
                else: cx, cy = bbox[0]+bbox[2]/2, bbox[1]+bbox[3]/2
                
                new_cx = int(h_scaled_orig - (cy * scale_factor))
                new_cy = int(cx * scale_factor)
                
                # Label gọn: ID + Class + Toạ độ
                label = f"#{obj_id}"
                if 'class_name' in metadata and metadata['class_name'] != "Unknown":
                    label += f" {metadata['class_name']}"
                
                # Toạ độ (robot hoặc pixel)
                coords_text = ""
                if self.camera_config and self.camera_config.is_calibrated():
                    robot_pos = self.camera_config.pixel_to_mm(cx, cy)
                    if robot_pos:
                        coords_text = f"({robot_pos[0]:.0f},{robot_pos[1]:.0f})"
                else:
                    coords_text = f"({int(cx)},{int(cy)})"
                
                # Tính kích thước text để căn giữa
                (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                (coords_w, coords_h), _ = cv2.getTextSize(coords_text, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)
                
                # Vẽ label ID + Class (phía trên center)
                cv2.putText(frame, label, (new_cx - label_w//2, new_cy - 8), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1, cv2.LINE_AA)
                
                # Vẽ toạ độ (phía dưới center)
                cv2.putText(frame, coords_text, (new_cx - coords_w//2, new_cy + coords_h + 6), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, box_color, 1, cv2.LINE_AA)

        if self.robot_controller and hasattr(self.robot_controller, 'trigger_line_p1'):
            p1_orig = self.robot_controller.trigger_line_p1
            if p1_orig and roi_w_original and roi_h_original:
                trigger_axis = getattr(self.robot_controller, 'trigger_axis', 'Y')
                h_frame, w_frame = frame.shape[:2]
                
                if trigger_axis == 'X':
                    trigger_val = p1_orig[0] * scale_factor
                    y_line = int(trigger_val)
                    y_line = max(0, min(y_line, h_frame - 1))
                    cv2.line(frame, (0, y_line), (w_frame, y_line), (0, 0, 255), 3)
                else:
                    trigger_val = p1_orig[1] * scale_factor
                    h_scaled = roi_h_original * scale_factor
                    x_line = int(h_scaled - trigger_val)
                    x_line = max(0, min(x_line, w_frame - 1))
                    cv2.line(frame, (x_line, 0), (x_line, h_frame), (0, 0, 255), 3)
            
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame
        
    def _update_fps(self):
        self._fps_counter += 1
        current_time = time.time()
        elapsed = current_time - self._fps_time
        if elapsed >= 1.0:
            self.fps = self._fps_counter / elapsed
            self._fps_counter = 0
            self._fps_time = current_time
