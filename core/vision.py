import cv2
import numpy as np
import time
import json
import threading
import os
import mediapipe as mp
from ultralytics import YOLO
from mediapipe.tasks import python
from mediapipe.tasks.python import vision as mp_vision

from utils.helpers import find_camera, draw_landmarks_opencv

MODEL_DIR = os.path.join(os.path.dirname(__file__), '..', 'models')
vision_model = YOLO(os.path.join(MODEL_DIR, 'yolo11n_ncnn_model'), task='detect')
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

MARKER_SIZE = 0.15 # 15 cm (0.15 meters)

base_options = python.BaseOptions(model_asset_path=os.path.join(MODEL_DIR, 'gesture_recognizer.task'))
gesture_options = mp_vision.GestureRecognizerOptions(base_options=base_options)
recognizer = mp_vision.GestureRecognizer.create_from_options(gesture_options)

try:
    from std_msgs.msg import String
except ImportError:
    class String:
        def __init__(self, data=""): self.data = data

class VisionSystem:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.camera = find_camera()
        self.latest_frame = None
        self.yolo_frame = None
        self.aruco_frame = None
        self.gesture_frame = None
        self.calibration_frame = None
        self.calibrating = False
        self.calib_samples_count = 0
        self.calib_grid = (8, 6) # Standard checkerboard grid
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane
        self.objp = np.zeros((self.calib_grid[0] * self.calib_grid[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.calib_grid[0], 0:self.calib_grid[1]].T.reshape(-1, 2)
        
        self.latest_calib_corners = None
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self._load_camera_params()
        
        self.lock = threading.Lock()

    def _load_camera_params(self):
        params_path = os.path.join(MODEL_DIR, 'camera_params.json')
        if os.path.exists(params_path):
            try:
                with open(params_path, 'r') as f:
                    data = json.load(f)
                    self.camera_matrix = np.array(data['camera_matrix'])
                    self.dist_coeffs = np.array(data['dist_coeff'])
                    print(f"Loaded camera parameters from {params_path}")
            except Exception as e:
                print(f"Failed to load camera parameters: {e}")
        
        if self.camera:
            threading.Thread(target=self._capture_loop, daemon=True).start()
            threading.Thread(target=self._yolo_loop, daemon=True).start()
            threading.Thread(target=self._aruco_loop, daemon=True).start()
            threading.Thread(target=self._gesture_loop, daemon=True).start()
            threading.Thread(target=self._calibration_loop, daemon=True).start()

    def _capture_loop(self):
        while self.camera.isOpened():
            success, frame = self.camera.read()
            if success:
                with self.lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.1)

    def _yolo_loop(self):
        while True:
            if self.latest_frame is not None:
                results = vision_model(self.latest_frame, conf=0.4, verbose=False)
                self.yolo_frame = results[0].plot()
                detections = [{"label": vision_model.names[int(box.cls[0])], 
                               "conf": round(float(box.conf[0]) * 100, 1)} 
                               for r in results for box in r.boxes]
                self.ros_node.latest_detections = detections
                self.ros_node.pub_detections.publish(String(data=json.dumps([d['label'] for d in detections])))
            time.sleep(0.01)

    def _aruco_loop(self):
        while True:
            if self.latest_frame is not None:
                gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = aruco_detector.detectMarkers(gray)
                aruco_viz = self.latest_frame.copy()
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(aruco_viz, corners, ids)
                    markers = []
                    for i, marker_id in enumerate(ids):
                        marker_data = {"id": int(marker_id[0])}
                        if self.camera_matrix is not None:
                            # Estimate pose: returns rvecs, tvecs
                            # Using estimatePoseSingleMarkers (found in some OpenCV versions)
                            # or we can use cv2.solvePnP for better compatibility
                            # but let's try the common ArUco one first
                            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                                [corners[i]], MARKER_SIZE, self.camera_matrix, self.dist_coeffs
                            )
                            # tvec is [x, y, z]
                            marker_data["distance"] = round(float(tvecs[0][0][2]), 3) # Z is distance
                            marker_data["offset_x"] = round(float(tvecs[0][0][0]), 3) # X is horizontal offset
                            
                            # Draw coordinate axes for the marker
                            cv2.drawFrameAxes(aruco_viz, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.05)
                        
                        markers.append(marker_data)
                    self.ros_node.latest_markers = markers
                else:
                    self.ros_node.latest_markers = []
                
                self.ros_node.pub_markers.publish(String(data=json.dumps(self.ros_node.latest_markers)))
                self.aruco_frame = aruco_viz
            time.sleep(0.01)

    def _gesture_loop(self):
        while True:
            if self.latest_frame is not None:
                rgb = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = recognizer.recognize(mp_image)
                
                gesture_viz = self.latest_frame.copy()
                label = "No Hand Detected"
                
                if result.hand_landmarks:
                    for hand_lms in result.hand_landmarks:
                        draw_landmarks_opencv(gesture_viz, hand_lms)
                
                if result.gestures:
                    label = result.gestures[0][0].category_name
                else:
                    label = "None"
                self.ros_node.latest_gesture = label
                self.ros_node.pub_gesture.publish(String(data=label))
                
                cv2.putText(gesture_viz, label, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 120), 3)
                self.gesture_frame = gesture_viz
            time.sleep(0.05)

    def _calibration_loop(self):
        while True:
            if self.latest_frame is not None:
                calib_viz = self.latest_frame.copy()
                gray = cv2.cvtColor(calib_viz, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, self.calib_grid, None)
                
                if ret:
                    self.latest_calib_corners = corners
                    cv2.drawChessboardCorners(calib_viz, self.calib_grid, corners, ret)
                    cv2.putText(calib_viz, "Checkerboard Detected", (10, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    self.latest_calib_corners = None
                    cv2.putText(calib_viz, "Searching for Checkerboard...", (10, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                cv2.putText(calib_viz, f"Samples: {self.calib_samples_count}", (10, 70), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                self.calibration_frame = calib_viz
            time.sleep(0.1)

    def capture_sample(self):
        if self.latest_calib_corners is not None:
            self.objpoints.append(self.objp)
            self.imgpoints.append(self.latest_calib_corners)
            self.calib_samples_count += 1
            return True, f"Sample {self.calib_samples_count} captured"
        return False, "No checkerboard detected"

    def reset_calibration(self):
        self.objpoints = []
        self.imgpoints = []
        self.calib_samples_count = 0
        return True, "Calibration reset"

    def run_calibration(self):
        if self.calib_samples_count < 10:
            return False, f"Need at least 10 samples (currently {self.calib_samples_count})"
        
        try:
            gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, gray.shape[::-1], None, None
            )
            
            if ret:
                calib_data = {
                    "camera_matrix": mtx.tolist(),
                    "dist_coeff": dist.tolist(),
                    "samples": self.calib_samples_count,
                    "date": time.strftime("%Y-%m-%d %H:%M:%S")
                }
                save_path = os.path.join(MODEL_DIR, 'camera_params.json')
                with open(save_path, 'w') as f:
                    json.dump(calib_data, f)
                return True, f"Calibration successful! Saved to {save_path}"
            else:
                return False, "Calibration failed"
        except Exception as e:
            return False, f"Error: {str(e)}"
