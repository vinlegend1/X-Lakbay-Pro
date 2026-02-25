import cv2
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
        self.lock = threading.Lock()
        
        if self.camera:
            threading.Thread(target=self._capture_loop, daemon=True).start()
            threading.Thread(target=self._yolo_loop, daemon=True).start()
            threading.Thread(target=self._aruco_loop, daemon=True).start()
            threading.Thread(target=self._gesture_loop, daemon=True).start()

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
                    self.ros_node.latest_markers = [{"id": int(i[0])} for i in ids]
                else:
                    self.ros_node.latest_markers = []
                self.ros_node.pub_markers.publish(String(data=json.dumps([m['id'] for m in self.ros_node.latest_markers])))
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
