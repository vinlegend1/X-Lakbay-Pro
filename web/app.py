import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import threading
import time
import cv2
import logging

from flask import Flask, render_template, jsonify
from utils.helpers import get_ip_address
from core.vision import VisionSystem

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

try:
    import rclpy
    from rclpy.node import Node
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    class Node:
        def __init__(self, *args, **kwargs): pass
        def create_publisher(self, *args, **kwargs): return MockPub()
        def create_subscription(self, *args, **kwargs): pass
        def get_logger(self): return MockLogger()
    class MockPub:
        def publish(self, msg): pass
    class MockLogger:
        def info(self, msg): print(f"INFO: {msg}")
        def warn(self, msg): print(f"WARN: {msg}")

if ROS_AVAILABLE:
    from std_msgs.msg import String
else:
    class String:
        def __init__(self, data=""): self.data = data

app = Flask(__name__)

class FlaskRosNode(Node):
    def __init__(self):
        super().__init__('flask_commander')
        self.tele_status = "ROS Offline" if not ROS_AVAILABLE else "Checking..."
        self.is_armed = False
        self.current_mode = "MOCK" if not ROS_AVAILABLE else "None"
        self.flight_mode = "MOCK" if not ROS_AVAILABLE else "MANUAL"
        self.latest_msg = ""
        self.latest_detections = []
        self.latest_markers = []
        self.latest_gesture = "None"
        self.battery_status = {"percent": -1, "voltage": 0.0}
        self.code_running = False

        if ROS_AVAILABLE:
            self.pub_mode = self.create_publisher(String, '/mode_switch', 10)
            self.pub_arm = self.create_publisher(String, '/arm_cmd', 10)
            self.pub_detections = self.create_publisher(String, '/vision/detections', 10)
            self.pub_markers = self.create_publisher(String, '/vision/markers', 10)
            self.pub_gesture = self.create_publisher(String, '/vision/gesture', 10)
            self.create_subscription(String, '/tele_status', self.tele_cb, 10)
            self.create_subscription(String, '/armed_status', self.arm_status_cb, 10)
            self.create_subscription(String, '/current_mode', self.mode_cb, 10)
            self.create_subscription(String, '/flight_mode', self.flight_cb, 10)
            self.create_subscription(String, '/status_info', self.info_cb, 10)
            self.create_subscription(String, '/battery_status', self.battery_cb, 10)
            self.create_subscription(String, '/code_status', self.code_cb, 10)
        else:
            self.pub_mode = MockPub()
            self.pub_arm = MockPub()
            self.pub_detections = MockPub()
            self.pub_markers = MockPub()
            self.pub_gesture = MockPub()
            
    def tele_cb(self, msg): self.tele_status = msg.data
    def arm_status_cb(self, msg): self.is_armed = (msg.data == "ARMED")
    def mode_cb(self, msg): self.current_mode = msg.data
    def flight_cb(self, msg): self.flight_mode = msg.data
    def info_cb(self, msg): self.latest_msg = msg.data
    def battery_cb(self, msg):
        import json
        try: self.battery_status = json.loads(msg.data)
        except: pass
    def code_cb(self, msg): self.code_running = (msg.data == "RUNNING")

if ROS_AVAILABLE:
    rclpy.init()
    ros_node = FlaskRosNode()
    threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()
else:
    ros_node = FlaskRosNode()

vision_sys = VisionSystem(ros_node)

@app.route('/')
def index(): return render_template('index.html')

@app.route('/api/status')
def system_status():
    msg = ros_node.latest_msg
    ros_node.latest_msg = "" 
    return jsonify({
        "ip_address": get_ip_address(),
        "hw_status": ros_node.tele_status,
        "is_armed": ros_node.is_armed,
        "current_mode": ros_node.current_mode,
        "flight_mode": ros_node.flight_mode,
        "detections": ros_node.latest_detections,
        "markers": ros_node.latest_markers,
        "gesture": ros_node.latest_gesture,
        "status_msg": msg,
        "battery": ros_node.battery_status,
        "code_running": ros_node.code_running,
        "ros_status": True if ROS_AVAILABLE else False
    })

@app.route('/api/set_mode/<mode_name>')
def set_mode(mode_name):
    ros_node.pub_mode.publish(String(data=mode_name.upper()))
    return jsonify({"status": "success"})

def gen_view(stream_type):
    while True:
        frame = None
        if stream_type == 'yolo': frame = vision_sys.yolo_frame
        elif stream_type == 'gesture': frame = vision_sys.gesture_frame
        elif stream_type == 'aruco': frame = vision_sys.aruco_frame
        elif stream_type == 'calibration': frame = vision_sys.calibration_frame
        
        if frame is None:
            time.sleep(0.1)
            continue
            
        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.04)

@app.route('/video_feed/<stream_type>')
def video_feed(stream_type):
    return app.response_class(gen_view(stream_type), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/calibration/capture')
def calibrate_capture():
    success, message = vision_sys.capture_sample()
    return jsonify({"status": "success" if success else "error", "message": message})

@app.route('/api/calibration/calculate')
def calibrate_calculate():
    success, message = vision_sys.run_calibration()
    return jsonify({"status": "success" if success else "error", "message": message})

@app.route('/api/calibration/reset')
def calibrate_reset():
    success, message = vision_sys.reset_calibration()
    return jsonify({"status": "success" if success else "error", "message": message})

@app.route('/api/shutdown')
def shutdown():
    import subprocess
    print("🛑 Shutdown request received.")
    if os.name != 'nt':
        try:
            # Using sudo -S to pipe the password
            password = "123"
            print("Executing: sudo -S poweroff")
            process = subprocess.Popen(['sudo', '-S', 'poweroff'], stdin=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            process.communicate(input=password + '\n')
            return jsonify({"status": "shutting down"})
        except Exception as e:
            print(f"❌ Shutdown failed: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500
    else:
        print("Windows detected, skipping actual poweroff.")
        return jsonify({"status": "mock_shutdown"})

if __name__ == '__main__':
    print(f"🚀 Server running at http://{get_ip_address()}:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)
