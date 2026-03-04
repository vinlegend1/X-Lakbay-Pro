import time

try:
    from std_msgs.msg import String
except ImportError:
    class String:
        def __init__(self, data=""): self.data = data

class RobotInterface:
    def __init__(self, driver, session_id):
        self.driver = driver
        self.session_id = session_id

    def _check_running(self):
        if not self.driver.code_running or self.driver.current_script_id != self.session_id:
            raise Exception("ROBOT_STOPPED")

    def log(self, *args):
        msg = " ".join(map(str, args))
        self.driver.get_logger().info(f"BLOCKLY_LOG: {msg}")
        self.driver.msg_pub.publish(String(data=f"LOG: {msg}"))

    def stop(self):
        # Programmed stop (from Blockly): motion set to 0, script continues
        self._check_running()
        self.driver.target_x = 0
        self.driver.target_y = 0
        self.driver.target_z = 0
        self.driver.target_r = 0

    def halt(self):
        # Emergency stop (from UI): kills code execution
        self.driver.code_running = False
        self.stop()

    def arm(self):
        self.driver.arm_vehicle(True)

    def disarm(self):
        self.stop() # Ensure we send 0 values before disarming
        self.driver.arm_vehicle(False)

    def forward(self, speed):
        self._check_running()
        self._apply_raw(throttle=speed)

    def backward(self, speed):
        self._check_running()
        self._apply_raw(throttle=-speed)

    def left(self, speed):
        self._check_running()
        self._apply_raw(steering=-speed)

    def right(self, speed):
        self._check_running()
        self._apply_raw(steering=speed)

    def _apply_raw(self, throttle=None, steering=None):
        # Scale 0-100 to 0-1000 for MAVLink MANUAL_CONTROL
        if throttle is not None:
            t = int(throttle * 10)
            if self.driver.mode == "UGV":
                self.driver.target_z = -t
            else:
                self.driver.target_y = t
        
        if steering is not None:
            s = int(steering * 10)
            if self.driver.mode == "UGV":
                self.driver.target_y = -s
            else:
                self.driver.target_z = s

    def move(self, x, y, z=0, r=0):
        self._check_running()
        # Direct raw mapping for legacy/advanced use
        self.driver.target_x = int(x)
        self.driver.target_y = int(y)
        self.driver.target_z = int(z)
        self.driver.target_r = int(r)
        
    def sees_object(self, obj_name):
        return obj_name in self.driver.latest_detections

    def sees_marker(self, marker_id):
        self._check_running()
        for m in self.driver.latest_markers:
            if m['id'] == int(marker_id):
                return True
        return False

    def get_marker_distance(self, marker_id):
        self._check_running()
        for m in self.driver.latest_markers:
            if m['id'] == int(marker_id):
                return m.get('distance', -1.0)
        return -1.0

    def get_marker_offset(self, marker_id):
        self._check_running()
        for m in self.driver.latest_markers:
            if m['id'] == int(marker_id):
                return m.get('offset_x', 0.0)
        return 0.0

    def sees_gesture(self, gesture_name):
        self._check_running()
        return self.driver.latest_gesture == gesture_name

    def follow_marker(self, marker_id, target_dist, deadzone, speed):
        """
        Smoothly follow a marker.
        - target_dist: Distance to maintain (meters)
        - deadzone: Distance tolerance (meters)
        - speed: Base speed (0-100)
        """
        self._check_running()
        
        marker = None
        for m in self.driver.latest_markers:
            if m['id'] == int(marker_id):
                marker = m
                break
        
        if not marker:
            self.stop()
            return

        dist = marker.get('distance', -1)
        offset = marker.get('offset_x', 0)
        
        # 1. Distance Control (Throttle)
        if dist > target_dist + deadzone:
            throttle = speed
        elif dist < target_dist - deadzone and dist > 0:
            # We move backward if too close, but only if we actually see the marker distance
            throttle = -speed
        else:
            throttle = 0 # Within deadzone
            
        # 2. Steering Control (Centered Offset)
        # We use a fixed 0.05m (5cm) steering deadzone for stability
        steer_deadzone = 0.05
        if offset > steer_deadzone:
            steering = speed * 0.7 # Scale steering for smoother turns
        elif offset < -steer_deadzone:
            steering = -speed * 0.7
        else:
            steering = 0
            
        self._apply_raw(throttle, steering)

    def set_heading_lock(self, enabled):
        """Enable or disable the PID straight-path correction."""
        self.driver.pid_enabled = bool(enabled)
        status = "ENABLED" if enabled else "DISABLED"
        self.log(f"Heading lock {status}")

    def wait(self, seconds):
        """Pause execution while allowing for immediate interruption."""
        start_time = time.time()
        while time.time() - start_time < seconds:
            self._check_running()
            time.sleep(0.05)
