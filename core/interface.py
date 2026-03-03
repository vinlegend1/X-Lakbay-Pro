import time

class RobotInterface:
    def __init__(self, driver):
        self.driver = driver

    def _check_running(self):
        if not self.driver.code_running:
            raise Exception("ROBOT_STOPPED")

    def stop(self):
        # Programmed stop (from Blockly): motion set to 0, script continues
        self.driver.target_x = 0
        self.driver.target_y = 0
        self.driver.target_z = 0
        self.driver.target_r = 0

    def halt(self):
        # Emergency stop (from UI): kills code execution
        self.driver.code_running = False
        self.driver.target_x = 0
        self.driver.target_y = 0
        self.driver.target_z = 0
        self.driver.target_r = 0

    def arm(self):
        self.driver.arm_vehicle(True)

    def disarm(self):
        self.stop() # Ensure we send 0 values before disarming
        self.driver.arm_vehicle(False)

    def forward(self, speed):
        self._check_running()
        self._apply_raw(speed, 0)

    def backward(self, speed):
        self._check_running()
        self._apply_raw(-speed, 0)

    def left(self, speed):
        self._check_running()
        self._apply_raw(0, -speed)

    def right(self, speed):
        self._check_running()
        self._apply_raw(0, speed)

    def _apply_raw(self, throttle, steering):
        # Scale 0-100 to 0-1000 for MAVLink MANUAL_CONTROL
        t = int(throttle * 10)
        s = int(steering * 10)
        
        # Reset all axes
        self.driver.target_x = 0
        self.driver.target_y = 0
        self.driver.target_z = 0
        self.driver.target_r = 0
        
        if self.driver.mode == "UGV":
            # UGV Mode: Z is Forward/Backward, Y is Yaw
            self.driver.target_z = -t
            self.driver.target_y = -s
        else: # USV Mode
            # USV Mode: Y is Forward/Backward, Z is Steering
            self.driver.target_y = t
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
        return int(marker_id) in self.driver.latest_markers

    def sees_gesture(self, gesture_name):
        return self.driver.latest_gesture == gesture_name
