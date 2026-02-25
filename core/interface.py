class RobotInterface:
    def __init__(self, driver):
        self.driver = driver

    def _check_running(self):
        if not self.driver.code_running:
            raise Exception("ROBOT_STOPPED")

    def stop(self):
        # Programmed stop (from Blockly): motion set to 0, script continues
        self.driver.set_movement(0, 0)

    def halt(self):
        # Emergency stop (from UI): kills code execution
        self.driver.code_running = False
        self.driver.set_movement(0, 0)

    def arm(self):
        self.driver.arm_vehicle(True)

    def disarm(self):
        self.driver.arm_vehicle(False)

    def move_forward(self, speed):
        self._check_running()
        self.driver.set_movement(linear=float(speed), angular=0.0)

    def turn(self, speed):
        self._check_running()
        self.driver.set_movement(linear=0.0, angular=float(speed))
        
    def sees_object(self, obj_name):
        return obj_name in self.driver.latest_detections

    def sees_marker(self, marker_id):
        return int(marker_id) in self.driver.latest_markers

    def sees_gesture(self, gesture_name):
        return self.driver.latest_gesture == gesture_name
