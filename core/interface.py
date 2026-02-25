class RobotInterface:
    def __init__(self, driver):
        self.driver = driver

    def stop(self):
        self.driver.set_movement(0, 0)

    def arm(self):
        self.driver.arm_vehicle(True)

    def disarm(self):
        self.driver.arm_vehicle(False)

    def move_forward(self, speed):
        self.driver.set_movement(linear=float(speed), angular=0.0)

    def turn(self, speed):
        self.driver.set_movement(linear=0.0, angular=float(speed))
        
    def sees_object(self, obj_name):
        return obj_name in self.driver.latest_detections

    def sees_marker(self, marker_id):
        return int(marker_id) in self.driver.latest_markers

    def sees_gesture(self, gesture_name):
        return self.driver.latest_gesture == gesture_name
