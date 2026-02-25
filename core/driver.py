from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pymavlink import mavutil
import threading
import time
import json
from rclpy.node import Node

from utils.hardware_config import get_serial_config
from core.interface import RobotInterface

class VehicleConfig:
    def __init__(self, mode, neutral_z, fwd_scale, rev_scale, turn_scale, deadzone):
        self.mode = mode
        self.neutral_z = neutral_z
        self.fwd_scale = fwd_scale
        self.rev_scale = rev_scale
        self.turn_scale = turn_scale
        self.deadzone = deadzone

UGV_CONFIG = VehicleConfig("UGV", neutral_z=750, fwd_scale=500, rev_scale=500, turn_scale=1000, deadzone=10)
USV_SAFE_DEFAULT = VehicleConfig("USV", neutral_z=1500, fwd_scale=300, rev_scale=300, turn_scale=500, deadzone=20)

class GSbotDriver(Node):
    def __init__(self):
        super().__init__('gsbot_driver')
        self.config = UGV_CONFIG
        self.get_logger().info(f"⚙️ Active Profile: {self.config.mode}")

        self.master = None
        self.connect_pixhawk()

        self.target_z = self.config.neutral_z
        self.target_y = 0
        self.running = True

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/mode_switch', self.mode_callback, 10)
        self.create_subscription(String, '/arm_cmd', self.arm_callback, 10)
        self.create_subscription(String, '/blockly_code', self.execute_blockly_thread, 10)
        self.create_subscription(String, '/vision/detections', self.detections_callback, 10)
        self.create_subscription(String, '/vision/markers', self.markers_callback, 10)
        self.create_subscription(String, '/vision/gesture', self.gesture_callback, 10)
        
        self.status_pub = self.create_publisher(String, '/tele_status', 10)
        self.arm_status_pub = self.create_publisher(String, '/armed_status', 10)
        self.mode_pub = self.create_publisher(String, '/current_mode', 10)
        self.flight_mode_pub = self.create_publisher(String, '/flight_mode', 10)
        self.msg_pub = self.create_publisher(String, '/status_info', 10)
        self.battery_pub = self.create_publisher(String, '/battery_status', 10)

        self.latest_detections = []
        self.latest_markers = []
        self.latest_gesture = "None"

        self.robot_interface = RobotInterface(self)
        self.is_tele_ok = False
        self.is_armed = False

        self.heartbeat_thread = threading.Thread(target=self.mavlink_loop, daemon=True)
        self.heartbeat_thread.start()
        
        self.create_timer(1.0, self.check_tele_health)

    def connect_pixhawk(self):
        port, baud = get_serial_config()
        self.get_logger().info(f"🔍 Auto-detected hardware: Port={port}, Baud={baud}")
        try:
            self.master = mavutil.mavlink_connection(port, baud=baud)
            self.master.wait_heartbeat(timeout=2)
            self.is_tele_ok = True
            self.get_logger().info(f"✅ Pixhawk Initialized on {port}!")
        except Exception:
            self.get_logger().error(f"❌ Pixhawk NOT detected on {port}.")
            self.is_tele_ok = False

    def check_tele_health(self):
        if self.master:
            while True:
                msg = self.master.recv_match(blocking=False)
                if not msg:
                    break
                mtype = msg.get_type()
                if mtype == 'STATUSTEXT':
                    error_text = str(msg.text)
                    self.get_logger().warn(f"Pixhawk Msg: {error_text}")
                    self.msg_pub.publish(String(data=error_text))
                elif mtype == 'HEARTBEAT':
                    self.is_tele_ok = True
                    self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    try:
                        flight_mode = self.master.flightmode
                        self.flight_mode_pub.publish(String(data=str(flight_mode).upper()))
                    except:
                        self.flight_mode_pub.publish(String(data="UNKNOWN"))
                elif mtype == 'SYS_STATUS':
                    batt_volts = getattr(msg, 'voltage_battery', 0) / 1000.0
                    batt_pct = getattr(msg, 'battery_remaining', -1)
                    self.battery_pub.publish(String(data=json.dumps({"percent": batt_pct, "voltage": batt_volts})))
            
            self.status_pub.publish(String(data="Connected" if self.is_tele_ok else "No Link"))
            self.arm_status_pub.publish(String(data="ARMED" if self.is_armed else "DISARMED"))
            self.mode_pub.publish(String(data=self.config.mode))

    def arm_callback(self, msg):
        if msg.data.lower() == "arm": self.arm_vehicle(True)
        elif msg.data.lower() == "disarm": self.arm_vehicle(False)

    def arm_vehicle(self, arm_bool):
        if not self.master: return
        cmd = 1 if arm_bool else 0
        state = "ARMING" if arm_bool else "DISARMING"
        self.get_logger().info(f">>> {state} vehicle... <<<")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, cmd, 0, 0, 0, 0, 0, 0
        )

    def set_movement(self, linear, angular):
        neutral = self.config.neutral_z
        if linear > 0: pwm = int(neutral - (linear * self.config.fwd_scale))
        else: pwm = int(neutral + (abs(linear) * self.config.rev_scale))
        self.target_z = max(min(pwm, 1000), 0)
        self.target_y = int(angular * self.config.turn_scale)

    def mavlink_loop(self):
        while self.running:
            if self.master:
                try:
                    self.master.mav.manual_control_send(
                        self.master.target_system, 0,
                        int(self.target_y), int(self.target_z), int(self.target_y), 0
                    )
                except Exception: pass
            time.sleep(0.1)

    def cmd_vel_callback(self, msg):
        self.set_movement(msg.linear.x, msg.angular.z)

    def mode_callback(self, msg):
        if msg.data == "UGV": self.config = UGV_CONFIG
        elif msg.data == "USV": self.config = USV_SAFE_DEFAULT
        self.target_z = self.config.neutral_z

    def detections_callback(self, msg):
        try: self.latest_detections = json.loads(msg.data)
        except: pass

    def markers_callback(self, msg):
        try: self.latest_markers = [int(x) for x in json.loads(msg.data)]
        except: pass

    def gesture_callback(self, msg):
        self.latest_gesture = msg.data

    def execute_blockly_thread(self, msg):
        code = msg.data
        self.get_logger().info(f"Exec: {code}")
        threading.Thread(target=self._run_code, args=(code,), daemon=True).start()

    def _run_code(self, code):
        try: exec(code, {'robot': self.robot_interface, 'time': time})
        except Exception as e:
            self.get_logger().error(f"Script Error: {e}")
            self.set_movement(0, 0)
