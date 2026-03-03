from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pymavlink import mavutil
import threading
import time
import json
from rclpy.node import Node
import math
from utils.hardware_config import get_serial_config
from utils.control import PID, LowPassFilter
from core.interface import RobotInterface

# Note: VehicleConfig and scaling logic removed in favor of direct raw control as requested.

class GSbotDriver(Node):
    def __init__(self):
        super().__init__('gsbot_driver')

        self.mode = "UGV"
        self.master = None
        self.connect_pixhawk()

        # Control targets (0 is neutral)
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0
        self.target_r = 0

        self.running = True
        self.code_running = False

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
        self.code_status_pub = self.create_publisher(String, '/code_status', 10)

        self.latest_detections = []
        self.latest_markers = []
        self.latest_gesture = "None"

        self.robot_interface = RobotInterface(self)
        self.is_tele_ok = False
        self.is_armed = False

        # PID & Filter for Straight Path Correction
        self.current_yaw = 0.0
        self.target_yaw = None
        self.lpf_yaw = LowPassFilter(alpha=0.2)
        # Tuning: These values may need adjustment based on vehicle responsiveness
        self.pid_yaw = PID(kp=10.0, ki=0, kd=0, output_limit=1000)

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
                if mtype == 'ATTITUDE':
                    self.current_yaw = self.lpf_yaw.apply(msg.yaw)
                elif mtype == 'STATUSTEXT':
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
            self.mode_pub.publish(String(data=self.mode))
            self.code_status_pub.publish(String(data="RUNNING" if self.code_running else "IDLE"))

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

    def mavlink_loop(self):
        while self.running:
            if self.master:
                try:
                    # 1. Update latest IMU/Attitude data for the PID
                    msg = self.master.recv_match(type='ATTITUDE', blocking=False)
                    if msg:
                        self.current_yaw = self.lpf_yaw.apply(msg.yaw)

                    # 2. Determine if we should apply PID Straight-Correction
                    # Steering is target_y for UGV, target_z for USV
                    # Throttle is target_z for UGV, target_y for USV
                    is_steering = False
                    if self.mode == "UGV":
                        is_steering = (abs(self.target_y) > 0)
                        is_moving = (abs(self.target_z) > 0)
                    else:
                        is_steering = (abs(self.target_z) > 0)
                        is_moving = (abs(self.target_y) > 0)

                    out_x, out_y, out_z, out_r = self.target_x, self.target_y, self.target_z, self.target_r

                    if is_moving and not is_steering:
                        # Start holding current heading if we weren't already
                        if self.target_yaw is None:
                            self.target_yaw = self.current_yaw
                        
                        # Calculate error and correct
                        error = self.target_yaw - self.current_yaw
                        # Normalize error to [-pi, pi]
                        if error > math.pi: error -= 2 * math.pi
                        if error < -math.pi: error += 2 * math.pi
                        
                        correction = self.pid_yaw.update(error)
                        
                        # Apply correction to the steering axis
                        if self.mode == "UGV":
                            out_y = -correction # UGV Steering is Y
                        else:
                            out_z = correction # USV Steering is Z
                    else:
                        # Reset PID when steering or stopped
                        self.target_yaw = None
                        self.pid_yaw.reset_integral()

                    # 3. Send Manual Control
                    self.master.mav.manual_control_send(
                        self.master.target_system,
                        int(out_x),
                        int(out_y),
                        int(out_z),
                        int(out_r),
                        0
                    )
                except Exception as e:
                    self.get_logger().debug(f"MAVLink Loop Error: {e}")
            time.sleep(0.05) # Increased frequency for smoother PID (20Hz)

    def cmd_vel_callback(self, msg):
        pass

    def mode_callback(self, msg):
        self.mode = msg.data
        self.mode_pub.publish(String(data=self.mode))

    def detections_callback(self, msg):
        try: self.latest_detections = json.loads(msg.data)
        except: pass

    def markers_callback(self, msg):
        try: self.latest_markers = [int(x) for x in json.loads(msg.data)]
        except: pass

    def gesture_callback(self, msg):
        self.latest_gesture = msg.data

    def execute_blockly_thread(self, msg):
        code = msg.data.strip()
        self.get_logger().info(f"Exec: {code}")
        
        # If it's just a halt command, stop immediately and don't start a new script thread
        if code == "robot.halt()":
            self.robot_interface.halt()
            return

        # New script execution: stop any previous script first
        self.code_running = False
        time.sleep(0.1) # Brief pause to allow old thread to catch the flag
        
        self.code_running = True
        threading.Thread(target=self._run_code, args=(code,), daemon=True).start()

    def _run_code(self, code):
        try: 
            exec(code, {'robot': self.robot_interface, 'time': time})
        except Exception as e:
            if str(e) != "ROBOT_STOPPED":
                self.get_logger().error(f"Script Error: {e}")
        finally:
            self.code_running = False
            # Reset all targets to 0
            self.target_x = 0
            self.target_y = 0
            self.target_z = 0
            self.target_r = 0