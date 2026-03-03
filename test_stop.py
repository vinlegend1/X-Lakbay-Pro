import threading
import time
import sys
from pymavlink import mavutil

# 直接调用你原代码里的串口配置方法，确保能连上 Pixhawk
sys.path.append('.') # 确保能找到 utils 文件夹
try:
    from utils.hardware_config import get_serial_config
    port, baud = get_serial_config()
except Exception as e:
    print("找不到串口配置文件，使用默认 /dev/ttyACM0")
    port, baud = '/dev/ttyACM0', 115200

print(f"正在连接 Pixhawk (端口: {port}, 波特率: {baud})...")
master = mavutil.mavlink_connection(port, baud=baud)
master.wait_heartbeat(timeout=2)
print("✅ Pixhawk 连接成功！")

# 强制解锁 (ARM)，因为必须解锁马达才会转
print("正在解锁 (ARM)...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
time.sleep(1)

# 全局变量，记录当前的通道数值，初始设为全 0
current_x = 0
current_y = 0
current_z = 0

def mavlink_loop():
    """后台线程：以 10Hz 的频率疯狂向 Pixhawk 发送当前的 XYZ 数值"""
    while True:
        try:
            # Map test values to PWM: 1500 is neutral. 1000-2000 range.
            # Assuming current_x is throttle and current_y is steering.
            # Note: rc_channels_override_send uses PWM values directly (1000-2000).
            # If input values are -500 to 500, we add 1500.
            pwm_steering = int(current_y) + 1500
            pwm_throttle = int(current_x) + 1500
            
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                pwm_steering, # Ch1: Steering
                65535,        # Ch2
                pwm_throttle, # Ch3: Throttle
                65535, 65535, 65535, 65535, 65535
            )
        except Exception:
            pass
        time.sleep(0.1)

# 启动发送线程
threading.Thread(target=mavlink_loop, daemon=True).start()

print("\n" + "="*50)
print("🕹️ 探针测试已启动！")
print("请输入 X, Y, Z 三个通道的数值（用空格隔开）。")
print("例如输入: 0 750 0")
print("输入 q 退出测试并锁定马达。")
print("="*50 + "\n")

try:
    while True:
        user_input = input("👉 请输入 [X Y Z]: ").strip()
        if user_input.lower() == 'q':
            break
        
        parts = user_input.split()
        if len(parts) == 3:
            current_x, current_y, current_z = int(parts[0]), int(parts[1]), int(parts[2])
            print(f"📡 已更新发送数值 -> X: {current_x} | Y: {current_y} | Z: {current_z}")
        else:
            print("⚠️ 格式错误！请输入三个数字，例如: 0 500 0")
except KeyboardInterrupt:
    pass
finally:
    # 退出时锁定 (DISARM) 以保安全
    print("\n正在锁定 (DISARM)...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("🛑 测试结束。")
