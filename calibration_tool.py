from pymavlink import mavutil
import time
import sys
from hardware_config import get_serial_config

# ==========================================
# ESC Calibration Tool (Restored from ros 2 basic control)
# ==========================================

def main():
    # 1. Connect to Pixhawk (Auto-detect port/baud)
    port, baud = get_serial_config()
    print(f"[-] Connecting to Pixhawk on {port} at {baud} baud...")
    try:
        master = mavutil.mavlink_connection(port, baud=baud)
        master.wait_heartbeat(timeout=5)
        print("[+] Connected! Battery Voltage: %s" % master.recv_match(type='SYS_STATUS', blocking=True).voltage_battery)
    except Exception as e:
        print(f"[!] Connection failed: {e}")
        return

    # 2. Safety Warning
    print("\n" + "="*40)
    print("⚠️  WARNING: MOTORS MAY SPIN  ⚠️")
    print("Ensure propellers are removed or vehicle is secured.")
    print("="*40 + "\n")
    
    confirm = input("Type 'yes' to ARM and start testing: ")
    if confirm != 'yes':
        print("Aborted.")
        return

    # 3. Arm the Vehicle
    print("[-] Arming...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("[+] ARMED! Ready for input.")

    # 4. Testing Loop
    try:
        current_z = 500 # Standard neutral start point
        
        while True:
            # Instructions
            print(f"\n[Current Z-Axis Value]: {current_z}")
            print("Enter new value (0-1000). Standard Neutral is 500.")
            print("Type 'q' to quit.")
            
            val = input("Value > ")
            if val.lower() == 'q':
                break
            
            try:
                new_z = int(val)
                if 0 <= new_z <= 1000:
                    current_z = new_z
                    print(f"--> Sending {current_z} ...")
                    
                    # Send this value repeatedly for 2 seconds to observe motor
                    # Pixhawk needs a stream of commands to stay active
                    for _ in range(20):
                        # Map current_z (0-1000) to PWM (1000-2000) for Throttle (Ch3)
                        pwm_throttle = int(current_z) + 1000
                        master.mav.rc_channels_override_send(
                            master.target_system,
                            master.target_component,
                            1500,           # Ch1: Steering (Neutral)
                            65535,          # Ch2
                            pwm_throttle,   # Ch3: Throttle
                            65535, 65535, 65535, 65535, 65535
                        )
                        time.sleep(0.1)
                else:
                    print("[!] Invalid range. Use 0-1000.")
            except ValueError:
                print("[!] Please enter a number.")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    # 5. Cleanup
    print("[-] Disarming...")
    master.arducopter_disarm()
    print("[+] Disarmed. Done.")

if __name__ == '__main__':
    main()
