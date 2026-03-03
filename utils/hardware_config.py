import platform

def get_serial_config():
    """
    Returns (port, baud) based on CPU architecture detection.
    - x86_64 (Laptop/PC): Uses /dev/ttyUSB0 (External UART Bridge)
    - aarch64 (Raspberry Pi): Uses /dev/ttyAMA0 (Internal GPIO Pins)
    """
    # Get the hardware architecture (e.g., 'x86_64' or 'aarch64')
    arch = platform.machine().lower()
    
    # Defaults (Safe fallbacks)
    port = '/dev/ttyUSB0'
    baud = 57600
    
    if arch == 'x86_64':
        # Typical Laptop setup with a USB-to-Serial adapter
        port = 'udpin:0.0.0.0:14551'
        baud = 57600
    elif 'aarch64' in arch or 'arm' in arch:
        # Typical Raspberry Pi 64-bit setup (Ubuntu 24.04)
        # Note: 'arm' catch-all for 32-bit legacy Pi OS too
        port = '/dev/ttyAMA0'
        baud = 921600
            
    return port, baud

# Test it
if __name__ == "__main__":
    current_port, current_baud = get_serial_config()
    print(f"Detected Arch: {platform.machine()}")
    print(f"Using Port: {current_port} at {current_baud} baud")
