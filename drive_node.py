import sys
import os
import rclpy

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from core.driver import GSbotDriver

def main(args=None):
    rclpy.init(args=args)
    node = GSbotDriver()
    rclpy.spin(node)
    node.running = False
    rclpy.shutdown()

if __name__ == '__main__':
    main()
