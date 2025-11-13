#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import sys

class ServoBridge(Node):
    def __init__(self):
        super().__init__('servo_bridge')

        # Ouvre la liaison série avec la Mega
        port = '/dev/ttyACM0'
        if len(sys.argv) > 1:
            port = sys.argv[1]

        self.ser = serial.Serial(port, 115200, timeout=1)

        # Publisher pour l'angle du servo
        self.angle_pub = self.create_publisher(Float32, 'servo_angle', 10)

        # Timer pour lire les données série
        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info(f'✅ ServoBridge node started and listening on {port}')

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            angle = float(line)

            msg = Float32()
            msg.data = angle
            self.angle_pub.publish(msg)

            self.get_logger().info(f"Angle={angle:.1f}°")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# This script is a ROS2 node that reads servo angles from an Arduino Mega via serial communication
