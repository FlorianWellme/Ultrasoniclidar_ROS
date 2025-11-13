#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import serial
import sys

class SonarBridge(Node):
    def __init__(self):
        super().__init__('sonar_bridge')

        # Port série
        port = '/dev/ttyACM0'
        if len(sys.argv) > 1:
            port = sys.argv[1]

        self.ser = serial.Serial(port, 115200, timeout=1)

        # Publishers ROS2
        self.angle_pub = self.create_publisher(Float32, 'servo_angle', 10)
        self.range_pub = self.create_publisher(Range, 'sonar_range', 10)

        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info(f'SonarBridge node started and listening on {port}')

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line or ',' not in line:
                return

            parts = line.split(',')
            if len(parts) != 2:
                return

            angle = float(parts[0])
            distance_cm = float(parts[1])

            # Publie l’angle
            angle_msg = Float32()
            angle_msg.data = angle
            self.angle_pub.publish(angle_msg)

            # Publie la distance sous forme de message Range
            range_msg = Range()
            range_msg.header.frame_id = 'sonar_frame'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.26
            range_msg.min_range = 0.02
            range_msg.max_range = 4.0
            range_msg.range = distance_cm / 100.0  # conversion cm → m
            self.range_pub.publish(range_msg)

            self.get_logger().info(f"Angle={angle:.1f}°, Distance={distance_cm:.1f} cm")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SonarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

