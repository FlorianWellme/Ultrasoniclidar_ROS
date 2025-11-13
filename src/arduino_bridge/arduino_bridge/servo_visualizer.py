#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

class ServoVisualizer(Node):
    def __init__(self):
        super().__init__('servo_visualizer')
        self.angle = 0.0

        # Subscriber sur le topic servo_angle
        self.create_subscription(Float32, 'servo_angle', self.angle_callback, 10)

        # Publisher pour RViz
        self.marker_pub = self.create_publisher(Marker, 'servo_marker', 10)

        # Timer pour publier régulièrement
        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info('ServoVisualizer started — listening on /servo_angle')

    def angle_callback(self, msg):
        self.angle = msg.data

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "servo"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position du servo (origine)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # Orientation selon l’angle reçu
        yaw = math.radians(self.angle)
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        # Taille et couleur
        marker.scale.x = 1.0  # longueur de la flèche
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ServoVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
