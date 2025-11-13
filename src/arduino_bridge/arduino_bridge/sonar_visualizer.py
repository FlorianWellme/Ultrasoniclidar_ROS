#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import math

class SonarVisualizer(Node):
    def __init__(self):
        super().__init__('sonar_visualizer')

        self.angle = 0.0
        self.distance = 0.5  # valeur par défaut

        self.angle_sub = self.create_subscription(Float32, 'servo_angle', self.angle_cb, 10)
        self.range_sub = self.create_subscription(Range, 'sonar_range', self.range_cb, 10)

        self.marker_pub = self.create_publisher(Marker, 'sonar_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

    def angle_cb(self, msg):
        self.angle = msg.data

    def range_cb(self, msg):
        self.distance = msg.range

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'sonar_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sonar"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # position et orientation du cône selon angle et distance
        x = self.distance * math.cos(math.radians(self.angle))
        y = self.distance * math.sin(math.radians(self.angle))
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # orientation : rotation autour de z pour pointer vers l’angle
        import tf_transformations
        q = tf_transformations.quaternion_from_euler(0, 0, math.radians(self.angle))
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        # taille de l’arrow
        marker.scale.x = self.distance
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SonarVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
