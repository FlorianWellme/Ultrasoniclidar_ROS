#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point
import math
import tf_transformations


class SonarVisualizer(Node):
    def __init__(self):
        super().__init__('sonar_visualizer')

        self.angle = 0.0
        self.distance = 0.5

        # Subscribers
        self.angle_sub = self.create_subscription(Float32, 'servo_angle', self.angle_cb, 10)
        self.range_sub = self.create_subscription(Range, 'sonar_range', self.range_cb, 10)

        # Publishers
        self.arrow_pub = self.create_publisher(Marker, 'sonar_arrow', 10)
        self.points_pub = self.create_publisher(Marker, 'sonar_points', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.update_visualization)

        # List of points (x,y)
        self.points = []


    def angle_cb(self, msg):
        self.angle = msg.data


    def range_cb(self, msg):
        self.distance = msg.range


    def update_visualization(self):
        # Reset si on revient à 0°
        if abs(self.angle) <= 6.0:  
            self.points = []

        self.publish_arrow()
        self.publish_points()



    # ---------- ARROW ----------
    def publish_arrow(self):
        marker = Marker()
        marker.header.frame_id = 'sonar_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sonar"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, math.radians(self.angle))
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.scale.x = self.distance
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.arrow_pub.publish(marker)


    # ---------- POINT CLOUD ----------
    def publish_points(self):
        # Ajoute un point seulement si distance < 1m
        if 0.02 < self.distance < 1.0:
            x = self.distance * math.cos(math.radians(self.angle))
            y = self.distance * math.sin(math.radians(self.angle))
            self.points.append((x, y))

        marker = Marker()
        marker.header.frame_id = "sonar_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sonar_points"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = 0.03
        marker.scale.y = 0.03

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Ajoute les points dans le Marker
        for (x, y) in self.points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.points_pub.publish(marker)



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
