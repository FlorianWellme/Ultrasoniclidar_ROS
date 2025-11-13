# Ultrasoniclidar_ROS
Create a lidar with ultrasonic sensor and servomotor on ROS2 and RVIZ

Prerequisites:
- ROS 2 Humble installed
- Arduino Mega with HC-SR04 ultrasonic sensor and servo motor
- ROS2 workspace (ros2_ws) containing the arduino_bridge package

Workspace setup:
# cd ~/Documents/ros2_ws
# colcon build
# source /opt/ros/humble/setup.bash
# source install/setup.bash

Running the project:
1. Start reading Arduino data (servo + ultrasonic):
ros2 run arduino_bridge sonar_bridge /dev/ttyACM0
   - Publishes:
     - /servo_angle : Float32 (angle of the servo in degrees)
     - /sonar_range : Range (distance in meters)

2. Start visualization in RViz:
ros2 run arduino_bridge sonar_visualizer
   - Displays a rotating cone representing the measured distance
   - In RViz, add a Marker display for topic /sonar_marker
   - Set Fixed Frame to sonar_frame

Notes:
- Verify the correct serial port with: ls /dev/ttyACM*
- If you have permission errors: sudo usermod -a -G dialout $USER && sudo reboot
- Rebuild workspace with colcon build after modifying scripts or adding launch files
