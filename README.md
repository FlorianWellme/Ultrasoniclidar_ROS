# Ultrasoniclidar_ROS
Create a lidar-like scanner using an ultrasonic sensor (HC-SR04) mounted on a servomotor, visualized in ROS2 and RViz2.


## PREREQUISITES

- ROS 2 Humble installed
- Arduino Mega or Arduino Uno with:
  - Servomotor (SG90, MG996R, etc.)
  - Ultrasonic sensor HC-SR04 mounted on the servo
- A ROS2 workspace named ros2_ws
- The ROS2 package: arduino_bridge


## WORKSPACE SETUP

1. Open a terminal and go inside your workspace:
   ```cd ~/Documents/ros2_ws```

2. Source ROS 2:
   ```source /opt/ros/humble/setup.bash```

3. Build the workspace:
   ```colcon build```

4. Source the workspace environment:
   ```source install/setup.bash```


## RUNNING THE PROJECT


1. Start reading Arduino data (servo angle + ultrasonic distance):
   ```
   ros2 run arduino_bridge sonar_bridge /dev/ttyACM0
   ```

   This node publishes:
   - /servo_angle → Float32 (servo angle in degrees)
   - /sonar_range → sensor_msgs/Range (distance measurement in meters)

2. Start the visualization node:
   ```
   ros2 run arduino_bridge sonar_visualizer
   ```


## RVIZ2 CONFIGURATION


1. Launch RViz2:
   ```rviz2```

2. In RViz2:
   - Set “Fixed Frame” to: sonar_frame

3. Add the point cloud display:
   - Click “Add”
   - Choose “By topic”
   - Select: /sonar_points → MarkerArray

4. (Optional) Display the sonar direction arrow:
   - Add → By topic → /sonar_arrow → Marker

5. You should now see:
   - A rotating arrow showing the sensor direction
   - A set of points (< 1m) representing detected obstacles
   - A 180° sweep creating a 2D map-like visualization


## UTILITIES


Check which serial port your Arduino uses:
   ```ls /dev/ttyACM*```

If you get a “permission denied” error on /dev/ttyACM0:
   ```
   sudo usermod -a -G dialout $USER
   sudo reboot
   ```

Rebuild after modifying Python scripts:
   ```
   colcon build
   source install/setup.bash
   ```


