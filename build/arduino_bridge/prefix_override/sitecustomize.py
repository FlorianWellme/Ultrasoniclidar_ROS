import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/florian/Documents/ros2_ws/install/arduino_bridge'
