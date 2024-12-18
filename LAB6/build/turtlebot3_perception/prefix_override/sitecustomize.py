import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rosario/Desktop/Sensors/LAB6/install/turtlebot3_perception'
