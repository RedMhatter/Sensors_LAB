import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rosario/Desktop/Sensors/LAB4/lab04_pkg/install/ekf_filter'
