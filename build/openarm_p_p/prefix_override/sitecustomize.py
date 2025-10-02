import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/endika/Documents/openarm_ros2/install/openarm_p_p'
