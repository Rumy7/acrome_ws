import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rumy/acrome_ws/install/rqt_joint_trajectory_controller'
