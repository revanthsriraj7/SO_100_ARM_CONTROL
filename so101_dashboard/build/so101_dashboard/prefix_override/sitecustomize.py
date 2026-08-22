import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/revanth/so100_arm_ws/src/so101_dashboard/install/so101_dashboard'
