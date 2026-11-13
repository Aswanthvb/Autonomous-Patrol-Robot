import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aswanth-v-b/patrol_ws/install/patrol_task_manager'
