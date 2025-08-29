import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xiatenghui/work_space/mujoco_ws/install/ur5e_description'
