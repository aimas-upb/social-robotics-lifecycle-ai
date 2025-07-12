import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/adrianpana/tiago_ws/src/bt_node/install/bt_node'
