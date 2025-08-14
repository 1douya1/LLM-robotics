import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wenhao/uf_custom_ws/src/install/easy_handeye2'
