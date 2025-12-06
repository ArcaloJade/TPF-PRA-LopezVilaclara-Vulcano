import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alumno1/Documents/TPF-PRA-LopezVilaclara-Vulcano/install/tpf_navigation'
