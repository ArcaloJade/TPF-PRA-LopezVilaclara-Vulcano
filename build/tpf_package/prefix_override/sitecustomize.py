import sys
if sys.prefix == '/home/facuvulcano/miniconda3/envs/rosenv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/facuvulcano/TPF-PRA-LopezVilaclara-Vulcano/install/tpf_package'
