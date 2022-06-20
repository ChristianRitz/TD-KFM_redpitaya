import os

os.chdir("/root/TD-KFM_redpitaya/build/")
os.system("cat /opt/redpitaya/fpga/fpga_0.94.bit >/dev/xdevcfg")
os.system("LD_LIBRARY_PATH=/opt/redpitaya/lib ./kfm_controller")
