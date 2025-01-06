import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tim/SparkFun-GPS-RTK-Dead-Reckoning-Kit/src/rtk/rtk/Qwiic_Ublox_Gps_Py-update_lib_v1.2.0/install/sparkfun_ublox_gps'
