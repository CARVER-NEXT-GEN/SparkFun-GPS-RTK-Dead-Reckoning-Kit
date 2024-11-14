#!/bin/bash

INSTALL_DIR="$(pwd)/src/rtk/rtk/Qwiic_Ublox_Gps_Py-update_lib_v1.2.0"
echo "Installing Qwiic UbloxGps Python package . . ."

pip install $INSTALL_DIR

colcon build && source install/setup.bash

echo "source $(pwd)/install/setup.bash" >> ~/.bashrc



