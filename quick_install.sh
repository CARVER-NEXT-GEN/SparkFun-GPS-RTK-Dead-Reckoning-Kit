#!/bin/bash

INSTALL_DIR="$(pwd)/src/carver_gps/carver_gps/Qwiic_Ublox_Gps_Py-update_lib_v1.2.0"
echo "Installing Qwiic UbloxGps Python package . . ."

pip install spidev
pip install $INSTALL_DIR




