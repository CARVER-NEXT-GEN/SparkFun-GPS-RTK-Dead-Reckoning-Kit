# SparkFun-GPS-RTK-Dead-Reckoning-Kit

# Introduction

-   This repository provides a Python-based implementation for integrating the SparkFun GPS-RTK Dead Reckoning Kit with ROS2, allowing you to publish GPS and dead reckoning data directly into your ROS2 ecosystem. Leveraging the SparkFun Qwiic Ublox GPS Python library, this setup enables high-accuracy position data output, making it suitable for robotics, autonomous navigation, and any applications that require reliable location information even in GPS-limited environments.

## Features

- **GPS-RTK Positioning**: High-precision GPS data utilizing Real-Time Kinematics (RTK) corrections for enhanced accuracy.
- **Dead Reckoning Support**: Continues to estimate position when GPS data is unavailable by combining onboard sensors.
- **ROS2 Compatibility**: Publish GPS and dead reckoning data as ROS2 messages, making it easy to integrate with other ROS2 nodes and packages.

## Prerequisites

- **SparkFun GPS-RTK Dead Reckoning Kit**: Ensure your hardware is connected and configured correctly.
- **Python Library Dependencies**: The SparkFun Qwiic Ublox GPS library is required. Installation instructions are included below.

---

## Installation

To set up the SparkFun GPS-RTK Dead Reckoning Kit with ROS2, follow these steps:

1. **Clone the SparkFun Qwiic Ublox GPS Python Library**:  
   Begin by cloning the required Python library into your working directory.

   ```bash
   git clone https://github.com/sparkfun/Qwiic_Ublox_Gps_Py.git
   ```

2. Navigate to the Dead Reckoning Kit Directory:
    Change directories to the main setup folder.

    ```bash
    cd SparkFun-GPS-RTK-Dead-Reckoning-Kit
    ```

3. Run the Quick Install Script:
    Execute the quick installation script to install necessary dependencies and set up the environment.

    ```bash
    . quick_install.sh
    ```

## Usage

- Connect the SparkFun GPS-RTK Dead Reckoning Breakout - ZED-F9R, SMA (Qwiic) and check the port by using the command below.
   ```bash
    ls /dev/tty*
    ```

- Find your device port and change the permission of it. (Assume that ttyACM0 for this package)
   ```bash
    sudo chmod 666 /dev/ttyACM0
    ```

   **If it's the difference port, you have to change at script file:    code /SparkFun-GPS-RTK-Dead-Reckoning-Kit/src/rtk/scripts/RTK_GPS.py and change the port at self.port**


- To start publishing data from the SparkFun GPS-RTK Dead Reckoning Kit, use the following command to run the node:

    ```bash
    ros2 run rtk RTK_GPS.py
    ```

- Once the node is running, you can check the list of active topics to verify that data is being published. Use the command below to see the available topics:

    ```bash
    ros2 topic list
    ```


- This will display the published topics, allowing you to inspect and subscribe to the data being output by the GPS-RTK Dead Reckoning Kit.