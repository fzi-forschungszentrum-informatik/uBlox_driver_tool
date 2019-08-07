# UBLOX DRIVER TOOL

This library provides a generic interface to data sent from a u-blox M8 (e.g. M8P) device in the UBX format including a ros wrapper and a demo tool.

## Installation

As usual with Catkin, after you have sourced the ros installation, you have to create a workspace and clone all required packages there. Then you can build.
```shell
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir catkin_ws && cd catkin_ws && mkdir src
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo # build in release mode (or whatever you prefer)
cd src
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
git clone https://github.com/fzi-forschungszentrum-informatik/uBlox_driver_tool.git
cd ..
catkin build
```
