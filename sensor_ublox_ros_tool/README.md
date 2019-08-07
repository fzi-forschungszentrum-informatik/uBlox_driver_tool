# sensor_ublox_tool

This tool provides a ROS wrapper for the sensor_ublox interface library.

## usage

The ROS tool is already configured for all vehicles to work out-of-the-box for plain GPS and DGPS. In case
modifications are required, the ROS tool like the library accepts three options:
* The serial device path
* The configuration
* The base configuration
* The base name for topics, e.g. /sensor/ublox/...

Please check the repository [sensor_ublox](https://gitlab.mrt.uni-karlsruhe.de/MRT/sensor_ublox) for details.

## Topics

By default, the following topics are published by the raw sensor node:
* /sensor/ublox/ubx_nav_pvt (Position, Velocity and Time)
* /sensor/ublox/ubx_nav_status (GPS Status data)
* /sensor/ublox/ubx_rxm_raw (Raw measurements)

When using the default launch files (sensor_ublox.launch and sensor_ublox_dgps.launch), a additional converter node
is launched providing ROS standard messages, like NavSatFix, etc:
* /sensor/ublox/gps (sensor_msgs/NavSatFix)
* /sensor/ublox/imu (sensor_msgs/Imu)
* /sensor/ublox/time (sensor_msgs/TimeReference)
* /sensor/ublox/vehicle_motion (mrt_sensor_msgs/VehicleMotion)

