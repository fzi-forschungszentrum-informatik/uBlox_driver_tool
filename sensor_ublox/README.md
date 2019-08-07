# sensor_ublox

This library provides a generic interface to data sent from a u-blox M8 (e.g. M8P) device in the UBX format.

## Usage

The library requires in total three parameters to be specified:
  * The serial device path (e.g. /dev/ublox_m8)
  * A config file (see details below)
  * A base config file (should generally not be touched) that contains default settings for the device.

### M8 configuration

A sample configuration file is shown below:
```
ublox_m8_config:
  device:
    baud_rate: 38400             # should not be changed
  gnss:
    rate: 5                      # measurement frequency (1 .. 10Hz)
  messages:
    # required by ROS tool
    nav_pvt: 1                   # 1 = enabled, 0 = disabled
    nav_status: 1
    mon_msgpp: 1
    # additional messages
    #nav_sol: 1
    #nav_posecef: 1
    #nav_dop: 1
    #nav_timegps: 1
    #nav_timeutc: 1
    #nav_sat: 1
    rxm_raw: 1
  dgps:
    enable: false               # true = provides a device_path (below) that could
                                #        be used with ntrip client
                                # false = no interface for ntrip is provided
    device_path: /tmp/ublox_rtcm
```

### Sample program

A sample program to show-case the library interface is available in the sensor_ublox_tool.

