#pragma once

#include <string>
#include <ros/node_handle.h>

namespace sensor_ublox_ros_tool {

struct UbloxConverterParameters {

    static UbloxConverterParameters& getInstance();

    void fromNodeHandle(const ros::NodeHandle&);

    int msg_queue_size;
    std::string verbosity;

    std::string subscriber_msg_name;
    std::string publisher_gps_msg_name;
    std::string publisher_vehicle_motion_msg_name;
    std::string publisher_time_msg_name;
    std::string publisher_imu_msg_name;
    std::string publisher_constraint_msg_name;
    std::string publisher_odometry_msg_name;

    std::string vehicle_tf_frame;
    std::string sensor_tf_frame;

    bool use_ground_plane_projection;
    bool publish_map_tf;
    bool publish_time;
    bool publish_vehiclemotion;
    bool publish_imu;
    bool publish_gps;
    bool publish_odometry;

private:
    UbloxConverterParameters();
};
} // namespace sensor_ublox_ros_tool
