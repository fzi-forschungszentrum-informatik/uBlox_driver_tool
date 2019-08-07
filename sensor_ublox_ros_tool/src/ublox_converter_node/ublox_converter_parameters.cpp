#include "ublox_converter_parameters.h"

#include <utils_ros/node_handle.hpp>

namespace sensor_ublox_ros_tool {

UbloxConverterParameters& UbloxConverterParameters::getInstance() {

    static UbloxConverterParameters p;
    return p;
}

void UbloxConverterParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {
    using namespace utils_ros;

    getParam(node_handle, "verbosity", verbosity);
    getParam(node_handle, "msg_queue_size", msg_queue_size);

    getParam(node_handle, "subscriber_msg_name", subscriber_msg_name);
    getParam(node_handle, "publisher_gps_msg_name", publisher_gps_msg_name);
    getParam(node_handle, "publisher_vehicle_motion_msg_name", publisher_vehicle_motion_msg_name);
    getParam(node_handle, "publisher_time_msg_name", publisher_time_msg_name);
    getParam(node_handle, "publisher_imu_msg_name", publisher_imu_msg_name);
    getParam(node_handle, "publisher_constraint_msg_name", publisher_constraint_msg_name);
    getParam(node_handle, "publisher_odometry_msg_name", publisher_odometry_msg_name);

    getParam(node_handle, "vehicle_tf_frame", vehicle_tf_frame);
    getParam(node_handle, "sensor_tf_frame", sensor_tf_frame);

    getParam(node_handle, "use_ground_plane_projection", use_ground_plane_projection);
    getParam(node_handle, "publish_map_tf", publish_map_tf);
    getParam(node_handle, "publish_time", publish_time);
    getParam(node_handle, "publish_vehiclemotion", publish_vehiclemotion);
    getParam(node_handle, "publish_gps", publish_gps);
    getParam(node_handle, "publish_imu", publish_imu);
    getParam(node_handle, "publish_imu", publish_odometry);
}

UbloxConverterParameters::UbloxConverterParameters() {
}
} // namespace sensor_ublox_ros_tool
