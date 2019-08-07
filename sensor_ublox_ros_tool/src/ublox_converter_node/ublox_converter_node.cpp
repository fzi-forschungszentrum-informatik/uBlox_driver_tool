#include "ublox_converter.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "ublox_converter_node");

    sensor_ublox_ros_tool::UbloxConverter converter(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
