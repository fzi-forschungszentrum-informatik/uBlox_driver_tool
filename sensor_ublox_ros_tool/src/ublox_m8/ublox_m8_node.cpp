#include "ublox_m8.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "ublox_m8_node");

    sensor_ublox_ros_tool::UbloxM8 converter(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
