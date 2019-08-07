#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "ublox_m8.hpp"

namespace sensor_ublox_ros_tool {

class UbloxM8Nodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<UbloxM8> m_;
};

void UbloxM8Nodelet::onInit() {
    m_.reset(new UbloxM8(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace sensor_ublox_ros_tool

PLUGINLIB_EXPORT_CLASS(sensor_ublox_ros_tool::UbloxM8Nodelet, nodelet::Nodelet);
