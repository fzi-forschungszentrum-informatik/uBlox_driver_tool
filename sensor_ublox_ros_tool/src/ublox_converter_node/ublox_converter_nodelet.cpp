#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "ublox_converter.h"

namespace sensor_ublox_ros_tool {

class UbloxConverterNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<UbloxConverter> m_;
};

void UbloxConverterNodelet::onInit() {
    m_.reset(new UbloxConverter(getNodeHandle(), getPrivateNodeHandle()));
}
} // namespace sensor_ublox_ros_tool

PLUGINLIB_EXPORT_CLASS(sensor_ublox_ros_tool::UbloxConverterNodelet, nodelet::Nodelet);
