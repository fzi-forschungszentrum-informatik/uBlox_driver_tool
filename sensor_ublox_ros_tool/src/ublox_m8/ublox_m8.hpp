#pragma once

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <boost/optional.hpp>
#include <mrt_sensor_msgs_ros/UBX_NAV_PVT.h>
#include <mrt_sensor_msgs_ros/UbxNavSBAS.h>
#include <mrt_sensor_msgs_ros/UbxNavSBAS_Info.h>
#include <sensor_ublox/data_types.hpp>
#include <sensor_ublox/ublox.hpp>

#include "sensor_ublox_ros_tool/UbloxM8Interface.h"

namespace sensor_ublox_ros_tool {

class UbloxM8 {
public:
    UbloxM8(ros::NodeHandle, ros::NodeHandle);

private:
    std::string getTopic(const std::string& topicName) {
        return this->params_.msg_base_name + "/" + topicName;
    };

    std::unique_ptr<sensor_ublox::GNSSReceiver> gnss_;

    std::unique_ptr<diagnostic_updater::DiagnosedPublisher<mrt_sensor_msgs_ros::UBX_NAV_PVT>> pubNavPVTDiagnostic_;
    ros::Publisher pubNavPOSECEF_;
    ros::Publisher pubNavSTATUS_;
    ros::Publisher pubNavDOP_;
    ros::Publisher pubNavSOL_;
    ros::Publisher pubNavTIMEGPS_;
    ros::Publisher pubNavTIMEUTC_;
    ros::Publisher pubNavDGPS_;
    ros::Publisher pubNavSAT_;
    ros::Publisher pubNavSBAS_;
    ros::Publisher pubRxmRAWX_;
    ros::Publisher pubMonMSGPP_;
    ros::Publisher pubRxmSFRBX_;

    dynamic_reconfigure::Server<UbloxM8Config> reconfigSrv_; // Dynamic reconfiguration service

    UbloxM8Interface params_;

    double gnssFrequency_;

    /// Diagnostics
    diagnostic_updater::Updater updater_;
    diagnostic_msgs::DiagnosticStatus diagnosticStatusGNSS_;
    diagnostic_msgs::DiagnosticStatus diagnosticStatusRTCM_;
    boost::optional<sensor_ublox::UbxDataNavSTATUS> lastStatus_;

    void setupDiagnostics();
    void checkSensorStatusGNSS(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper);
    void checkSensorStatusRTCM(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper);
    void diagnostic_msg(diagnostic_updater::DiagnosticStatusWrapper&);
    void diagnoseError();

    void reconfigureRequest(UbloxM8Config&, uint32_t);

    void ubxCallbackNavPVT(const sensor_ublox::UbxDataNavPVT& data);
    void ubxCallbackNavPOSECEF(const sensor_ublox::UbxDataNavPOSECEF& data);
    void ubxCallbackNavSTATUS(const sensor_ublox::UbxDataNavSTATUS& data);
    void ubxCallbackNavDOP(const sensor_ublox::UbxDataNavDOP& data);
    void ubxCallbackNavSOL(const sensor_ublox::UbxDataNavSOL& data);
    void ubxCallbackNavTIMEGPS(const sensor_ublox::UbxDataNavTIMEGPS& data);
    void ubxCallbackNavTIMEUTC(const sensor_ublox::UbxDataNavTIMEUTC& data);
    void ubxCallbackNavDGPS(const sensor_ublox::UbxDataNavDGPS& data);
    void ubxCallbackNavSAT(const sensor_ublox::UbxDataNavSAT& data);
    void ubxCallbackNavSBAS(const sensor_ublox::UbxDataNavSBAS& data);
    void ubxCallbackRxmRAWX(const sensor_ublox::UbxDataRxmRAWX& data);
    void ubxCallbackMonMSGPP(const sensor_ublox::UbxDataMonMSGPP& data);
    void ubxCallbackRxmSFRBX(const sensor_ublox::UbxDataRxmSFRBX& data);

    void enableMessage(ros::NodeHandle& nodeHandle, const std::pair<uint8_t, uint8_t>& msgID);
};

} // namespace sensor_ublox_ros_tool
