#include "ublox_m8.hpp"
#include <rosinterface_handler/utilities.hpp>

#include <generic_logger/generic_logger.hpp>
#include <mrt_sensor_msgs_ros/UbxNavDGPS.h>
#include <mrt_sensor_msgs_ros/UbxNavDOP.h>
#include <mrt_sensor_msgs_ros/UbxNavPOSECEF.h>
#include <mrt_sensor_msgs_ros/UbxNavSAT.h>
#include <mrt_sensor_msgs_ros/UbxNavSOL.h>
#include <mrt_sensor_msgs_ros/UbxNavSTATUS.h>
#include <mrt_sensor_msgs_ros/UbxNavTIMEGPS.h>
#include <mrt_sensor_msgs_ros/UbxNavTIMEUTC.h>
#include <mrt_sensor_msgs_ros/UbxRxmRAW.h>
#include <mrt_sensor_msgs_ros/UbxRxmSFRBX.h>
#include <std_msgs/Header.h>
#include <generic_logger/sinks/ros_sink.h>

namespace sensor_ublox_ros_tool {

UbloxM8::UbloxM8(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    /**
     * Initialization
     */
    rosinterface_handler::setLoggerLevel(private_node_handle);
    generic_logger::set_sink(std::make_shared<generic_logger::sinks::ros_sink>());
    generic_logger::set_level("debug");

    params_.fromParamServer();
    setupDiagnostics();

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&UbloxM8::reconfigureRequest, this, _1, _2));

    /**
     * Setup of GNSS receiver.
     */
    try {
        sensor_ublox::GNSSReceiverOptions gnssOpts;
        gnssOpts.deviceName = params_.serial_device_path;
        gnssOpts.configFile = params_.config_file;
        gnssOpts.baseConfigFile = params_.base_config_file;
        gnss_ = std::make_unique<sensor_ublox::GNSSReceiver>(gnssOpts);
        gnssFrequency_ = gnss_->gnssFrequency();
    } catch (sensor_ublox::GNSSException& e) {
        ROS_ERROR_STREAM("GNSS Exception: " << e.what());
    }

    /**
     * Publishers & subscriber
     */
    // NAV-PVT
    gnss_->registerCallback<sensor_ublox::UbxDataNavPVT>(
        [this](const sensor_ublox::UbxDataNavPVT& data) { this->ubxCallbackNavPVT(data); });
    pubNavPVTDiagnostic_ = std::make_unique<diagnostic_updater::DiagnosedPublisher<mrt_sensor_msgs_ros::UBX_NAV_PVT>>(
        node_handle.advertise<mrt_sensor_msgs_ros::UBX_NAV_PVT>(getTopic("ubx_nav_pvt"), params_.msg_queue_size),
        updater_,
        diagnostic_updater::FrequencyStatusParam(
            &gnssFrequency_, &gnssFrequency_, params_.diagnostic_updater_rate_tolerance, 5),
        diagnostic_updater::TimeStampStatusParam());
    // NAV-STATUS
    gnss_->registerCallback<sensor_ublox::UbxDataNavSTATUS>(
        [this](const sensor_ublox::UbxDataNavSTATUS& data) { this->ubxCallbackNavSTATUS(data); });
    pubNavSTATUS_ =
        node_handle.advertise<mrt_sensor_msgs_ros::UbxNavSTATUS>(getTopic("ubx_nav_status"), params_.msg_queue_size);
    // other messages (as per the configuration)
    const auto& enabledMessages = gnss_->getEnabledMessages();
    for (const auto& m : enabledMessages) {
        enableMessage(node_handle, m);
    }
    gnss_->run();
    rosinterface_handler::showNodeInfo();
}

void UbloxM8::enableMessage(ros::NodeHandle& nodeHandle, const std::pair<uint8_t, uint8_t>& msgID) {
    using namespace sensor_ublox;
    if (msgID.first == UbxMsgID::ClassID::Nav && msgID.second == UbxMsgID::NavMsgID::DGPS) {
        gnss_->registerCallback<UbxDataNavDGPS>(
            [this](const sensor_ublox::UbxDataNavDGPS& data) { this->ubxCallbackNavDGPS(data); });
        pubNavDGPS_ =
            nodeHandle.advertise<mrt_sensor_msgs_ros::UbxNavDGPS>(getTopic("ubx_nav_dgps"), params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Nav && msgID.second == UbxMsgID::NavMsgID::DOP) {
        gnss_->registerCallback<UbxDataNavDOP>(
            [this](const sensor_ublox::UbxDataNavDOP& data) { this->ubxCallbackNavDOP(data); });
        pubNavDOP_ =
            nodeHandle.advertise<mrt_sensor_msgs_ros::UbxNavDOP>(getTopic("ubx_nav_dop"), params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Nav && msgID.second == UbxMsgID::NavMsgID::PosECEF) {
        gnss_->registerCallback<UbxDataNavPOSECEF>(
            [this](const sensor_ublox::UbxDataNavPOSECEF& data) { this->ubxCallbackNavPOSECEF(data); });
        pubNavPOSECEF_ = nodeHandle.advertise<mrt_sensor_msgs_ros::UbxNavPOSECEF>(getTopic("ubx_nav_posecef"),
                                                                                  params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Nav && msgID.second == UbxMsgID::NavMsgID::Sat) {
        gnss_->registerCallback<UbxDataNavSAT>(
            [this](const sensor_ublox::UbxDataNavSAT& data) { this->ubxCallbackNavSAT(data); });
        pubNavSAT_ =
            nodeHandle.advertise<mrt_sensor_msgs_ros::UbxNavSAT>(getTopic("ubx_nav_sat"), params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Nav && msgID.second == UbxMsgID::NavMsgID::SBAS) {
        gnss_->registerCallback<UbxDataNavSBAS>(
            [this](const sensor_ublox::UbxDataNavSBAS& data) { this->ubxCallbackNavSBAS(data); });
        pubNavSBAS_ =
            nodeHandle.advertise<mrt_sensor_msgs_ros::UbxNavSBAS>(getTopic("ubx_nav_sbas"), params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Nav && msgID.second == UbxMsgID::NavMsgID::Sol) {
        gnss_->registerCallback<UbxDataNavSOL>(
            [this](const sensor_ublox::UbxDataNavSOL& data) { this->ubxCallbackNavSOL(data); });
        pubNavSOL_ =
            nodeHandle.advertise<mrt_sensor_msgs_ros::UbxNavSOL>(getTopic("ubx_nav_sol"), params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Nav && msgID.second == UbxMsgID::NavMsgID::TimeGPS) {
        gnss_->registerCallback<UbxDataNavTIMEGPS>(
            [this](const sensor_ublox::UbxDataNavTIMEGPS& data) { this->ubxCallbackNavTIMEGPS(data); });
        pubNavTIMEGPS_ = nodeHandle.advertise<mrt_sensor_msgs_ros::UbxNavTIMEGPS>(getTopic("ubx_nav_timegps"),
                                                                                  params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Nav && msgID.second == UbxMsgID::NavMsgID::TimeUTC) {
        gnss_->registerCallback<UbxDataNavTIMEUTC>(
            [this](const sensor_ublox::UbxDataNavTIMEUTC& data) { this->ubxCallbackNavTIMEUTC(data); });
        pubNavTIMEUTC_ = nodeHandle.advertise<mrt_sensor_msgs_ros::UbxNavTIMEUTC>(getTopic("ubx_nav_timeutc"),
                                                                                  params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Rxm && msgID.second == UbxMsgID::RxmMsgID::Rawx) {
        gnss_->registerCallback<UbxDataRxmRAWX>(
            [this](const sensor_ublox::UbxDataRxmRAWX& data) { this->ubxCallbackRxmRAWX(data); });
        pubRxmRAWX_ =
            nodeHandle.advertise<mrt_sensor_msgs_ros::UbxRxmRAW>(getTopic("ubx_rxm_raw"), params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Rxm && msgID.second == UbxMsgID::RxmMsgID::Sfrbx) {
        gnss_->registerCallback<UbxDataRxmSFRBX>(
            [this](const sensor_ublox::UbxDataRxmSFRBX& data) { this->ubxCallbackRxmSFRBX(data); });
        pubRxmSFRBX_ =
            nodeHandle.advertise<mrt_sensor_msgs_ros::UbxRxmSFRBX>(getTopic("ubx_rxm_sfrb"), params_.msg_queue_size);
    } else if (msgID.first == UbxMsgID::ClassID::Mon && msgID.second == UbxMsgID::MonMsgID::MsgPP) {
        gnss_->registerCallback<UbxDataMonMSGPP>(
            [this](const sensor_ublox::UbxDataMonMSGPP& data) { this->ubxCallbackMonMSGPP(data); });
    }
}

void UbloxM8::ubxCallbackNavPVT(const sensor_ublox::UbxDataNavPVT& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UBX_NAV_PVT>();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = params_.frame_id;

    /* time [GPST] */
    msg->time.year = data.year;
    msg->time.month = data.month;
    msg->time.day = data.day;
    msg->time.hour = data.hour;
    msg->time.min = data.min;
    msg->time.sec = data.sec;
    msg->time.iTOW = data.iTOW;

    /* pose [Position and orientation] */
    msg->pose.lon = data.lon;
    msg->pose.lat = data.lat;
    msg->pose.height = data.height;
    msg->pose.hMSL = data.hMSL;
    msg->pose.velN = data.velN;
    msg->pose.velE = data.velE;
    msg->pose.velD = data.velD;
    msg->pose.gSpeed = data.gSpeed;
    msg->pose.headMot = data.headMot;
    msg->pose.headVeh = data.headVeh;

    /* acc [Accuracy] */
    msg->acc.hAcc = data.hAcc;
    msg->acc.vAcc = data.vAcc;
    msg->acc.sAcc = data.sAcc;
    msg->acc.headAcc = data.headAcc;
    msg->acc.pDOP = data.pDOP;
    pubNavPVTDiagnostic_->publish(msg);
}

void UbloxM8::ubxCallbackRxmRAWX(const sensor_ublox::UbxDataRxmRAWX& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxRxmRAW>();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = params_.frame_id;

    msg->rcvTow = data.rcvTow;
    msg->week = data.week;
    msg->leapS = data.leapS;
    msg->recStat = data.recStat;
    msg->version = data.version;
    msg->numMeas = data.numMeas;
    msg->measurements.reserve(msg->numMeas);
    for (const auto& m : data.measurements) {
        mrt_sensor_msgs_ros::UbxRxmRAW_Measurement mea;
        mea.prMes = m.prMes;
        mea.cpMes = m.cpMes;
        mea.doMes = m.doMes;
        mea.gnssId = m.gnssId;
        mea.svId = m.svId;
        mea.freqId = m.freqId;
        mea.locktime = m.locktime;
        mea.cno = m.cno;
        mea.prStdev = m.prStdev;
        mea.cpStdev = m.cpStdev;
        mea.doStdev = m.doStdev;
        mea.trkStat = m.trkStat;
        msg->measurements.push_back(mea);
    }
    pubRxmRAWX_.publish(msg);
}

void UbloxM8::ubxCallbackRxmSFRBX(const sensor_ublox::UbxDataRxmSFRBX& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxRxmSFRBX>();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = params_.frame_id;

    msg->gnssId = data.gnssId;
    msg->svId = data.svId;
    msg->freqId = data.freqId;
    msg->version = data.version;
    msg->numWords = data.numWords;
    msg->words.resize(data.numWords);
    std::copy(data.words.cbegin(), data.words.cend(), msg->words.begin());
    pubRxmSFRBX_.publish(msg);
}

void UbloxM8::ubxCallbackNavPOSECEF(const sensor_ublox::UbxDataNavPOSECEF& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavPOSECEF>();
    msg->iTOW = data.iTOW;
    msg->ecefX = data.ecefX;
    msg->ecefY = data.ecefY;
    msg->ecefZ = data.ecefZ;
    msg->pAcc = data.pAcc;
    pubNavPOSECEF_.publish(msg);
}

void UbloxM8::ubxCallbackNavDGPS(const sensor_ublox::UbxDataNavDGPS& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavDGPS>();
    msg->iTOW = data.iTOW;
    msg->age = data.age;
    msg->baseId = data.baseId;
    msg->baseHealth = data.baseHealth;
    msg->status = data.status;
    msg->numCh = data.numCh;
    msg->channels.reserve(msg->numCh);
    for (const auto& m : data.channels) {
        mrt_sensor_msgs_ros::UbxNavDGPS_Channel ch;
        ch.svid = m.svid;
        ch.flags = m.flags;
        ch.ageC = m.ageC;
        ch.prc = m.prc;
        ch.prrc = m.prrc;
        msg->channels.push_back(ch);
    }
    pubNavDGPS_.publish(msg);
}

void UbloxM8::ubxCallbackNavDOP(const sensor_ublox::UbxDataNavDOP& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavDOP>();
    msg->iTOW = data.iTOW;
    msg->gDOP = data.gDOP;
    msg->pDOP = data.pDOP;
    msg->tDOP = data.tDOP;
    msg->vDOP = data.vDOP;
    msg->hDOP = data.hDOP;
    msg->nDOP = data.nDOP;
    msg->eDOP = data.eDOP;
    pubNavDOP_.publish(msg);
}

void UbloxM8::ubxCallbackNavSOL(const sensor_ublox::UbxDataNavSOL& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavSOL>();
    msg->iTOW = data.iTOW;
    msg->fTOW = data.fTOW;
    msg->week = data.week;
    msg->gpsFix = static_cast<uint8_t>(data.gpsFix);
    msg->flags = data.flags;
    msg->ecefX = data.ecefX;
    msg->ecefY = data.ecefY;
    msg->ecefZ = data.ecefZ;
    msg->pAcc = data.pAcc;
    msg->ecefVX = data.ecefVX;
    msg->ecefVY = data.ecefVY;
    msg->ecefVZ = data.ecefVZ;
    msg->sAcc = data.sAcc;
    msg->pDOP = data.pDOP;
    msg->numSV = data.numSV;
    pubNavSOL_.publish(msg);
};

void UbloxM8::ubxCallbackNavSAT(const sensor_ublox::UbxDataNavSAT& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavSAT>();
    msg->iTOW = data.iTOW;
    msg->numSvs = data.numSvs;
    msg->satellites.reserve(msg->numSvs);
    for (const auto& m : data.satellites) {
        mrt_sensor_msgs_ros::UbxNavSAT_Satellite sat;
        sat.gnssId = m.gnssId;
        sat.svId = m.svId;
        sat.cno = m.cno;
        sat.elev = m.elev;
        sat.azim = m.azim;
        sat.prRes = m.prRes;
        sat.flags = m.flags;
        msg->satellites.push_back(sat);
    }
    pubNavSAT_.publish(msg);
}

void UbloxM8::ubxCallbackNavSBAS(const sensor_ublox::UbxDataNavSBAS& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavSBAS>();
    msg->iTOW = data.iTOW;
    msg->geo = data.geo;
    msg->mode = data.mode;
    msg->sys = data.sys;
    msg->service = data.service;
    msg->cnt = data.cnt;

    msg->infos.reserve(msg->cnt);
    for (const auto& m : data.infos) {
        mrt_sensor_msgs_ros::UbxNavSBAS_Info info;
        info.svId = m.svId;
        info.flags = m.flags;
        info.udre = m.udre;
        info.svSys = m.svSys;
        info.svService = m.svService;
        info.prc = m.prc;
        info.ic = m.ic;
        msg->infos.push_back(info);
    }
    pubNavSBAS_.publish(msg);
}

void UbloxM8::ubxCallbackNavSTATUS(const sensor_ublox::UbxDataNavSTATUS& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavSTATUS>();
    msg->iTOW = data.iTOW;
    msg->gpsFix = static_cast<uint8_t>(data.gpsFix);
    msg->flags = data.flags;
    msg->fixStat = data.fixStat;
    msg->flags2 = data.flags2;
    msg->ttff = data.ttff;
    msg->msss = data.msss;
    pubNavSTATUS_.publish(msg);
    lastStatus_ = data;
    updater_.update();
}

void UbloxM8::ubxCallbackNavTIMEGPS(const sensor_ublox::UbxDataNavTIMEGPS& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavTIMEGPS>();
    msg->iTOW = data.iTOW;
    msg->fTOW = data.fTOW;
    msg->week = data.week;
    msg->leapS = data.leapS;
    msg->valid = data.valid;
    msg->tAcc = data.tAcc;
    pubNavTIMEGPS_.publish(msg);
}

void UbloxM8::ubxCallbackNavTIMEUTC(const sensor_ublox::UbxDataNavTIMEUTC& data) {
    auto msg = boost::make_shared<mrt_sensor_msgs_ros::UbxNavTIMEUTC>();
    msg->iTOW = data.iTOW;
    msg->tAcc = data.tAcc;
    msg->nano = data.nano;
    msg->year = data.year;
    msg->month = data.month;
    msg->day = data.day;
    msg->hour = data.hour;
    msg->min = data.min;
    msg->sec = data.sec;
    msg->valid = data.valid;
    pubNavTIMEUTC_.publish(msg);
}

void UbloxM8::ubxCallbackMonMSGPP(const sensor_ublox::UbxDataMonMSGPP& data) {
    int rtcm3 = data.rtcm3[5];
    if (rtcm3 > 0) {
        diagnosticStatusRTCM_.level = diagnostic_msgs::DiagnosticStatus::OK;
        diagnosticStatusRTCM_.message = "Received RTCM3 packages: " + std::to_string(rtcm3);
    }
    updater_.update();
}

/**
 * This callback is called whenever a change was made in the dynamic_reconfigure window
 */
void UbloxM8::reconfigureRequest(UbloxM8Config& config, uint32_t level) {
    params_.fromConfig(config);
}

/*
 * Setup the Diagnostic Updater
 */
void UbloxM8::setupDiagnostics() {
    // Give a unique hardware id
    diagnosticStatusGNSS_.hardware_id = params_.diagnostic_updater_hardware_id;
    diagnosticStatusGNSS_.message = "Waiting for solutions to become ready ...";
    diagnosticStatusGNSS_.level = diagnostic_msgs::DiagnosticStatus::STALE;

    diagnosticStatusRTCM_.hardware_id = diagnosticStatusGNSS_.hardware_id = params_.diagnostic_updater_hardware_id;
    diagnosticStatusRTCM_.message = "Received RTCM3 packages: 0";
    diagnosticStatusRTCM_.level = diagnostic_msgs::DiagnosticStatus::OK;

    updater_.setHardwareID(params_.diagnostic_updater_hardware_id);

    // Add further callbacks (or unittests) that should be called regularly
    updater_.add("GNSS Status", this, &UbloxM8::checkSensorStatusGNSS);
    updater_.add("RTCM", this, &UbloxM8::checkSensorStatusRTCM);

    updater_.force_update();
}

void UbloxM8::checkSensorStatusGNSS(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
    if (lastStatus_) {
        const auto& data = *lastStatus_;
        // update the diagnostic agent
        if (data.gpsFix == sensor_ublox::UbxDataGPSFix::FixNone) {
            status_wrapper.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            status_wrapper.message = "No fix";
        } else if (data.gpsFix == sensor_ublox::UbxDataGPSFix::FixDeadReckoningOnly) {
            status_wrapper.level = diagnostic_msgs::DiagnosticStatus::WARN;
            status_wrapper.message = "Dead reckoning only";
        } else if (data.gpsFix == sensor_ublox::UbxDataGPSFix::Fix2D) {
            status_wrapper.level = diagnostic_msgs::DiagnosticStatus::OK;
            status_wrapper.message = "2D fix";
        } else if (data.gpsFix == sensor_ublox::UbxDataGPSFix::Fix3D) {
            status_wrapper.level = diagnostic_msgs::DiagnosticStatus::OK;
            status_wrapper.message = "3D fix";
        } else if (data.gpsFix == sensor_ublox::UbxDataGPSFix::FixGPSDeadRecknoning) {
            status_wrapper.level = diagnostic_msgs::DiagnosticStatus::OK;
            status_wrapper.message = "GPS and dead reckoning combined";
        } else if (data.gpsFix == sensor_ublox::UbxDataGPSFix::FixTimeOnly) {
            status_wrapper.level = diagnostic_msgs::DiagnosticStatus::WARN;
            status_wrapper.message = "Time fix only";
        }
        if (data.flags & 0x02) {
            status_wrapper.add("DGPS", "enabled");
        }
        if (data.fixStat & 0x01) {
            status_wrapper.add("DGPS-Status: ", "PR+PRR Correction");
        }
    }
}

void UbloxM8::checkSensorStatusRTCM(diagnostic_updater::DiagnosticStatusWrapper& status_wrapper) {
    status_wrapper.summary(diagnosticStatusRTCM_);
}

} // namespace sensor_ublox_ros_tool
