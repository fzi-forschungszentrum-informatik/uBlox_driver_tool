#pragma once

#include <Eigen/Eigen>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <gnss_coordinate_transform_ros/LocalGeographicCSTransformer.h>
#include <mapping_msgs_ros/PoseConstraintArray.h>
#include <mrt_sensor_msgs_ros/UBX_NAV_PVT.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ublox_converter_parameters.h"

namespace sensor_ublox_ros_tool {

class UbloxConverter {

public:
    UbloxConverter(ros::NodeHandle, ros::NodeHandle);

private:
    void process(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr&);

    ros::Publisher publisher_gps_;
    ros::Publisher publisher_imu_;
    ros::Publisher publisher_time_;
    ros::Publisher publisher_vehicle_motion_;
    ros::Publisher publisher_constraint_;
    ros::Publisher publisher_odometry_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    gnss::LocalGeographicCSTransformer localGeographicCSTransformer_;

    ros::Subscriber subscriber_;

    std::string local_cs_frame;

    UbloxConverterParameters& params_;

    double convertNEDtoENU(const double degreeNED);

    sensor_msgs::NavSatFix getGpsMsg(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox);
    sensor_msgs::TimeReference getTimeMsg(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox);
    sensor_msgs::Imu getImuMsg(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox);
    geometry_msgs::TwistWithCovarianceStamped getTwistMsg(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox);
    mapping_msgs_ros::PoseConstraintArray getConstraintMsg(const geometry_msgs::PoseWithCovarianceStamped& poseMsg);
    geometry_msgs::PoseWithCovarianceStamped getLocalCSPoseMsg(
        const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox);
    geometry_msgs::TransformStamped getMapTransform(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox);
};
} // namespace sensor_ublox_ros_tool
