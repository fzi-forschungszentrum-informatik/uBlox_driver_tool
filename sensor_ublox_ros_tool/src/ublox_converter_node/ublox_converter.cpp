#include "ublox_converter.h"
#include <automated_driving_msgs/MotionState.h>
#include <eigen_conversions/eigen_msg.h>
#include <rosinterface_handler/utilities.hpp>
#include <sensor_msgs/TimeReference.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sensor_ublox_ros_tool {

UbloxConverter::UbloxConverter(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : params_{UbloxConverterParameters::getInstance()} {

    rosinterface_handler::setLoggerLevel(private_node_handle);
    params_.fromNodeHandle(private_node_handle);

    if (params_.publish_gps)
        publisher_gps_ = private_node_handle.advertise<sensor_msgs::NavSatFix>(params_.publisher_gps_msg_name,
                                                                               params_.msg_queue_size);

    if (params_.publish_imu)
        publisher_imu_ =
            private_node_handle.advertise<sensor_msgs::Imu>(params_.publisher_imu_msg_name, params_.msg_queue_size);

    if (params_.publish_vehiclemotion)
        publisher_vehicle_motion_ = private_node_handle.advertise<automated_driving_msgs::MotionState>(
            params_.publisher_vehicle_motion_msg_name, params_.msg_queue_size);

    if (params_.publish_time)
        publisher_time_ = private_node_handle.advertise<sensor_msgs::TimeReference>(params_.publisher_time_msg_name,
                                                                                    params_.msg_queue_size);

    if (params_.publish_odometry)
        publisher_odometry_ = private_node_handle.advertise<nav_msgs::Odometry>(params_.publisher_odometry_msg_name,
                                                                                params_.msg_queue_size);

    publisher_constraint_ = private_node_handle.advertise<mapping_msgs_ros::PoseConstraintArray>(
        params_.publisher_constraint_msg_name, params_.msg_queue_size);
    subscriber_ = private_node_handle.subscribe(params_.subscriber_msg_name,
                                                params_.msg_queue_size,
                                                &UbloxConverter::process,
                                                this,
                                                ros::TransportHints().tcpNoDelay());

    node_handle.param<std::string>("local_xy_frame", local_cs_frame, "map");
    rosinterface_handler::showNodeInfo();
}

void UbloxConverter::process(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox) {
    /***
     * Publish
     */
    sensor_msgs::Imu msg_imu = getImuMsg(msg_ublox);
    geometry_msgs::TwistWithCovarianceStamped msg_twist = getTwistMsg(msg_ublox);
    geometry_msgs::PoseWithCovarianceStamped msg_pose = getLocalCSPoseMsg(msg_ublox);
    auto constraint_msg = getConstraintMsg(msg_pose);

    if (params_.publish_gps)
        publisher_gps_.publish(getGpsMsg(msg_ublox));

    if (params_.publish_imu)
        publisher_imu_.publish(msg_imu);

    if (params_.publish_time)
        publisher_time_.publish(getTimeMsg(msg_ublox));

    publisher_constraint_.publish(constraint_msg);

    if (params_.publish_vehiclemotion || params_.publish_odometry) {

        automated_driving_msgs::MotionState msg_vehiclemotion;

        msg_vehiclemotion.header.stamp = msg_ublox->header.stamp;
        msg_vehiclemotion.child_frame_id = params_.sensor_tf_frame;
        // gloabal frame
        msg_vehiclemotion.header.frame_id = msg_pose.header.frame_id;

        /**
         * Accel
         **/

        msg_vehiclemotion.accel.accel.linear = msg_imu.linear_acceleration;

        // Copy the 3x3 covariance matrix into [1-3;1-3] of the 6x6 matrix
        for (int i = 0; i < 9; ++i) {
            msg_vehiclemotion.accel.covariance[i + ((i / 3) * 3)] = msg_imu.linear_acceleration_covariance[i];
        }
        msg_vehiclemotion.accel.covariance[21] = -1;
        msg_vehiclemotion.accel.covariance[28] = -1;
        msg_vehiclemotion.accel.covariance[35] = -1;

        /**
         * Twist
         **/
        msg_vehiclemotion.twist = msg_twist.twist;

        /**
         * Pose
         */
        msg_vehiclemotion.pose = msg_pose.pose;

        if (params_.publish_vehiclemotion)
            publisher_vehicle_motion_.publish(msg_vehiclemotion);

        nav_msgs::Odometry msg_odometry;
        msg_odometry.header = msg_vehiclemotion.header;
        msg_odometry.pose = msg_vehiclemotion.pose;
        msg_odometry.twist = msg_vehiclemotion.twist;
        msg_odometry.child_frame_id = msg_vehiclemotion.child_frame_id;

        if (params_.publish_odometry)
            publisher_odometry_.publish(msg_odometry);
    }

    if (params_.publish_map_tf) {
        if (localGeographicCSTransformer_.is_initialized())
            tf_broadcaster_.sendTransform(getMapTransform(msg_ublox));
    }
}

double UbloxConverter::convertNEDtoENU(const double degreeNED) {
    double yaw = (90.0 - degreeNED) * M_PI / 180.0;
    if (yaw < -M_PI) {
        yaw += 2 * M_PI;
    }
    return yaw;
}

sensor_msgs::Imu UbloxConverter::getImuMsg(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox) {
    sensor_msgs::Imu msg_imu;
    msg_imu.header.stamp = msg_ublox->header.stamp;
    msg_imu.header.frame_id = params_.sensor_tf_frame;

    // Convert from NED to ENU
    double yaw = convertNEDtoENU(msg_ublox->pose.headMot);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0., 0., yaw);
    tf2::convert(quaternion, msg_imu.orientation);

    /// Covariance matrix needs to be specified for valid data
    ///< See: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

    // We do not have angular velocities
    msg_imu.angular_velocity.x = 0;
    msg_imu.angular_velocity.y = 0;
    msg_imu.angular_velocity.z = 0;
    msg_imu.angular_velocity_covariance[0] = -1;
    msg_imu.angular_velocity_covariance[4] = -1;
    msg_imu.angular_velocity_covariance[8] = -1;

    // We do not have linear accelerations
    msg_imu.linear_acceleration.x = 0;
    msg_imu.linear_acceleration.y = 0;
    msg_imu.linear_acceleration.z = 0;
    msg_imu.linear_acceleration_covariance[0] = -1;
    msg_imu.linear_acceleration_covariance[4] = -1;
    msg_imu.linear_acceleration_covariance[8] = -1;
    msg_imu.orientation_covariance[0] = -1;
    msg_imu.orientation_covariance[4] = -1;
    msg_imu.orientation_covariance[8] = msg_ublox->acc.headAcc * msg_ublox->acc.headAcc;

    return std::move(msg_imu);
}

geometry_msgs::TwistWithCovarianceStamped UbloxConverter::getTwistMsg(
    const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox) {
    geometry_msgs::TwistWithCovarianceStamped msg_twist;
    msg_twist.header.stamp = msg_ublox->header.stamp;
    msg_twist.header.frame_id = params_.sensor_tf_frame;

    /// Only the linear velocity is specified here.
    /// The angular is part of the IMU message.
    msg_twist.twist.twist.linear.x = msg_ublox->pose.gSpeed;
    msg_twist.twist.twist.linear.y = 0.0;
    msg_twist.twist.twist.linear.z = 0.0;
    msg_twist.twist.twist.angular.x = 0.0;
    msg_twist.twist.twist.angular.y = 0.0;
    msg_twist.twist.twist.angular.z = 0.0;
    msg_twist.twist.covariance[0] = msg_ublox->acc.sAcc * msg_ublox->acc.sAcc;
    msg_twist.twist.covariance[7] = -1;
    msg_twist.twist.covariance[14] = -1;
    msg_twist.twist.covariance[21] = -1;
    msg_twist.twist.covariance[28] = -1;
    msg_twist.twist.covariance[35] = -1;

    return std::move(msg_twist);
}

sensor_msgs::NavSatFix UbloxConverter::getGpsMsg(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox) {

    sensor_msgs::NavSatFix msg_gps;
    msg_gps.header.stamp = msg_ublox->header.stamp;
    msg_gps.header.frame_id = params_.sensor_tf_frame;

    sensor_msgs::NavSatStatus gps_status_msg;

    /**
     * http://docs.ros.org/api/sensor_msgs/html/msg/NavSatStatus.html
     * http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
     */
    gps_status_msg.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    gps_status_msg.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    msg_gps.status = gps_status_msg;

    msg_gps.latitude = msg_ublox->pose.lat;
    msg_gps.longitude = msg_ublox->pose.lon;
    msg_gps.altitude = msg_ublox->pose.height;

    msg_gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
    msg_gps.position_covariance[0] = msg_ublox->acc.hAcc * msg_ublox->acc.hAcc;
    msg_gps.position_covariance[4] = msg_ublox->acc.hAcc * msg_ublox->acc.hAcc;
    msg_gps.position_covariance[8] = msg_ublox->acc.hAcc * msg_ublox->acc.hAcc;

    return std::move(msg_gps);
}

sensor_msgs::TimeReference UbloxConverter::getTimeMsg(const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox) {
    sensor_msgs::TimeReference msg_time;

    msg_time.header.stamp = msg_ublox->header.stamp;
    ros::Time refTime;
    refTime.fromNSec(msg_ublox->time.iTOW * 1000UL);
    msg_time.time_ref = refTime;
    msg_time.source = "GPS";

    //    mrt_sensor_msgs_ros::VehicleMotion::Ptr msg_out{new mrt_sensor_msgs_ros::VehicleMotion};

    return std::move(msg_time);
}

geometry_msgs::PoseWithCovarianceStamped UbloxConverter::getLocalCSPoseMsg(
    const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox) {
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.stamp = msg_ublox->header.stamp;
    msg_pose.header.frame_id = local_cs_frame;

    double yaw = convertNEDtoENU(msg_ublox->pose.headMot);

    tf2::Quaternion quaternion_pose;
    quaternion_pose.setRPY(0, 0, yaw);
    tf2::convert(quaternion_pose, msg_pose.pose.pose.orientation);

    msg_pose.pose.covariance[0] = msg_ublox->acc.hAcc * msg_ublox->acc.hAcc;
    msg_pose.pose.covariance[7] = msg_ublox->acc.hAcc * msg_ublox->acc.hAcc;
    msg_pose.pose.covariance[14] = msg_ublox->acc.vAcc * msg_ublox->acc.vAcc;
    msg_pose.pose.covariance[21] = -1;
    msg_pose.pose.covariance[28] = -1;
    msg_pose.pose.covariance[35] = msg_ublox->acc.headAcc * msg_ublox->acc.headAcc;

    double x, y;
    if (localGeographicCSTransformer_.ll2xy(msg_ublox->pose.lat, msg_ublox->pose.lon, x, y)) {
        msg_pose.pose.pose.position.x = x;
        msg_pose.pose.pose.position.y = y;
        if (params_.use_ground_plane_projection) {
            msg_pose.pose.pose.position.z = 0;
        } else {
            msg_pose.pose.pose.position.z = msg_ublox->pose.height;
        }
    } else {
        ROS_DEBUG_STREAM_THROTTLE(5, "local_cs not initialized yet. Is initialize_origin node running?");
        // Invalidate
        msg_pose.pose.covariance[0] = -1;
        msg_pose.pose.covariance[7] = -1;
        msg_pose.pose.covariance[14] = -1;
        msg_pose.pose.covariance[21] = -1;
        msg_pose.pose.covariance[28] = -1;
        //        msg_pose.pose.covariance[35] = -1; // This value is still valid
    }

    return std::move(msg_pose);
}

mapping_msgs_ros::PoseConstraintArray UbloxConverter::getConstraintMsg(
    const geometry_msgs::PoseWithCovarianceStamped& poseMsg) {
    mapping_msgs_ros::PoseConstraintArray poseArray;
    mapping_msgs_ros::PoseConstraint poseConstr;
    poseConstr.pose = poseMsg.pose;
    poseConstr.header.stamp = poseMsg.header.stamp;
    poseConstr.header.frame_id = params_.sensor_tf_frame;
    poseConstr.id = poseMsg.header.stamp.toNSec();

    poseArray.header.stamp = poseMsg.header.stamp;
    poseArray.header.frame_id = params_.sensor_tf_frame;
    poseArray.constraints.push_back(poseConstr);
    return poseArray;
}

geometry_msgs::TransformStamped UbloxConverter::getMapTransform(
    const mrt_sensor_msgs_ros::UBX_NAV_PVT::ConstPtr& msg_ublox) {

    // LocalCS
    geometry_msgs::TransformStamped map_transform_stamped;
    map_transform_stamped.header.stamp = msg_ublox->header.stamp;
    map_transform_stamped.header.frame_id = params_.sensor_tf_frame;
    map_transform_stamped.child_frame_id = local_cs_frame;
    tf2::Transform transform;

    double yaw = convertNEDtoENU(msg_ublox->pose.headMot);

    double x, y;
    if (localGeographicCSTransformer_.ll2xy(msg_ublox->pose.lat, msg_ublox->pose.lon, x, y)) {
        if (params_.use_ground_plane_projection) {
            transform.setOrigin(tf2::Vector3(x, y, 0));
        } else {
            transform.setOrigin(tf2::Vector3(x, y, msg_ublox->pose.height));
        }
        tf2::Quaternion quaternion_pose;
        quaternion_pose.setRPY(0, 0, yaw);
        transform.setRotation(quaternion_pose);
    }

    /**
     * Transform is given from oxts -> map, so that the tf tree is consistent!
     * \attention We need the inverse transform here
     */
    transform = transform.inverse();
    map_transform_stamped.transform = tf2::toMsg(transform);
    return std::move(map_transform_stamped);
}
} // namespace sensor_ublox_ros_tool
