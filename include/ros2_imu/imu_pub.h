/**
 * @file imu_pub.h
 * @author Leo Huang (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-07-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */

#ifndef __IMU_PUB_H__
#define __IMU_PUB_H__

#include <memory>

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#endif

#include "imu_interface.h"

class ImuPub
{
public:
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ImuPub(std::shared_ptr<ros::NodeHandle> node);
#else
    ImuPub(std::shared_ptr<rclcpp::Node> node);
#endif
    ~ImuPub();

    /**imu_publisher_
     * @brief IMU回调
     */
    void ImuPubCallback();

private:
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    using ImuMsg = sensor_msgs::Imu;
    std::shared_ptr<ros::NodeHandle> ros_node_;
    std::shared_ptr<ros::Publisher> imu_pub_;
    ros::Timer imu_timer_;
#else
    using ImuMsg = sensor_msgs::msg::Imu;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
#endif

    std::shared_ptr<ImuInterface> imu_data_ptr_;
    std::string frame_id_;
};

#endif
