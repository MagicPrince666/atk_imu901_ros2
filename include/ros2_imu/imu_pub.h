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

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

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

private:
    /**imu_publisher_
     * @brief IMU回调
     */
    void ImuPubCallback();
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    std::shared_ptr<ros::NodeHandle> ros_node_;
#else
    std::shared_ptr<rclcpp::Node> ros_node_;
#endif
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr imu_timer_;

    std::shared_ptr<ImuInterface> imu_data_ptr_;
    std::string frame_id_;
};

#endif
