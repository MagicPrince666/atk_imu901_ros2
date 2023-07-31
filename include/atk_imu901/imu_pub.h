/**
 * @file imu_pub.h
 * @author 黄李全 (846863428@qq.com)
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

#include "atk_ms901m.h"

class ImuPub : public rclcpp::Node
{
public:
    ImuPub();
    ~ImuPub();

private:
    /**imu_publisher_
     * @brief IMU回调
     */
    void ImuPubCallback();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr imu_timer_;

    std::shared_ptr<AtkMs901m> atk_ms901_;
};

#endif
