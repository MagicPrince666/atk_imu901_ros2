#include <atomic>
#include <csignal>
#include <thread>
#include <unistd.h>

#include "ros2_imu/imu_pub.h"
#include "ros2_imu/atk_ms901m.h"
#include "ros2_imu/zyf176ex.h"
#include "rclcpp/rclcpp.hpp"

ImuPub::ImuPub() : rclcpp::Node("imu901m")
{
    std::string port;
    this->declare_parameter("imu_port", "/dev/ttyS6");
    this->get_parameter("imu_port", port);
    RCLCPP_INFO(this->get_logger(), "port = %s", port.c_str());

    int baudrate;
    this->declare_parameter("baudrate", 115200);
    this->get_parameter("baudrate", baudrate);
    RCLCPP_INFO(this->get_logger(), "baudrate = %d", baudrate);

    std::string topic;
    this->declare_parameter("topic", "imu");
    this->get_parameter("topic", topic);
    RCLCPP_INFO(this->get_logger(), "topic = %s", topic.c_str());

    this->declare_parameter("imu_frame_id", "imu");
    this->get_parameter("imu_frame_id", frame_id_);
    RCLCPP_INFO(this->get_logger(), "frame_id = %s", frame_id_.c_str());

    atk_ms901_ = std::make_shared<AtkMs901m>(port, baudrate);
    atk_ms901_->Init();

    imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ImuPub::ImuPubCallback, this));
}

ImuPub::~ImuPub()
{
}

void ImuPub::ImuPubCallback()
{
    // 定义IMU数据
    sensor_msgs::msg::Imu imu_msg;

    if (atk_ms901_) {
        Imu get_imu = atk_ms901_->GetImuData();

        imu_msg.orientation.w         = get_imu.orientation.w;
        imu_msg.orientation.x         = get_imu.orientation.x;
        imu_msg.orientation.y         = get_imu.orientation.y;
        imu_msg.orientation.z         = get_imu.orientation.z;
        imu_msg.angular_velocity.x    = get_imu.angular_velocity.x;
        imu_msg.angular_velocity.y    = get_imu.angular_velocity.y;
        imu_msg.angular_velocity.z    = get_imu.angular_velocity.z;
        imu_msg.linear_acceleration.x = get_imu.linear_acceleration.x;
        imu_msg.linear_acceleration.y = get_imu.linear_acceleration.y;
        imu_msg.linear_acceleration.z = get_imu.linear_acceleration.z;
    }

    for (size_t i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i]         = 0;
        imu_msg.angular_velocity_covariance[i]    = 0;
        imu_msg.linear_acceleration_covariance[i] = 0;
    }

    imu_msg.header.frame_id = frame_id_;
    imu_msg.header.stamp    = this->get_clock()->now();
    imu_pub_->publish(imu_msg);
}
