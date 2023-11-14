#include <atomic>
#include <csignal>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_imu/atk_ms901m.h"
#include "ros2_imu/imu_pub.h"
#include "ros2_imu/zyf176ex.h"

ImuPub::ImuPub() : rclcpp::Node("imu901m")
{
    std::string imu_module;
    this->declare_parameter("imu_module", "");
    this->get_parameter("imu_module", imu_module);
    RCLCPP_INFO(this->get_logger(), "imu_module = %s", imu_module.c_str());

    std::string port;
    this->declare_parameter("imu_port", "/dev/ttyS6");
    this->get_parameter("imu_port", port);
    RCLCPP_INFO(this->get_logger(), "port = %s", port.c_str());

    int baudrate;
    this->declare_parameter("baudrate", 115200);
    this->get_parameter("baudrate", baudrate);
    RCLCPP_INFO(this->get_logger(), "baudrate = %d", baudrate);

    int data_len;
    this->declare_parameter("data_len", 0);
    this->get_parameter("data_len", data_len);
    RCLCPP_INFO(this->get_logger(), "data_len = %d", data_len);

    std::string topic;
    this->declare_parameter("topic", "imu");
    this->get_parameter("topic", topic);
    RCLCPP_INFO(this->get_logger(), "topic = %s", topic.c_str());

    this->declare_parameter("imu_frame_id", "imu");
    this->get_parameter("imu_frame_id", frame_id_);
    RCLCPP_INFO(this->get_logger(), "frame_id = %s", frame_id_.c_str());

    if (imu_module == "atk") {
        imu_data_ptr_ = std::make_shared<AtkMs901m>(imu_module, port, baudrate);
    } else if (imu_module == "zyz_176" || imu_module == "zyz_143") {
        imu_data_ptr_ = std::make_shared<Zyf176ex>(imu_module, port, baudrate);
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s imu is not support yet", imu_module.c_str());
    }
    if (imu_data_ptr_) {
        RCLCPP_INFO(this->get_logger(), "%s imu start", imu_module.c_str());
        imu_data_ptr_->Init();

        imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
        imu_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ImuPub::ImuPubCallback, this));
    }
}

ImuPub::~ImuPub()
{
}

void ImuPub::ImuPubCallback()
{
    // 定义IMU数据
    sensor_msgs::msg::Imu imu_msg;

    if (imu_data_ptr_) {
        Imu get_imu = imu_data_ptr_->GetImuData();

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
