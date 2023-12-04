#include <atomic>
#include <csignal>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_imu/atk_ms901m.h"
#include "ros2_imu/imu_pub.h"
#include "ros2_imu/zyf176ex.h"
#include "ros2_imu/mpu6050.h"
#include "ros2_imu/mpu9250.h"

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
ImuPub::ImuPub(std::shared_ptr<ros::NodeHandle> node)
#else
ImuPub::ImuPub(std::shared_ptr<rclcpp::Node> node)
#endif
: ros_node_(node)
{
    std::string imu_module;
    ros_node_->declare_parameter("imu_module", "");
    ros_node_->get_parameter("imu_module", imu_module);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "imu_module = %s", imu_module.c_str());

    std::string port;
    ros_node_->declare_parameter("imu_port", "/dev/ttyS6");
    ros_node_->get_parameter("imu_port", port);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "port = %s", port.c_str());

    std::string imu_int;
    ros_node_->declare_parameter("imu_int", "");
    ros_node_->get_parameter("imu_int", imu_int);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "imu_int = %s", imu_int.c_str());

    int baudrate;
    ros_node_->declare_parameter("baudrate", 115200);
    ros_node_->get_parameter("baudrate", baudrate);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "baudrate = %d", baudrate);

    int data_len;
    ros_node_->declare_parameter("data_len", 0);
    ros_node_->get_parameter("data_len", data_len);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "data_len = %d", data_len);

    std::string topic;
    ros_node_->declare_parameter("topic", "imu");
    ros_node_->get_parameter("topic", topic);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "topic = %s", topic.c_str());

    ros_node_->declare_parameter("imu_frame_id", "imu");
    ros_node_->get_parameter("imu_frame_id", frame_id_);
    RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "frame_id = %s", frame_id_.c_str());

    if (imu_module == "atk") {
        imu_data_ptr_ = std::make_shared<AtkMs901m>(imu_module, port, baudrate);
    } else if (imu_module == "zyz_176" || imu_module == "zyz_143") {
        imu_data_ptr_ = std::make_shared<Zyf176ex>(imu_module, port, baudrate);
    } else if (imu_module == "mpu6050") {
        imu_data_ptr_ = std::make_shared<Mpu6050>(imu_module, port, baudrate);
    } else if (imu_module == "mpu9250") {
        imu_data_ptr_ = std::make_shared<Mpu9250>(imu_module, port, baudrate);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(__FUNCTION__), "%s imu is not support yet", imu_module.c_str());
    }

    if (imu_data_ptr_) {
        RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "%s imu start", imu_module.c_str());
        imu_data_ptr_->Init();

        imu_pub_   = ros_node_->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
        imu_timer_ = ros_node_->create_wall_timer(
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
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    imu_msg.header.stamp    = ros::Time::now();
#else
    imu_msg.header.stamp    = ros_node_->get_clock()->now();
#endif
    imu_pub_->publish(imu_msg);
}
