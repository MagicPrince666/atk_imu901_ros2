#include <atomic>
#include <csignal>
#include <thread>
#include <unistd.h>

#include "ros2_imu/atk_ms901m.h"
#include "ros2_imu/imu_pub.h"
#include "ros2_imu/mpu6050.h"
#include "ros2_imu/mpu9250.h"
#include "ros2_imu/zyf176ex.h"
#include <spdlog/spdlog.h>
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include "ros/ros.h"
#else
#include "rclcpp/rclcpp.hpp"
#endif

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
ImuPub::ImuPub(std::shared_ptr<ros::NodeHandle> node)
#else
ImuPub::ImuPub(std::shared_ptr<rclcpp::Node> node)
#endif
    : ros_node_(node)
{
    ImuConf conf;
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros_node_->getParam("ros2_imu_node/imu_module", conf.module);
    spdlog::info("imu_module = {}", conf.module.c_str());

    ros_node_->getParam("ros2_imu_node/imu_port", conf.port);
    spdlog::info("port = {}", conf.port.c_str());

    std::string imu_int;
    ros_node_->getParam("ros2_imu_node/imu_int", imu_int);
    spdlog::info("imu_int = {}", imu_int.c_str());

    ros_node_->getParam("ros2_imu_node/baudrate", conf.baudrate);
    spdlog::info("baudrate = {}", conf.baudrate);

    int data_len;
    ros_node_->getParam("ros2_imu_node/data_len", data_len);
    spdlog::info("data_len = {}", data_len);

    std::string topic;
    ros_node_->getParam("ros2_imu_node/topic", topic);
    spdlog::info("topic = {}", topic.c_str());

    ros_node_->getParam("ros2_imu_node/imu_frame_id", frame_id_);
    spdlog::info("frame_id = {}", frame_id_.c_str());
#else
    ros_node_->declare_parameter("imu.module", "");
    ros_node_->get_parameter("imu.module", conf.module);
    spdlog::info("imu_module = {}", conf.module.c_str());

    ros_node_->declare_parameter("imu.port", "/dev/ttyS6");
    ros_node_->get_parameter("imu.port", conf.port);
    spdlog::info("port = {}", conf.port.c_str());

    ros_node_->declare_parameter("imu.interupt.chip", "");
    ros_node_->get_parameter("imu.interupt.chip", conf.int_chip);
    spdlog::info("interupt chip = {}", conf.int_chip);

    ros_node_->declare_parameter("imu.interupt.line", -1);
    ros_node_->get_parameter("imu.interupt.line", conf.int_line);
    spdlog::info("interupt line = {}", conf.int_line);

    ros_node_->declare_parameter("imu.baudrate", 115200);
    ros_node_->get_parameter("imu.baudrate", conf.baudrate);
    spdlog::info("baudrate = {}", conf.baudrate);

    int data_len;
    ros_node_->declare_parameter("data_len", 0);
    ros_node_->get_parameter("data_len", data_len);
    spdlog::info("data_len = {}", data_len);

    std::string topic;
    ros_node_->declare_parameter("topic", "imu");
    ros_node_->get_parameter("topic", topic);
    spdlog::info("topic = {}", topic.c_str());

    ros_node_->declare_parameter("imu_frame_id", "imu");
    ros_node_->get_parameter("imu_frame_id", frame_id_);
    spdlog::info("frame_id = {}", frame_id_.c_str());
#endif

    if (conf.module == "atk") {
        imu_data_ptr_ = std::make_shared<AtkMs901m>(conf);
    } else if (conf.module == "zyz_176" || conf.module == "zyz_143") {
        imu_data_ptr_ = std::make_shared<Zyf176ex>(conf);
    } else if (conf.module == "mpu6050") {
        imu_data_ptr_ = std::make_shared<Mpu6050>(conf);
    } else if (conf.module == "mpu9250") {
        imu_data_ptr_ = std::make_shared<Mpu9250>(conf);
    } else {
        spdlog::error("{} imu is not support yet", conf.module.c_str());
    }

    if (imu_data_ptr_) {
        spdlog::info("{} imu start", conf.module.c_str());
        imu_data_ptr_->Init();
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
        imu_pub_ = std::make_shared<ros::Publisher>(ros_node_->advertise<ImuMsg>(topic, 10));
#else
        imu_pub_ = ros_node_->create_publisher<ImuMsg>(topic, 10);

        imu_timer_ = ros_node_->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ImuPub::ImuPubCallback, this));
#endif
        
    }
}

ImuPub::~ImuPub()
{
}

void ImuPub::ImuPubCallback()
{
    // 定义IMU数据
    ImuMsg imu_msg;

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
    imu_msg.header.stamp = ros::Time::now();
#else
    imu_msg.header.stamp = ros_node_->get_clock()->now();
#endif
    imu_pub_->publish(imu_msg);
}
