/**
 * @file imu_interface.h
 * @author Leo Huang (846863428@qq.com)
 * @brief 串口IMU
 * @version 0.1
 * @date 2023-10-21
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __IMU_INTERFACE_H__
#define __IMU_INTERFACE_H__

#include <iostream>

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;
} Quaternion;

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
} Vector3;

typedef struct {
    Quaternion orientation;                     // 姿态
    Vector3 angular_velocity;                   // 角速度
    Vector3 linear_acceleration;                // 线加速度
} Imu;

class ImuInterface
{
public:
    ImuInterface(std::string port, uint32_t rate)
        : imu_port_(port), baud_rate_(rate) {}
    virtual ~ImuInterface() {}

    virtual bool Init() = 0;

    virtual Imu GetImuData() = 0;

protected:
    std::string imu_port_;
    uint32_t baud_rate_;
};

#endif
