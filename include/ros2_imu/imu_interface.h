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

typedef struct
{
    float roll  = 0.0; /* 横滚角，单位：rad */
    float pitch = 0.0; /* 俯仰角，单位：rad */
    float yaw   = 0.0; /* 航向角，单位：rad */
} Eular;

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
} Vector3;

typedef struct {
    Quaternion orientation;      // 姿态
    Vector3 angular_velocity;    // 角速度
    Vector3 linear_acceleration; // 线加速度
    Eular eular;                 // 欧拉角
} Imu;

typedef struct {
    std::string module;
    std::string port;
    std::string int_chip;
    int int_line;
    int baudrate;
} ImuConf;

class ImuInterface
{
public:
    ImuInterface(ImuConf conf)
        : imu_conf_(conf) {}
    virtual ~ImuInterface() {}

    virtual bool Init() = 0;

    virtual Imu GetImuData() = 0;

protected:
    ImuConf imu_conf_;
};

#endif
