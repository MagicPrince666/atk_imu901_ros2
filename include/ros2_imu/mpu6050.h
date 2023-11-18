/**
 * @file mpu6050.h
 * @author Leo Huang (846863428@qq.com)
 * @brief 陀螺仪数据获取及解算
 * @version 0.1
 * @date 2023-02-02
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */
#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "driver_mpu6050_dmp.h"
#include "imu_interface.h"
#include "iic.h"
#include "gpio_key.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <unordered_map>
#include <condition_variable>
#include <vector>
#include <mutex>
#include <thread>

class Mpu6050 : public ImuInterface
{
public:
    Mpu6050(std::string type, std::string dev, uint32_t rate);
    ~Mpu6050();

    bool Init();

    Imu GetImuData();

private:
    std::shared_ptr<GpioKey> mpu_int_;
    std::shared_ptr<IicBus> i2c_bus_;
    std::thread imu_thread_;
    std::mutex data_lock_;
    Imu imu_data_;
    std::condition_variable g_cv_; // 全局条件变量
    std::mutex g_mtx_;             // 全局互斥锁.

    void Euler2Quaternion(float roll, float pitch, float yaw, Quaternion &quat);

    int GpioInterruptInit();

    void GpioInterruptHandler();

    void GpioInterruptDeinit();

    void Mpu6050Loop();

    static void ReceiveCallback(uint8_t type);
    static void DmpTapCallback(uint8_t count, uint8_t direction);
    static void DmpOrientCallback(uint8_t orientation);
};

#endif
