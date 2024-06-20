#include <assert.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <sys/stat.h>
#include <spdlog/spdlog.h>

#include "driver_mpu6050_interface.h"
#include "ros2_imu/mpu6050.h"
#include "gpio_chip.h"

Mpu6050::Mpu6050(ImuConf conf)
    : ImuInterface(conf)
{
    i2c_bus_ = std::make_shared<IicBus>(imu_conf_.port);
    i2c_bus_->IicInit();
    GpioInterruptInit();
    spdlog::info("Mpu6050 Iio bus path {}", imu_conf_.port.c_str());
    mpu6050_i2c_interface_set(i2c_bus_);
}

Mpu6050::~Mpu6050()
{
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
    GpioInterruptDeinit();
    mpu6050_dmp_deinit();
    spdlog::info("Close mpu6050 device!");
}

bool Mpu6050::Init()
{
    imu_thread_ = std::thread([](Mpu6050 *p_this) { p_this->Mpu6050Loop(); }, this);
    return true;
}

void Mpu6050::Euler2Quaternion(float roll, float pitch, float yaw, Quaternion &quat)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    quat.w    = cy * cp * cr + sy * sp * sr;
    quat.x    = cy * cp * sr - sy * sp * cr;
    quat.y    = sy * cp * sr + cy * sp * cr;
    quat.z    = sy * cp * cr - cy * sp * sr;
}

int Mpu6050::GpioInterruptInit()
{
    if (!imu_conf_.int_chip.empty() && imu_conf_.int_line != -1) {
        mpu_int_ = std::make_shared<GpioChip>(imu_conf_.int_chip, imu_conf_.int_line);
        mpu_int_->AddEvent(std::bind(&Mpu6050::ReadHander, this, std::placeholders::_1, std::placeholders::_2));
        mpu_int_->Init();
    }
    return 0;
}

void Mpu6050::GpioInterruptHandler()
{
    std::unique_lock<std::mutex> lck(g_mtx_);
    uint8_t ret = mpu6050_dmp_irq_handler();
    if (!ret) {
        spdlog::error("dmp irq handler fail with code {}", ret);
    }
    g_cv_.notify_all(); // 唤醒所有线程.
}

void Mpu6050::GpioInterruptDeinit()
{
    // delete mpu_int_;
}

Imu Mpu6050::GetImuData()
{
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    return imu_data_;
}

void Mpu6050::ReadHander(const bool val, const uint64_t timestamp)
{
    if (!val) {
        GpioInterruptHandler();
    }
    if (timestamp) {}
}

void Mpu6050::Mpu6050Loop()
{
    uint16_t fifo_len = 128;
    uint32_t cnt      = 0;
    int16_t gs_accel_raw[128][3];
    int16_t gs_gyro_raw[128][3];
    float gs_accel_g[128][3];
    float gs_gyro_dps[128][3];
    int32_t gs_quat[128][4];
    float gs_pitch[128];
    float gs_roll[128];
    float gs_yaw[128];

    int ret = mpu6050_dmp_init(MPU6050_ADDRESS_AD0_LOW, ReceiveCallback,
                               DmpTapCallback, DmpOrientCallback);
    if (ret != 0) {
        spdlog::error("dmp init fail with code {}!!", ret);
        return;
    }

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif 
    {
        std::unique_lock<std::mutex> lck(g_mtx_);
        g_cv_.wait_for(lck, std::chrono::milliseconds(100));

        if (mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                 gs_gyro_raw, gs_gyro_dps,
                                 gs_quat,
                                 gs_pitch, gs_roll, gs_yaw,
                                 &fifo_len) != 0) {
            spdlog::error("dmp read all fail!!");
            // return;
            continue;
        }
        // spdlog::info("fifo len = {}", fifo_len);
        // for (uint32_t i = 0; i < fifo_len; i++) {
        //     spdlog::info("eular: ({}, {}, {})",
        //                 gs_pitch[i], gs_roll[i], gs_yaw[i]);
        //     // spdlog::info("acc ({}, {}, {})",
        //     //             gs_accel_g[i][0], gs_accel_g[i][1], gs_accel_g[i][2]);
        //     // spdlog::info("gyro ({}, {}, {})",
        //     //             gs_gyro_dps[i][0], gs_gyro_dps[i][1], gs_gyro_dps[i][2]);
        // }

        Eular euler;
        euler.pitch = gs_pitch[fifo_len];
        euler.roll  = gs_roll[fifo_len];
        euler.yaw   = gs_yaw[fifo_len];
        data_lock_.lock();
        // Euler2Quaternion(euler.roll, euler.pitch, euler.yaw, imu_data_.orientation);
        imu_data_.orientation.w         = gs_quat[0][0];
        imu_data_.orientation.x         = gs_quat[0][1];
        imu_data_.orientation.y         = gs_quat[0][2];
        imu_data_.orientation.z         = gs_quat[0][3];
        imu_data_.linear_acceleration.x = gs_accel_g[0][0];
        imu_data_.linear_acceleration.y = gs_accel_g[0][1];
        imu_data_.linear_acceleration.z = gs_accel_g[0][2];
        imu_data_.angular_velocity.x    = gs_gyro_dps[0][0] * M_PI / 180.0;
        imu_data_.angular_velocity.y    = gs_gyro_dps[0][1] * M_PI / 180.0;
        imu_data_.angular_velocity.z    = gs_gyro_dps[0][2] * M_PI / 180.0;
        data_lock_.unlock();

        /* get the pedometer step count */
        if (mpu6050_dmp_get_pedometer_counter(&cnt) != 0) {
            spdlog::error("dmp get pedometer counter fail!!");
            return;
        }
    }
    /* deinit */
    mpu6050_dmp_deinit();
    GpioInterruptDeinit();
}

void Mpu6050::ReceiveCallback(uint8_t type)
{
    switch (type) {
    case MPU6050_INTERRUPT_MOTION: {
        mpu6050_interface_debug_print("mpu6050: irq motion.\n");
        break;
    }
    case MPU6050_INTERRUPT_FIFO_OVERFLOW: {
        mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");
        break;
    }
    case MPU6050_INTERRUPT_I2C_MAST: {
        mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");
        break;
    }
    case MPU6050_INTERRUPT_DMP: {
        mpu6050_interface_debug_print("mpu6050: irq dmp\n");
        break;
    }
    case MPU6050_INTERRUPT_DATA_READY: {
        mpu6050_interface_debug_print("mpu6050: irq data ready\n");
        break;
    }
    default: {
        mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");
        break;
    }
    }
}

void Mpu6050::DmpTapCallback(uint8_t count, uint8_t direction)
{
    switch (direction) {
    case MPU6050_DMP_TAP_X_UP: {
        mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);
        break;
    }
    case MPU6050_DMP_TAP_X_DOWN: {
        mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);
        break;
    }
    case MPU6050_DMP_TAP_Y_UP: {
        mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);
        break;
    }
    case MPU6050_DMP_TAP_Y_DOWN: {
        mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);
        break;
    }
    case MPU6050_DMP_TAP_Z_UP: {
        mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);
        break;
    }
    case MPU6050_DMP_TAP_Z_DOWN: {
        mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);
        break;
    }
    default: {
        mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");
        break;
    }
    }
}

void Mpu6050::DmpOrientCallback(uint8_t orientation)
{
    switch (orientation) {
    case MPU6050_DMP_ORIENT_PORTRAIT: {
        mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");
        break;
    }
    case MPU6050_DMP_ORIENT_LANDSCAPE: {
        mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");
        break;
    }
    case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT: {
        mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");
        break;
    }
    case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE: {
        mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");
        break;
    }
    default: {
        mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");
        break;
    }
    }
}
