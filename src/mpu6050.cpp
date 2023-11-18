#include <assert.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <sys/stat.h>

#include "driver_mpu6050_interface.h"
#include "ros2_imu/mpu6050.h"

Mpu6050::Mpu6050(std::string type, std::string dev, uint32_t rate)
    : ImuInterface(type, dev, rate)
{
    i2c_bus_ = std::make_shared<IicBus>(imu_port_);
    i2c_bus_->IicInit();
    GpioInterruptInit();
    RCLCPP_INFO(rclcpp::get_logger(imu_type_), "Mpu6050 Iio bus path %s", imu_port_.c_str());
    mpu6050_i2c_interface_set(i2c_bus_);
}

Mpu6050::~Mpu6050()
{
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
    GpioInterruptDeinit();
    mpu6050_dmp_deinit();
    g_gpio_irq_ = nullptr;
    RCLCPP_INFO(rclcpp::get_logger(imu_type_), "Close mpu6050 device!");
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
    mpu_int_ = std::make_shared<GpioKey>("/dev/input/event5");
    return 0;
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

void Mpu6050::Mpu6050Loop()
{
    g_gpio_irq_ = mpu6050_dmp_irq_handler;
    /* run dmp function */
    if (mpu6050_dmp_init(MPU6050_ADDRESS_AD0_LOW, ReceiveCallback,
                         DmpTapCallback, DmpOrientCallback) != 0) {
        g_gpio_irq_ = nullptr;
        GpioInterruptDeinit();
        RCLCPP_ERROR(rclcpp::get_logger(imu_type_), "dmp init fail!!");
        return;
    }

    /* delay 500 ms */
    mpu6050_interface_delay_ms(500);

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

    while (rclcpp::ok()) {
        /* read */
        if (mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                 gs_gyro_raw, gs_gyro_dps,
                                 gs_quat,
                                 gs_pitch, gs_roll, gs_yaw,
                                 &fifo_len) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger(imu_type_), "dmp read all fail!!");
            return;
        }

        /* output */
        RCLCPP_INFO(rclcpp::get_logger(imu_type_), "fifo %d.", fifo_len);
        RCLCPP_INFO(rclcpp::get_logger(imu_type_), "pitch: %0.2f\troll: %0.2f\tyaw: %0.2f", gs_pitch[0], gs_roll[0], gs_yaw[0]);

        // RCLCPP_INFO(rclcpp::get_logger(imu_type_), "acc x[0] is %0.2fg.", gs_accel_g[0][0]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_), "acc y[0] is %0.2fg.", gs_accel_g[0][1]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_), "acc z[0] is %0.2fg.", gs_accel_g[0][2]);

        // RCLCPP_INFO(rclcpp::get_logger(imu_type_), "gyro x[0] is %0.2fdps.", gs_gyro_dps[0][0]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_), "gyro y[0] is %0.2fdps.", gs_gyro_dps[0][1]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_), "gyro z[0] is %0.2fdps.", gs_gyro_dps[0][2]);

        Eular euler;
        euler.pitch = gs_pitch[0] * M_PI / 180.0;
        euler.roll  = gs_roll[0] * M_PI / 180.0;
        euler.yaw   = gs_yaw[0] * M_PI / 180.0;
        data_lock_.lock();
        Euler2Quaternion(euler.roll, euler.pitch, euler.yaw, imu_data_.orientation);
        imu_data_.linear_acceleration.x = gs_accel_g[0][0];
        imu_data_.linear_acceleration.y = gs_accel_g[0][1];
        imu_data_.linear_acceleration.z = gs_accel_g[0][2];
        imu_data_.angular_velocity.x    = gs_gyro_dps[0][0] * M_PI / 180.0;
        imu_data_.angular_velocity.y    = gs_gyro_dps[0][1] * M_PI / 180.0;
        imu_data_.angular_velocity.z    = gs_gyro_dps[0][2] * M_PI / 180.0;
        data_lock_.unlock();

        mpu6050_interface_delay_ms(100);

        /* get the pedometer step count */
        if (mpu6050_dmp_get_pedometer_counter(&cnt) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger(imu_type_), "dmp get pedometer counter fail!!");
            return;
        }
    }
    /* deinit */
    mpu6050_dmp_deinit();
    g_gpio_irq_ = nullptr;
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
