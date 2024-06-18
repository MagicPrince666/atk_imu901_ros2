#include "ros2_imu/mpu9250.h"
#include <spdlog/spdlog.h>
#include "driver_mpu9250_interface.h"
#include "gpio_chip.h"
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include "ros/ros.h"
#else
#include <rclcpp/rclcpp.hpp>
#endif

Mpu9250::Mpu9250(ImuConf conf)
    : ImuInterface(conf)
{
    i2c_bus_ = std::make_shared<IicBus>(imu_conf_.port);
    i2c_bus_->IicInit();
    GpioInterruptInit();
    spdlog::info("Mpu9250 Iio bus path {}", imu_conf_.port.c_str());
    mpu9250_i2c_interface_set(i2c_bus_);
}

Mpu9250::~Mpu9250()
{
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
    GpioInterruptDeinit();
    mpu9250_dmp_deinit();

    spdlog::info("Close Mpu9250 device!");
}

Imu Mpu9250::GetImuData()
{
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    return imu_data_;
}

int Mpu9250::GpioInterruptInit()
{
    mpu_int_ = std::make_shared<GpioChip>(imu_conf_.int_chip, imu_conf_.int_line);
    mpu_int_->Init();
    return 0;
}

void Mpu9250::GpioInterruptHandler()
{
    std::unique_lock<std::mutex> lck(g_mtx_);
    uint8_t ret = mpu9250_dmp_irq_handler();
    if (!ret) {
       spdlog::error("dmp irq handler fail with code {}", ret);
    }
    g_cv_.notify_all(); // 唤醒所有线程.
}

void Mpu9250::GpioInterruptDeinit()
{
    // delete mpu_int_;
}

bool Mpu9250::Init()
{
    imu_thread_ = std::thread([](Mpu9250 *p_this) { p_this->Mpu9250Loop(); }, this);
    return true;
}

void Mpu9250::ReadHander(const bool val, const uint64_t timestamp)
{
    if (!val) {
        GpioInterruptHandler();
    }
    if (timestamp) {}
}

void Mpu9250::Mpu9250Loop()
{
    uint32_t cnt;
    uint16_t fifo_len = 128;
    int16_t gs_accel_raw[128][3];
    float gs_accel_g[128][3];
    int16_t gs_gyro_raw[128][3];
    float gs_gyro_dps[128][3];
    int32_t gs_quat[128][4];
    float gs_pitch[128];
    float gs_roll[128];
    float gs_yaw[128];

    mpu_int_->AddEvent(std::bind(&Mpu9250::ReadHander, this, std::placeholders::_1, std::placeholders::_2));

    /* init */
    int ret = mpu9250_dmp_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_AD0_LOW, ReceiveCallback,
                               DmpTapCallback, DmpOrientCallback);
    if (ret != 0) {
        spdlog::warn("dmp init fail with code {}!!", ret);
        return;
    }

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif 
    {
        std::unique_lock<std::mutex> lck(g_mtx_);
        g_cv_.wait_for(lck, std::chrono::milliseconds(500));

        if (mpu9250_dmp_read_all(gs_accel_raw, gs_accel_g,
                                 gs_gyro_raw, gs_gyro_dps,
                                 gs_quat,
                                 gs_pitch, gs_roll, gs_yaw,
                                 &fifo_len) != 0) {
            spdlog::error("dmp read all fail!!");
            return;
        }

        for (uint32_t i = 0; i < fifo_len; i++) {
            spdlog::info("eular: ({}, {}, {})",
                     gs_pitch[i], gs_roll[i], gs_yaw[i]);
            spdlog::info("acc ({}, {}, {})",
                     gs_accel_g[i][0], gs_accel_g[i][1], gs_accel_g[i][2]);
            spdlog::info("gyro ({}, {}, {})",
                     gs_gyro_dps[i][0], gs_gyro_dps[i][1], gs_gyro_dps[i][2]);
        }

        /* get the pedometer step count */
        int res = mpu9250_dmp_get_pedometer_counter(&cnt);
        if (res != 0) {
            spdlog::warn("dmp get pedometer counter fail!!");
            return;
        }
    }
    mpu9250_dmp_deinit();
    GpioInterruptDeinit();
}

void Mpu9250::ReceiveCallback(uint8_t type)
{
    switch (type) {
    case MPU9250_INTERRUPT_MOTION: {
        mpu9250_interface_debug_print("mpu9250: irq motion.\n");
        break;
    }
    case MPU9250_INTERRUPT_FIFO_OVERFLOW: {
        mpu9250_interface_debug_print("mpu9250: irq fifo overflow.\n");
        break;
    }
    case MPU9250_INTERRUPT_FSYNC_INT: {
        mpu9250_interface_debug_print("mpu9250: irq fsync int.\n");
        break;
    }
    case MPU9250_INTERRUPT_DMP: {
        mpu9250_interface_debug_print("mpu9250: irq dmp\n");
        break;
    }
    case MPU9250_INTERRUPT_DATA_READY: {
        mpu9250_interface_debug_print("mpu9250: irq data ready\n");
        break;
    }
    default: {
        mpu9250_interface_debug_print("mpu9250: irq unknown code.\n");
        break;
    }
    }
}

void Mpu9250::DmpTapCallback(uint8_t count, uint8_t direction)
{
    switch (direction) {
    case MPU9250_DMP_TAP_X_UP: {
        mpu9250_interface_debug_print("mpu9250: tap irq x up with %d.\n", count);
        break;
    }
    case MPU9250_DMP_TAP_X_DOWN: {
        mpu9250_interface_debug_print("mpu9250: tap irq x down with %d.\n", count);
        break;
    }
    case MPU9250_DMP_TAP_Y_UP: {
        mpu9250_interface_debug_print("mpu9250: tap irq y up with %d.\n", count);
        break;
    }
    case MPU9250_DMP_TAP_Y_DOWN: {
        mpu9250_interface_debug_print("mpu9250: tap irq y down with %d.\n", count);
        break;
    }
    case MPU9250_DMP_TAP_Z_UP: {
        mpu9250_interface_debug_print("mpu9250: tap irq z up with %d.\n", count);
        break;
    }
    case MPU9250_DMP_TAP_Z_DOWN: {
        mpu9250_interface_debug_print("mpu9250: tap irq z down with %d.\n", count);
        break;
    }
    default: {
        mpu9250_interface_debug_print("mpu9250: tap irq unknown code.\n");
        break;
    }
    }
}

void Mpu9250::DmpOrientCallback(uint8_t orientation)
{
    switch (orientation) {
    case MPU9250_DMP_ORIENT_PORTRAIT: {
        mpu9250_interface_debug_print("mpu9250: orient irq portrait.\n");

        break;
    }
    case MPU9250_DMP_ORIENT_LANDSCAPE: {
        mpu9250_interface_debug_print("mpu9250: orient irq landscape.\n");

        break;
    }
    case MPU9250_DMP_ORIENT_REVERSE_PORTRAIT: {
        mpu9250_interface_debug_print("mpu9250: orient irq reverse portrait.\n");

        break;
    }
    case MPU9250_DMP_ORIENT_REVERSE_LANDSCAPE: {
        mpu9250_interface_debug_print("mpu9250: orient irq reverse landscape.\n");

        break;
    }
    default: {
        mpu9250_interface_debug_print("mpu9250: orient irq unknown code.\n");

        break;
    }
    }
}
