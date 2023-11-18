#include "ros2_imu/mpu9250.h"
#include "driver_mpu9250_interface.h"
#include "rclcpp/rclcpp.hpp"

Mpu9250::Mpu9250(std::string type, std::string dev, uint32_t rate)
    : ImuInterface(type, dev, rate)
{
    i2c_bus_ = std::make_shared<IicBus>(imu_port_);
    RCLCPP_INFO(rclcpp::get_logger(imu_type_), "Mpu9250 Iio bus path %s", imu_port_.c_str());
    mpu9250_i2c_interface_set(i2c_bus_);
}

Mpu9250::~Mpu9250()
{
    mpu9250_dmp_deinit();
    g_gpio_irq_ = nullptr;
    GpioInterruptDeinit();
}

Imu Mpu9250::GetImuData()
{
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    return imu_data_;
}

int Mpu9250::GpioInterruptInit()
{
    // mpu_int_ = new GpioKey;
    // if (mpu_int_) {
    //     return 0;
    // }
    return 0;
}

void Mpu9250::GpioInterruptDeinit()
{
    // delete mpu_int_;
}

bool Mpu9250::Init()
{
    /* init */
    if (GpioInterruptInit() != 0) {
        return false;
    }
    g_gpio_irq_ = mpu9250_dmp_irq_handler;

    /* init */
    if (mpu9250_dmp_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_AD0_LOW, ReceiveCallback,
                         DmpTapCallback, DmpOrientCallback) != 0) {
        g_gpio_irq_ = nullptr;
        GpioInterruptDeinit();

        return false;
    }

    mpu9250_interface_delay_ms(500);
    return true;
}

void Mpu9250::Mpu9250Loop()
{
    uint32_t cnt;
    uint16_t len = 128;
    int16_t gs_accel_raw[128][3];
    float gs_accel_g[128][3];
    int16_t gs_gyro_raw[128][3];
    float gs_gyro_dps[128][3];
    int32_t gs_quat[128][4];
    float gs_pitch[128];
    float gs_roll[128];
    float gs_yaw[128];

    while (rclcpp::ok()) {
        /* read */
        if (mpu9250_dmp_read_all(gs_accel_raw, gs_accel_g,
                                 gs_gyro_raw, gs_gyro_dps,
                                 gs_quat,
                                 gs_pitch, gs_roll, gs_yaw,
                                 &len) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger(imu_type_), "dmp read all fail!!");
            return;
        }

        /* output */
        RCLCPP_INFO(rclcpp::get_logger(imu_type_), "fifo size %d.\n", len);
        RCLCPP_INFO(rclcpp::get_logger(imu_type_), "pitch[0] is %0.2fdps.\n", gs_pitch[0]);
        RCLCPP_INFO(rclcpp::get_logger(imu_type_), "roll[0] is %0.2fdps.\n", gs_roll[0]);
        RCLCPP_INFO(rclcpp::get_logger(imu_type_), "yaw[0] is %0.2fdps.\n", gs_yaw[0]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_),"acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_),"acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_),"acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_),"gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_),"gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
        // RCLCPP_INFO(rclcpp::get_logger(imu_type_),"gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);

        /* delay 500 ms */
        mpu9250_interface_delay_ms(100);

        /* get the pedometer step count */
        int res = mpu9250_dmp_get_pedometer_counter(&cnt);
        if (res != 0) {
            RCLCPP_ERROR(rclcpp::get_logger(imu_type_), "dmp get pedometer counter fail!!");
            return;
        }
    }
    mpu9250_dmp_deinit();
    g_gpio_irq_ = nullptr;
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
