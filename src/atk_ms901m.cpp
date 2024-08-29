/**
 ****************************************************************************************************
 * @file        atk_ms901m.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS901M模块驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "ros2_imu/atk_ms901m.h"
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include "ros/ros.h"
#else
#include <rclcpp/rclcpp.hpp>
#endif

#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <unistd.h>

AtkMs901m::AtkMs901m(ImuConf conf)
    : ImuInterface(conf)
{
    atk_ms901m_fsr_.gyro          = 0x03;
    atk_ms901m_fsr_.accelerometer = 0x01;
    /* ATK-MS601M UART初始化 */
    // 创建通讯部件工厂,这一步可以优化到从lunch配置文件选择初始化不同的通讯部件工厂
    std::shared_ptr<CommFactory> factory(new SerialComm());
    // 通过工厂方法创建通讯产品
    std::shared_ptr<Communication> serial(factory->CreateCommTarget(imu_conf_.port, imu_conf_.baudrate, false));
    serial_comm_ = serial;
}

AtkMs901m::~AtkMs901m()
{
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
}

bool AtkMs901m::Init()
{
    // SetBaudRate(0x03); // 230400
    // SaveToFlash();
    // /* 获取ATK-MS901M陀螺仪满量程 */
    ReadRegById(ATK_MS901M_FRAME_ID_REG_GYROFSR);
    // /* 获取ATK-MS901M加速度计满量程 */
    ReadRegById(ATK_MS901M_FRAME_ID_REG_ACCFSR);
    // 获取回传率
    ReadRegById(ATK_MS901M_FRAME_ID_REG_RETURNRATE);
    // 获取陀螺仪带宽
    ReadRegById(ATK_MS901M_FRAME_ID_REG_GYROBW);
    // 获取加速度计带宽
    ReadRegById(ATK_MS901M_FRAME_ID_REG_ACCBW);
    // 获取UART通讯波特率
    ReadRegById(ATK_MS901M_FRAME_ID_REG_BAUD);

    SetPortMode(ATK_MS901M_PORT_D0, ATK_MS901M_PORT_MODE_ANALOG_INPUT);
    SetPortMode(ATK_MS901M_PORT_D1, ATK_MS901M_PORT_MODE_OUTPUT_PWM);
    SetPortMode(ATK_MS901M_PORT_D2, ATK_MS901M_PORT_MODE_ANALOG_INPUT);
    SetPortMode(ATK_MS901M_PORT_D3, ATK_MS901M_PORT_MODE_OUTPUT_PWM);
    SetPortPwmPulse(ATK_MS901M_PORT_D1, 1000);
    SetPortPwmPeriod(ATK_MS901M_PORT_D1, 20000);
    SetPortPwmPulse(ATK_MS901M_PORT_D3, 1000);
    SetPortPwmPeriod(ATK_MS901M_PORT_D3, 20000);
    imu_thread_ = std::thread([](AtkMs901m *p_this) { p_this->ImuReader(); }, this);

    return true;
}

std::string AtkMs901m::Bytes2String(uint8_t *data, uint32_t len)
{
    char temp[512];
    std::string str("");
    for (size_t i = 0; i < len; i++) {
        snprintf(temp, sizeof(temp), "%02x ", data[i]);
        str.append(temp);
    }
    return str;
}

void AtkMs901m::ReadBuffer(const uint8_t *buffer, const int length)
{
    std::unique_lock<std::mutex> lck(g_mtx_);
    int32_t buf_size = sizeof(atk_ms901m_buffer_.rx_buffer) - atk_ms901m_buffer_.size;
    if (buf_size >= length) {
        memcpy(atk_ms901m_buffer_.rx_buffer + atk_ms901m_buffer_.size, buffer, length);
        atk_ms901m_buffer_.size += length; // 更新buff长度
    } else {
        memcpy(atk_ms901m_buffer_.rx_buffer + atk_ms901m_buffer_.size, buffer, buf_size);
        atk_ms901m_buffer_.size += buf_size; // 更新buff长度
    }
    g_cv_.notify_all(); // 唤醒所有线程.
}

void AtkMs901m::ImuReader()
{
    serial_comm_->AddCallback(std::bind(&AtkMs901m::ReadBuffer, this, std::placeholders::_1, std::placeholders::_2));
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    while (ros::ok())
#else
    while (rclcpp::ok())
#endif
    {
        std::unique_lock<std::mutex> lck(g_mtx_);
        g_cv_.wait_for(lck, std::chrono::milliseconds(100));
        for (uint32_t i = 0; atk_ms901m_buffer_.size >= 6; i++) {
            uint8_t *ros_rx_buffer_ptr  = atk_ms901m_buffer_.rx_buffer;
            uint32_t index              = 0;
            atk_ms901m_frame_t *res_tmp = SearchHearLE(ros_rx_buffer_ptr, atk_ms901m_buffer_.size, index);
            if (res_tmp == nullptr) {
                // 已经处理完所有可识别的包
                std::cerr << "not found buffer head size = " << atk_ms901m_buffer_.size << std::endl;
                break;
            } else {
                // 重置指针位置指向包头位置和更新长度

                uint8_t lenght = atk_ms901m_buffer_.size - index;
                if (lenght >= (res_tmp->len + 5)) {
                    ros_rx_buffer_ptr += index;
                    atk_ms901m_buffer_.size = lenght;
                } else {
                    // RCLCPP_WARN(rclcpp::get_logger(__FUNCTION__), "not a full buffer size = %d", atk_ms901m_buffer_.size);
                    break;
                }

                imu_frame_           = *res_tmp;
                imu_frame_.check_sum = res_tmp->dat[res_tmp->len];

                uint8_t sum = imu_frame_.head_l + imu_frame_.head_h + imu_frame_.id + imu_frame_.len;
                for (uint32_t i = 0; i < imu_frame_.len; i++) {
                    sum += imu_frame_.dat[i];
                }
                if (sum == imu_frame_.check_sum) {
                    uint32_t buf_len = imu_frame_.len + 5;
                    // RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "buffer = %s", Bytes2String(ros_rx_buffer_ptr, buf_len).c_str());
                    // RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "check_sum = %X, sum =%X ", imu_frame_.check_sum,sum);
                    atk_ms901m_buffer_.size -= buf_len;
                    uint8_t buffer[sizeof(atk_ms901m_buffer_.rx_buffer)];
                    // 剩余未处理数据拷贝到临时变量
                    memcpy(buffer, ros_rx_buffer_ptr + buf_len, atk_ms901m_buffer_.size);
                    // 覆盖掉原来的buff
                    memcpy(atk_ms901m_buffer_.rx_buffer, buffer, atk_ms901m_buffer_.size);
                } else {
                    // RCLCPP_WARN(rclcpp::get_logger(__FUNCTION__), "check_sum = %X, sum =%X ", imu_frame_.check_sum,sum);
                    std::cerr << "Check sum fail and delete it later " << std::endl;
                    uint32_t buf_len = imu_frame_.len + 5;
                    atk_ms901m_buffer_.size -= buf_len;
                    uint8_t buffer[sizeof(atk_ms901m_buffer_.rx_buffer)];
                    memcpy(buffer, ros_rx_buffer_ptr + buf_len, atk_ms901m_buffer_.size);
                    memcpy(atk_ms901m_buffer_.rx_buffer, buffer, atk_ms901m_buffer_.size);
                    continue;
                }

                if (imu_frame_.head_h == ATK_MS901M_FRAME_HEAD_UPLOAD_H) {
                    std::lock_guard<std::mutex> mylock_guard(data_lock_);
                    switch (imu_frame_.id) {
                    case ATK_MS901M_FRAME_ID_ATTITUDE /* 姿态角 */: {
                        imu_data_.eular.roll  = *(int16_t *)(imu_frame_.dat) / 32768.0 * M_PI;
                        imu_data_.eular.pitch = *(int16_t *)(imu_frame_.dat + 2) / 32768.0 * M_PI;
                        imu_data_.eular.yaw   = *(int16_t *)(imu_frame_.dat + 4) / 32768.0 * M_PI;
                        // RCLCPP_INFO(rclcpp::get_logger(__FUNCTION__), "roll = %lf  pitch = %lf yaw = %lf", imu_data_.eular.roll, imu_data_.eular.pitch, imu_data_.eular.yaw);
                    } break;

                    case ATK_MS901M_FRAME_ID_QUAT /* 四元数 */: {
                        imu_data_.orientation.w = *(int16_t *)(imu_frame_.dat) / 32768.0;
                        imu_data_.orientation.x = *(int16_t *)(imu_frame_.dat + 2) / 32768.0;
                        imu_data_.orientation.y = *(int16_t *)(imu_frame_.dat + 4) / 32768.0;
                        imu_data_.orientation.z = *(int16_t *)(imu_frame_.dat + 6) / 32768.0;
                    } break;

                    case ATK_MS901M_FRAME_ID_GYRO_ACCE /* 陀螺仪，加速度计 */: {
                        imu_data_.linear_acceleration.x = *(int16_t *)(imu_frame_.dat) / 32768.0 * atk_ms901m_accelerometer_fsr_table_[atk_ms901m_fsr_.accelerometer] * 9.8;
                        imu_data_.linear_acceleration.y = *(int16_t *)(imu_frame_.dat + 2) / 32768.0 * atk_ms901m_accelerometer_fsr_table_[atk_ms901m_fsr_.accelerometer] * 9.8;
                        imu_data_.linear_acceleration.z = *(int16_t *)(imu_frame_.dat + 4) / 32768.0 * atk_ms901m_accelerometer_fsr_table_[atk_ms901m_fsr_.accelerometer] * 9.8;
                        imu_data_.angular_velocity.x    = *(int16_t *)(imu_frame_.dat + 6) / 32768.0 * atk_ms901m_gyro_fsr_table_[atk_ms901m_fsr_.gyro] * M_PI / 180.0;
                        imu_data_.angular_velocity.y    = *(int16_t *)(imu_frame_.dat + 8) / 32768.0 * atk_ms901m_gyro_fsr_table_[atk_ms901m_fsr_.gyro] * M_PI / 180.0;
                        imu_data_.angular_velocity.z    = *(int16_t *)(imu_frame_.dat + 10) / 32768.0 * atk_ms901m_gyro_fsr_table_[atk_ms901m_fsr_.gyro] * M_PI / 180.0;
                    } break;

                    case ATK_MS901M_FRAME_ID_MAG /* 磁力计 */: {
                        magnetometer_.x =  *(int16_t *)(imu_frame_.dat);
                        magnetometer_.y =  *(int16_t *)(imu_frame_.dat + 2);
                        magnetometer_.z =  *(int16_t *)(imu_frame_.dat + 4);
                        magnetometer_.temperature = (float)(*(int16_t *)(imu_frame_.dat + 6)) / 100.0;
                    } break;

                    case ATK_MS901M_FRAME_ID_BARO /* 气压计 */: {
                        barometer_.pressure = *(int32_t *)(imu_frame_.dat);
                        barometer_.altitude = *(int32_t *)(imu_frame_.dat + 4);
                        barometer_.temperature = (float)(*(int16_t *)(imu_frame_.dat + 8)) / 100.0;
                    } break;

                    case ATK_MS901M_FRAME_ID_PORT /* 端口 */: {
                        port_dat_.d0 = *(uint16_t *)(imu_frame_.dat);
                        port_dat_.d1 = *(uint16_t *)(imu_frame_.dat + 2);
                        port_dat_.d2 = *(uint16_t *)(imu_frame_.dat + 4);
                        port_dat_.d3 = *(uint16_t *)(imu_frame_.dat + 6);
                    } break;

                    default:
                        break;
                    }
                } else if (imu_frame_.head_h == ATK_MS901M_FRAME_HEAD_ACK_H) {
                    switch (imu_frame_.id) {
                    case ATK_MS901M_FRAME_ID_REG_SENSTA /* 读取传感器校准状态 */: {
                        if (imu_frame_.dat[0] & 0x01) {
                            std::cout << "accelerometer cailbrated" << std::endl;
                        } else {
                            std::cout << "accelerometer not cailbrated" << std::endl;
                        }
                        if (imu_frame_.dat[0] & 0x02) {
                            std::cout << "magnetometer cailbrated" << std::endl;
                        } else {
                            std::cout << "magnetometer not cailbrated" << std::endl;
                        }
                        if (imu_frame_.dat[0] & 0x04) {
                            std::cout << "gyro cailbrated" << std::endl;
                        } else {
                            std::cout << "gyro not cailbrated" << std::endl;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_GYROFSR /* 获取ATK-MS901M陀螺仪满量程 */: {
                        if (imu_frame_.dat[0] < 6) {
                            atk_ms901m_fsr_.gyro = imu_frame_.dat[0];
                            std::cout << "full gyro = " << atk_ms901m_gyro_fsr_table_[atk_ms901m_fsr_.gyro] << std::endl;
                        } else {
                            std::cerr << "get imu gyro fail" << imu_frame_.dat[0] << std::endl;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_ACCFSR /* 获取ATK-MS901M加速度计满量程 */: {
                        if (imu_frame_.dat[0] < 4) {
                            atk_ms901m_fsr_.accelerometer = imu_frame_.dat[0];
                            std::cout << "full accelerometer = " << static_cast<uint32_t>(atk_ms901m_accelerometer_fsr_table_[atk_ms901m_fsr_.accelerometer]) << std::endl;
                        } else {
                            std::cerr << "get imu accelerometer fail" << imu_frame_.dat[0] << std::endl;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_GYROBW /* 设置陀螺仪带宽 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "gyro bitwidth = 176" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "gyro bitwidth = 92" << std::endl;
                            break;
                        case 0x02:
                            std::cout << "gyro bitwidth = 41" << std::endl;
                            break;
                        case 0x03:
                            std::cout << "gyro bitwidth = 20" << std::endl;
                            break;
                        case 0x04:
                            std::cout << "gyro bitwidth = 10" << std::endl;
                            break;
                        case 0x05:
                            std::cout << "gyro bitwidth = 5" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_ACCBW /* 设置加速度计带宽 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "accelerometer bitwidth = 218" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "accelerometer bitwidth = 99" << std::endl;
                            break;
                        case 0x02:
                            std::cout << "accelerometer bitwidth = 45" << std::endl;
                            break;
                        case 0x03:
                            std::cout << "accelerometer bitwidth = 21" << std::endl;
                            break;
                        case 0x04:
                            std::cout << "accelerometer bitwidth = 10" << std::endl;
                            break;
                        case 0x05:
                            std::cout << "accelerometer bitwidth = 5" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_BAUD /* 设置UART通讯波特率 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "uart baud rate = 921600" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "uart baud rate = 460800" << std::endl;
                            break;
                        case 0x02:
                            std::cout << "uart baud rate = 256000" << std::endl;
                            break;
                        case 0x03:
                            std::cout << "uart baud rate = 230400" << std::endl;
                            break;
                        case 0x04:
                            std::cout << "uart baud rate = 115200" << std::endl;
                            break;
                        case 0x05:
                            std::cout << "uart baud rate = 57600" << std::endl;
                            break;
                        case 0x06:
                            std::cout << "uart baud rate = 38400" << std::endl;
                            break;
                        case 0x07:
                            std::cout << "uart baud rate = 19200" << std::endl;
                            break;
                        case 0x08:
                            std::cout << "uart baud rate = 9600" << std::endl;
                            break;
                        case 0x09:
                            std::cout << "uart baud rate = 4800" << std::endl;
                            break;
                        case 0x0A:
                            std::cout << "uart baud rate = 2400" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_RETURNSET /* 设置回传内容 */: {
                        if (imu_frame_.dat[0] & 0x01) {
                            std::cout << "posture angle upload" << std::endl;
                        } else {
                            std::cout << "posture angle not upload" << std::endl;
                        }
                        if (imu_frame_.dat[0] & 0x02) {
                            std::cout << "quadrany upload" << std::endl;
                        } else {
                            std::cout << "quadrany not upload" << std::endl;
                        }
                        if (imu_frame_.dat[0] & 0x04) {
                            std::cout << "accelerometer upload" << std::endl;
                        } else {
                            std::cout << "accelerometer not upload" << std::endl;
                        }
                        if (imu_frame_.dat[0] & 0x08) {
                            std::cout << "magnetometer upload" << std::endl;
                        } else {
                            std::cout << "magnetometer not upload" << std::endl;
                        }
                        if (imu_frame_.dat[0] & 0x10) {
                            std::cout << "barometer upload" << std::endl;
                        } else {
                            std::cout << "barometer not upload" << std::endl;
                        }
                        if (imu_frame_.dat[0] & 0x20) {
                            std::cout << "port data upload" << std::endl;
                        } else {
                            std::cout << "port data not upload" << std::endl;
                        }
                        if (imu_frame_.dat[0] & 0x40) {
                            std::cout << "upper computer data upload" << std::endl;
                        } else {
                            std::cout << "upper computer data not upload" << std::endl;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_RETURNRATE /* 设置回传速率 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "return rate = 250Hz" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "return rate = 200Hz" << std::endl;
                            break;
                        case 0x02:
                            std::cout << "return rate = 125Hz" << std::endl;
                            break;
                        case 0x03:
                            std::cout << "return rate = 100Hz" << std::endl;
                            break;
                        case 0x04:
                            std::cout << "return rate = 50Hz" << std::endl;
                            break;
                        case 0x05:
                            std::cout << "return rate = 20Hz" << std::endl;
                            break;
                        case 0x06:
                            std::cout << "return rate = 10Hz" << std::endl;
                            break;
                        case 0x07:
                            std::cout << "return rate = 5Hz" << std::endl;
                            break;
                        case 0x08:
                            std::cout << "return rate = 2Hz" << std::endl;
                            break;
                        case 0x09:
                            std::cout << "return rate = 1Hz" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_ALG /* 设置算法 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "imu601" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "imu901" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_ASM /* 设置安装方向 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00: // 水平安装
                            std::cout << "horizontal installation" << std::endl;
                            break;
                        case 0x01: // 垂直安装
                            std::cout << "vertical installation" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_GAUCAL /* 设置陀螺仪自校准开关 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "enable gyro self-calibration after power on" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "disable gyro self-calibration after power on" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_BAUCAL /* 设置气压计自校准开关 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "enable barometer self-calibration after power on" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "disable barometer self-calibration after power on" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_LEDOFF /* 设置LED开关 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "enable led" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "disable led" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_D0MODE /* 设置端口D0模式 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "port 0 analog input" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "port 0 digital input" << std::endl;
                            break;
                        case 0x02:
                            std::cout << "port 0 digital output high" << std::endl;
                            break;
                        case 0x03:
                            std::cout << "port 0 digital output low" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_D1MODE /* 设置端口D1模式 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "port 1 analog input" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "port 1 digital input" << std::endl;
                            break;
                        case 0x02:
                            std::cout << "port 1 digital output high" << std::endl;
                            break;
                        case 0x03:
                            std::cout << "port 1 digital output low" << std::endl;
                            break;
                        case 0x04:
                            std::cout << "port 1 pwm output" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_D2MODE /* 设置端口D2模式 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "port 2 analog input" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "port 2 digital input" << std::endl;
                            break;
                        case 0x02:
                            std::cout << "port 2 digital output high" << std::endl;
                            break;
                        case 0x03:
                            std::cout << "port 2 digital output low" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_D3MODE /* 设置端口D3模式 */: {
                        switch (imu_frame_.dat[0])
                        {
                        case 0x00:
                            std::cout << "port 3 analog input" << std::endl;
                            break;
                        case 0x01:
                            std::cout << "port 3 digital input" << std::endl;
                            break;
                        case 0x02:
                            std::cout << "port 3 digital output high" << std::endl;
                            break;
                        case 0x03:
                            std::cout << "port 3 digital output low" << std::endl;
                            break;
                        case 0x04:
                            std::cout << "port 3 pwm output" << std::endl;
                            break;
                        
                        default:
                            break;
                        }
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_D1PULSE /* 设置端口D1 PWM高电平脉宽 */: {
                        port1_pulse_ = *(uint16_t *)(imu_frame_.dat);
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_D3PULSE /* 设置端口D3 PWM高电平脉宽 */: {
                        port3_pulse_ = *(uint16_t *)(imu_frame_.dat);
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_D1PERIOD /* 设置端口D1 PWM周期 */: {
                        port1_period_ = *(uint16_t *)(imu_frame_.dat);
                    } break;

                    case ATK_MS901M_FRAME_ID_REG_D3PERIOD /* 设置端口D3 PWM周期 */: {
                        port3_period_ = *(uint16_t *)(imu_frame_.dat);
                    } break;

                    default:
                        break;
                    }
                } else {
                    std::cout << "unknow head = " << std::hex << imu_frame_.head_l << imu_frame_.head_h << std::endl;
                }
            }
        }
    }
}

Imu AtkMs901m::GetImuData()
{
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    return imu_data_;
}

atk_ms901m_frame_t *AtkMs901m::SearchHearLE(uint8_t *data, uint32_t total_len, uint32_t &index)
{
    atk_ms901m_frame_t *res_tmp = nullptr;
    for (uint32_t i = 0; i < total_len; i++) {
        // 剩余长度大于一个包长度，说明还有协议数据包可以处理
        if ((total_len - i) >= 6) {
            // 一个字节一个字节的偏移，直到查找到协议头
            res_tmp = (atk_ms901m_frame_t *)(data + i);
            // 找到协议头
            if (res_tmp->head_l == ATK_MS901M_FRAME_HEAD_L) {
                if (res_tmp->head_h == ATK_MS901M_FRAME_HEAD_UPLOAD_H || res_tmp->head_h == ATK_MS901M_FRAME_HEAD_ACK_H) {
                    index = i; // 记录偏移地址
                    break;
                }
            }
        } else {
            return nullptr;
        }
    }
    return res_tmp;
}

/**
 * @brief       通过指定帧ID获取接收到的数据帧
 * @param       frame  : 接收到的数据帧
 *              id     : 指定帧ID
 *              id_type: 帧ID类型，
 *                       ATK_MS901M_FRAME_ID_TYPE_UPLOAD: ATK-MS901M主动上传帧ID
 *                       ATK_MS901M_FRAME_ID_TYPE_ACK   : ATK-MS901M应答帧ID
 *              timeout: 等待数据帧最大超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK     : 获取指定数据帧成功
 *              ATK_MS901M_EINVAL  : 错误函数参数，获取指定数据帧失败
 *              ATK_MS901M_ETIMEOUT: 接收数据帧超时，获取指定数据帧失败
 */
uint8_t AtkMs901m::GetFrameById(atk_ms901m_frame_t *frame, uint8_t id, uint8_t id_type, uint32_t timeout)
{
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
    for (uint32_t i = 0; i < timeout; i++) {
        frame = &imu_frame_;
        if (frame->id == id) {
            if (id_type == ATK_MS901M_FRAME_ID_TYPE_UPLOAD) {
                break;
            } else if (id_type == ATK_MS901M_FRAME_ID_TYPE_ACK) {
                break;
            } else {
                return ATK_MS901M_EINVAL;
            }
        }
        usleep(1000);
    }
    return ATK_MS901M_EOK;
}

void AtkMs901m::ReadRegById(uint8_t id)
{
    uint8_t buf[7];
    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_READ_REG_ID(id);
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    serial_comm_->SendBuffer(buf, 6);
}

uint8_t AtkMs901m::ReadRegById(uint8_t id, uint8_t *dat, uint32_t timeout)
{
    uint8_t buf[7];
    uint8_t ret;
    atk_ms901m_frame_t frame;
    uint8_t dat_index;

    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_READ_REG_ID(id);
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    serial_comm_->SendBuffer(buf, 6);
    ret = GetFrameById(&frame, id, ATK_MS901M_FRAME_ID_TYPE_ACK, timeout);
    if (ret != ATK_MS901M_EOK) {
        return 0;
    }

    for (dat_index = 0; dat_index < frame.len; dat_index++) {
        dat[dat_index] = frame.dat[dat_index];
    }

    return frame.len;
}

/**
 * @brief       通过帧ID写入ATK-MS901M寄存器
 * @param       id : 寄存器对应的通讯帧ID
 *              len: 待写入数据长度（1或2）
 *              dat: 待写入的数据
 * @retval      ATK_MS901M_EOK   : 寄存器写入成功
 *              ATK_MS901M_EINVAL: 函数参数len有误
 */
uint8_t AtkMs901m::WriteRegById(uint8_t id, uint8_t len, uint8_t *dat)
{
    uint8_t buf[7];

    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_WRITE_REG_ID(id);
    buf[3] = len;
    if (len == 1) {
        buf[4] = dat[0];
        buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
        serial_comm_->SendBuffer(buf, 6);
    } else if (len == 2) {
        buf[4] = dat[0];
        buf[5] = dat[1];
        buf[6] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5];
        serial_comm_->SendBuffer(buf, 7);
    } else {
        return ATK_MS901M_EINVAL;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M磁力计数据
 * @param       magnetometer_dat: 磁力计数据结构体
 *              timeout         : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M磁力计数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M磁力计数据失败
 */
uint8_t AtkMs901m::GetMagnetometer(atk_ms901m_magnetometer_data_t *magnetometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame;

    if (magnetometer_dat == NULL) {
        return ATK_MS901M_ERROR;
    }

    ret = GetFrameById(&frame, ATK_MS901M_FRAME_ID_MAG, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    magnetometer_dat->x           = (int16_t)(frame.dat[1] << 8) | frame.dat[0];
    magnetometer_dat->y           = (int16_t)(frame.dat[3] << 8) | frame.dat[2];
    magnetometer_dat->z           = (int16_t)(frame.dat[5] << 8) | frame.dat[4];
    magnetometer_dat->temperature = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 100;

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M气压计数据
 * @param       barometer_dat: 气压计数据结构体
 *              timeout      : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M气压计数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M气压计数据失败
 */
uint8_t AtkMs901m::GetBarometer(atk_ms901m_barometer_data_t *barometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame;

    if (barometer_dat == NULL) {
        return ATK_MS901M_ERROR;
    }

    ret = GetFrameById(&frame, ATK_MS901M_FRAME_ID_BARO, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    barometer_dat->pressure    = (int32_t)(frame.dat[3] << 24) | (frame.dat[2] << 16) | (frame.dat[1] << 8) | frame.dat[0];
    barometer_dat->altitude    = (int32_t)(frame.dat[7] << 24) | (frame.dat[6] << 16) | (frame.dat[5] << 8) | frame.dat[4];
    barometer_dat->temperature = (float)((int16_t)(frame.dat[9] << 8) | frame.dat[8]) / 100;

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M端口数据
 * @param       port_dat: 端口数据结构体
 *              timeout : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M端口数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M端口数据失败
 */
uint8_t AtkMs901m::GetPort(atk_ms901m_port_data_t *port_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame;

    if (port_dat == NULL) {
        return ATK_MS901M_ERROR;
    }

    ret = GetFrameById(&frame, ATK_MS901M_FRAME_ID_PORT, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    port_dat->d0 = (uint16_t)(frame.dat[1] << 8) | frame.dat[0];
    port_dat->d1 = (uint16_t)(frame.dat[3] << 8) | frame.dat[2];
    port_dat->d2 = (uint16_t)(frame.dat[5] << 8) | frame.dat[4];
    port_dat->d3 = (uint16_t)(frame.dat[7] << 8) | frame.dat[6];

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M LED灯状态
 * @param       state: LED灯状态
 *              timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M LED灯状态成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M LED灯状态失败
 */
uint8_t AtkMs901m::GetLedState(atk_ms901m_led_state_t *state, uint32_t timeout)
{
    uint8_t ret;

    ret = ReadRegById(ATK_MS901M_FRAME_ID_REG_LEDOFF, (uint8_t *)state, timeout);
    if (ret == 0) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief       设置ATK-MS901M LED灯状态
 * @param       state: LED灯状态
 *              timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 设置ATK-MS901M LED灯状态成功
 *              ATK_MS901M_ERROR: 设置ATK-MS901M LED灯状态失败
 */
uint8_t AtkMs901m::SetLedState(atk_ms901m_led_state_t state)
{
    uint8_t ret;

    ret = WriteRegById(ATK_MS901M_FRAME_ID_REG_LEDOFF, 1, (uint8_t *)&state);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M指定端口模式
 * @param       port   : 指定端口
 *              mode   : 端口的模式
 *              timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M指定端口模式成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M指定端口模式失败
 */
uint8_t AtkMs901m::GetPortMode(atk_ms901m_port_t port, atk_ms901m_port_mode_t *mode, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;

    if (port == ATK_MS901M_PORT_D0) {
        id = ATK_MS901M_FRAME_ID_REG_D0MODE;
    } else if (port == ATK_MS901M_PORT_D1) {
        id = ATK_MS901M_FRAME_ID_REG_D1MODE;
    } else if (port == ATK_MS901M_PORT_D2) {
        id = ATK_MS901M_FRAME_ID_REG_D2MODE;
    } else if (port == ATK_MS901M_PORT_D3) {
        id = ATK_MS901M_FRAME_ID_REG_D3MODE;
    } else {
        return ATK_MS901M_ERROR;
    }

    ret = ReadRegById(id, (uint8_t *)mode, timeout);
    if (ret == 0) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief       设置ATK-MS901M指定端口模式
 * @param       port   : 指定端口
 *              mode   : 端口的模式
 *              timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 设置ATK-MS901M指定端口模式成功
 *              ATK_MS901M_ERROR: 设置ATK-MS901M指定端口模式失败
 */
uint8_t AtkMs901m::SetPortMode(atk_ms901m_port_t port, atk_ms901m_port_mode_t mode)
{
    uint8_t ret;
    uint8_t id;

    if (port == ATK_MS901M_PORT_D0) {
        if (mode == ATK_MS901M_PORT_MODE_OUTPUT_PWM) {
            return ATK_MS901M_ERROR;
        }
        id = ATK_MS901M_FRAME_ID_REG_D0MODE;
    } else if (port == ATK_MS901M_PORT_D1) {
        id = ATK_MS901M_FRAME_ID_REG_D1MODE;
    } else if (port == ATK_MS901M_PORT_D2) {
        if (mode == ATK_MS901M_PORT_MODE_OUTPUT_PWM) {
            return ATK_MS901M_ERROR;
        }
        id = ATK_MS901M_FRAME_ID_REG_D2MODE;
    } else if (port == ATK_MS901M_PORT_D3) {
        id = ATK_MS901M_FRAME_ID_REG_D3MODE;
    } else {
        return ATK_MS901M_ERROR;
    }

    ret = WriteRegById(id, 1, (uint8_t *)&mode);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M指定端口PWM高电平的宽度
 * @param       port   : 指定端口
 *              pulse  : 端口PWM高电平的宽度，单位：纳秒（ns）
 *              timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M指定端口PWM高电平的宽度成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M指定端口PWM高电平的宽度失败
 */
uint8_t AtkMs901m::GetPortPwmPulse(atk_ms901m_port_t port, uint16_t *pulse, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;

    if (port == ATK_MS901M_PORT_D0) {
        return ATK_MS901M_ERROR;
    } else if (port == ATK_MS901M_PORT_D1) {
        id = ATK_MS901M_FRAME_ID_REG_D1PULSE;
    } else if (port == ATK_MS901M_PORT_D2) {
        return ATK_MS901M_ERROR;
    } else if (port == ATK_MS901M_PORT_D3) {
        id = ATK_MS901M_FRAME_ID_REG_D3PULSE;
    } else {
        return ATK_MS901M_ERROR;
    }

    ret = ReadRegById(id, (uint8_t *)pulse, timeout);
    if (ret == 0) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief       设置ATK-MS901M指定端口PWM高电平的宽度
 * @param       port   : 指定端口
 *              pulse  : 端口PWM高电平的宽度，单位：纳秒（ns）
 *              timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 设置ATK-MS901M指定端口PWM高电平的宽度成功
 *              ATK_MS901M_ERROR: 设置ATK-MS901M指定端口PWM高电平的宽度失败
 */
uint8_t AtkMs901m::SetPortPwmPulse(atk_ms901m_port_t port, uint16_t pulse)
{
    uint8_t ret;
    uint8_t id;

    if (port == ATK_MS901M_PORT_D0) {
        return ATK_MS901M_ERROR;
    } else if (port == ATK_MS901M_PORT_D1) {
        id = ATK_MS901M_FRAME_ID_REG_D1PULSE;
    } else if (port == ATK_MS901M_PORT_D2) {
        return ATK_MS901M_ERROR;
    } else if (port == ATK_MS901M_PORT_D3) {
        id = ATK_MS901M_FRAME_ID_REG_D3PULSE;
    } else {
        return ATK_MS901M_ERROR;
    }

    ret = WriteRegById(id, 2, (uint8_t *)&pulse);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M指定端口PWM周期
 * @param       port   : 指定端口
 *              period : 端口PWM周期，单位：纳秒（ns）
 *              timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M指定端口PWM周期成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M指定端口PWM周期失败
 */
uint8_t AtkMs901m::GetPortPwmPeriod(atk_ms901m_port_t port, uint16_t *period, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;

    if (port == ATK_MS901M_PORT_D0) {
        return ATK_MS901M_ERROR;
    } else if (port == ATK_MS901M_PORT_D1) {
        id = ATK_MS901M_FRAME_ID_REG_D1PERIOD;
    } else if (port == ATK_MS901M_PORT_D2) {
        return ATK_MS901M_ERROR;
    } else if (port == ATK_MS901M_PORT_D3) {
        id = ATK_MS901M_FRAME_ID_REG_D3PERIOD;
    } else {
        return ATK_MS901M_ERROR;
    }

    ret = ReadRegById(id, (uint8_t *)period, timeout);
    if (ret == 0) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

/**
 * @brief       设置ATK-MS901M指定端口PWM周期
 * @param       port   : 指定端口
 *              period : 端口PWM周期，单位：纳秒（ns）
 *              timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 设置ATK-MS901M指定端口PWM周期成功
 *              ATK_MS901M_ERROR: 设置ATK-MS901M指定端口PWM周期失败
 */
uint8_t AtkMs901m::SetPortPwmPeriod(atk_ms901m_port_t port, uint16_t period)
{
    uint8_t ret;
    uint8_t id;

    if (port == ATK_MS901M_PORT_D0) {
        return ATK_MS901M_ERROR;
    } else if (port == ATK_MS901M_PORT_D1) {
        id = ATK_MS901M_FRAME_ID_REG_D1PERIOD;
    } else if (port == ATK_MS901M_PORT_D2) {
        return ATK_MS901M_ERROR;
    } else if (port == ATK_MS901M_PORT_D3) {
        id = ATK_MS901M_FRAME_ID_REG_D3PERIOD;
    } else {
        return ATK_MS901M_ERROR;
    }

    ret = WriteRegById(id, 2, (uint8_t *)&period);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

uint8_t AtkMs901m::SetBaudRate(uint8_t rate)
{
    uint8_t ret;
    uint8_t id = ATK_MS901M_FRAME_ID_REG_BAUD;

    ret = WriteRegById(id, 1, (uint8_t *)&rate);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

uint8_t AtkMs901m::SaveToFlash()
{
    uint8_t ret;
    uint8_t id = ATK_MS901M_FRAME_ID_REG_SAVE;
    uint8_t cmd = 0x00;

    ret = WriteRegById(id, 1, (uint8_t *)&cmd);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}
