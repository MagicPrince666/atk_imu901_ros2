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
#include "rclcpp/rclcpp.hpp"

#include <cstring>
#include <unistd.h>

AtkMs901m::AtkMs901m(std::string port, uint32_t rate)
    : ImuInterface(port, rate)
{
    /* ATK-MS601M UART初始化 */
    // 创建通讯部件工厂,这一步可以优化到从lunch配置文件选择初始化不同的通讯部件工厂
    std::shared_ptr<CommFactory> factory(new SerialComm());
    // 通过工厂方法创建通讯产品
    std::shared_ptr<Communication> serial(factory->CreateCommTarget(imu_port_, baud_rate_, false));
    serial_comm_ = serial;
}

AtkMs901m::~AtkMs901m() {}

bool AtkMs901m::Init()
{
    /* 获取ATK-MS901M陀螺仪满量程 */
    uint8_t ret = ReadRegById(ATK_MS901M_FRAME_ID_REG_GYROFSR, &atk_ms901m_fsr_.gyro, 100);
    if (ret == 0) {
        return false;
    }

    /* 获取ATK-MS901M加速度计满量程 */
    ret = ReadRegById(ATK_MS901M_FRAME_ID_REG_ACCFSR, &atk_ms901m_fsr_.accelerometer, 100);
    if (ret == 0) {
        return false;
    }

    imu_thread_ = std::thread([](AtkMs901m *p_this) { p_this->ImuReader(); }, this);

    return true;
}

void AtkMs901m::ImuReader()
{
    while (rclcpp::ok()) {
        int len = serial_comm_->ReadBuffer(atk_ms901m_buffer_.rx_buffer + atk_ms901m_buffer_.size, sizeof(atk_ms901m_buffer_.rx_buffer) - atk_ms901m_buffer_.size);
        if (len > 0) {
            atk_ms901m_buffer_.size += len; // 更新buff长度
            uint8_t *ros_rx_buffer_ptr  = atk_ms901m_buffer_.rx_buffer;
            int index                   = 0;
            atk_ms901m_frame_t *res_tmp = SearchHearLE(ros_rx_buffer_ptr, atk_ms901m_buffer_.size, index);
            if (res_tmp == nullptr) {
                // 已经处理完所有可识别的包
                RCLCPP_WARN(rclcpp::get_logger("AtkMs901m"), "not found buffer head size = %d", atk_ms901m_buffer_.size);
                usleep(10000);
                continue;
            } else {
                // 重置指针位置指向包头位置和更新长度
                if (index) {
                    int32_t lenght = atk_ms901m_buffer_.size - index;
                    if (lenght >= (res_tmp->len + 5)) {
                        ros_rx_buffer_ptr += index;
                        atk_ms901m_buffer_.size = lenght;
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("AtkMs901m"), "not a full buffer size = %d", atk_ms901m_buffer_.size);
                        usleep(10000);
                        continue;
                    }
                }

                if (res_tmp->id == 0x0) {
                    std::lock_guard<std::mutex> mylock_guard(data_lock_);
                    uint8_t sum = res_tmp->head_l + res_tmp->head_h + res_tmp->id + res_tmp->len;
                    for (uint32_t i = 0; i < res_tmp->len; i++) {
                        sum += res_tmp->dat[i];
                    }
                    if (sum == res_tmp->check_sum) {
                        // memcpy(frame, res_tmp, res_tmp->len + 5);
                        RCLCPP_INFO(rclcpp::get_logger("AtkMs901m"), "len = %d", res_tmp->len);
                        atk_ms901m_buffer_.size -= res_tmp->len + 5;
                        std::unique_ptr<uint8_t[]> buffer(new uint8_t[atk_ms901m_buffer_.size]);
                        // 剩余未处理数据拷贝到临时变量
                        memcpy(buffer.get(), ros_rx_buffer_ptr + res_tmp->len + 5, atk_ms901m_buffer_.size);
                        // 覆盖掉原来的buff
                        memcpy(atk_ms901m_buffer_.rx_buffer, buffer.get(), atk_ms901m_buffer_.size);
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("AtkMs901m"), "Check sum fail");
                        usleep(10000);
                        continue;
                    }
                }
            }
        }
        usleep(10000);
    }
}

void AtkMs901m::Euler2Quaternion(double roll, double pitch, double yaw, Quaternion &q)
{
    // 传入机器人的欧拉角 roll、pitch 和 yaw。
    // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中
    // https://blog.csdn.net/xiaoma_bk/article/details/79082629
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    // 计算出四元数的四个分量 q.w、q.x、q.y、q.z
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

Imu AtkMs901m::GetImuData()
{
    std::lock_guard<std::mutex> mylock_guard(data_lock_);
#if 0
    // atk_ms901m_attitude_data_t attitude_dat;           /* 姿态角数据 */
    atk_ms901m_gyro_data_t gyro_dat;                   /* 陀螺仪数据 */
    atk_ms901m_accelerometer_data_t accelerometer_dat; /* 加速度计数据 */
    atk_ms901m_quaternion_data_t quaternion_dat;

    // GetAttitude(&attitude_dat, 100);                          /* 获取姿态角数据 */
    GetGyroAccelerometer(&gyro_dat, &accelerometer_dat, 100); /* 获取陀螺仪、加速度计数据 */
    GetQuaternion(&quaternion_dat, 100);

    imu_data_.orientation.w         = quaternion_dat.q0;
    imu_data_.orientation.x         = quaternion_dat.q1;
    imu_data_.orientation.y         = quaternion_dat.q2;
    imu_data_.orientation.z         = quaternion_dat.q3;
    imu_data_.angular_velocity.x    = gyro_dat.x;
    imu_data_.angular_velocity.y    = gyro_dat.y;
    imu_data_.angular_velocity.z    = gyro_dat.z;
    imu_data_.linear_acceleration.x = accelerometer_dat.x;
    imu_data_.linear_acceleration.y = accelerometer_dat.y;
    imu_data_.linear_acceleration.z = accelerometer_dat.z;
#endif
    return imu_data_;
}

atk_ms901m_frame_t *AtkMs901m::SearchHearLE(uint8_t *data, uint32_t total_len, int &index)
{
    atk_ms901m_frame_t *res_tmp = nullptr;
    for (uint32_t i = 0; i < total_len; i++) {
        // 剩余长度大于一个包长度，说明还有协议数据包可以处理
        if ((total_len - i) >= sizeof(atk_ms901m_frame_t)) {
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
    if (timeout == 0) {
        return ATK_MS901M_ETIMEOUT;
    }
    int len = serial_comm_->ReadBuffer(atk_ms901m_buffer_.rx_buffer + atk_ms901m_buffer_.size, sizeof(atk_ms901m_buffer_.rx_buffer) - atk_ms901m_buffer_.size);
    if (len < 1) {
        // 没有有效数据直接返回
        return ATK_MS901M_ETIMEOUT;
    }
    atk_ms901m_buffer_.size += len; // 更新buff长度
    if (atk_ms901m_buffer_.size < sizeof(atk_ms901m_frame_t)) {
        return ATK_MS901M_ETIMEOUT;
    }
    uint8_t *ros_rx_buffer_ptr = atk_ms901m_buffer_.rx_buffer;

    while (rclcpp::ok()) {
        int index                   = 0;
        atk_ms901m_frame_t *res_tmp = SearchHearLE(ros_rx_buffer_ptr, atk_ms901m_buffer_.size, index);
        if (res_tmp == nullptr) {
            // 已经处理完所有可识别的包
            return ATK_MS901M_EOK;
        } else {
            // 重置指针位置指向包头位置和更新长度
            if (index) {
                ros_rx_buffer_ptr += index;
                atk_ms901m_buffer_.size -= index;
            }
        }

        if (res_tmp->id == id) {
            std::lock_guard<std::mutex> mylock_guard(data_lock_);
            uint8_t sum = res_tmp->head_l + res_tmp->head_h + res_tmp->id + res_tmp->len;
            for (uint32_t i = 0; i < res_tmp->len; i++) {
                sum += res_tmp->dat[i];
            }
            if (sum == res_tmp->check_sum) {
                memcpy(frame, res_tmp, res_tmp->len + 5);
                atk_ms901m_buffer_.size -= res_tmp->len + 5;
                std::unique_ptr<uint8_t[]> buffer(new uint8_t[atk_ms901m_buffer_.size]);
                // 剩余未处理数据拷贝到临时变量
                memcpy(buffer.get(), ros_rx_buffer_ptr + res_tmp->len + 5, atk_ms901m_buffer_.size);
                // 覆盖掉原来的buff
                memcpy(atk_ms901m_buffer_.rx_buffer, buffer.get(), atk_ms901m_buffer_.size);

            } else {
                continue;
            }

            if (id_type == ATK_MS901M_FRAME_ID_TYPE_UPLOAD) {
                return ATK_MS901M_EOK;
            } else if (id_type == ATK_MS901M_FRAME_ID_TYPE_ACK) {
                return ATK_MS901M_EOK;
            } else {
                return ATK_MS901M_EINVAL;
            }
        } else {
            continue;
        }
    }
    return ATK_MS901M_EOK;
}

/**
 * @brief       通过帧ID读取ATK-MS901M寄存器
 * @param       id     : 寄存器对应的通讯帧ID
 *              dat    : 读取到的数据
 *              timeout: 等待数据的最大超时时间，单位：毫秒（ms）
 * @retval      0: 读取失败
 *              其他值: 读取到数据的长度
 */
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
 * @brief       获取ATK-MS901M姿态角数据
 * @param       attitude_dat: 姿态角数据结构体
 *              timeout     : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M姿态角数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M姿态角数据失败
 */
uint8_t AtkMs901m::GetAttitude(atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame;

    if (attitude_dat == NULL) {
        return ATK_MS901M_ERROR;
    }

    ret = GetFrameById(&frame, ATK_MS901M_FRAME_ID_ATTITUDE, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    attitude_dat->roll  = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768.0 * M_PI;
    attitude_dat->pitch = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768.0 * M_PI;
    attitude_dat->yaw   = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768.0 * M_PI;

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M四元数数据
 * @param       quaternion_dat: 四元数数据结构体
 *              timeout       : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M四元数数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M四元数数据失败
 */
uint8_t AtkMs901m::GetQuaternion(atk_ms901m_quaternion_data_t *quaternion_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame;

    if (quaternion_dat == NULL) {
        return ATK_MS901M_ERROR;
    }

    ret = GetFrameById(&frame, ATK_MS901M_FRAME_ID_QUAT, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    quaternion_dat->q0 = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768.0;
    quaternion_dat->q1 = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768.0;
    quaternion_dat->q2 = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768.0;
    quaternion_dat->q3 = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 32768.0;

    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M陀螺仪、加速度计数据
 * @param       gyro_dat         : 陀螺仪数据结构体
 *              accelerometer_dat: 加速度计数据结构体
 *              timeout          : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M陀螺仪、加速度计数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M陀螺仪、加速度计数据失败
 */
uint8_t AtkMs901m::GetGyroAccelerometer(atk_ms901m_gyro_data_t *gyro_dat, atk_ms901m_accelerometer_data_t *accelerometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame;

    if ((gyro_dat == NULL) && (accelerometer_dat == NULL)) {
        return ATK_MS901M_ERROR;
    }

    ret = GetFrameById(&frame, ATK_MS901M_FRAME_ID_GYRO_ACCE, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    if (gyro_dat != NULL) {
        gyro_dat->raw.x = (int16_t)(frame.dat[7] << 8) | frame.dat[6];
        gyro_dat->raw.y = (int16_t)(frame.dat[9] << 8) | frame.dat[8];
        gyro_dat->raw.z = (int16_t)(frame.dat[11] << 8) | frame.dat[10];

        gyro_dat->x = (float)gyro_dat->raw.x / 32768.0 * 2000 / 180.0 * M_PI;
        gyro_dat->y = (float)gyro_dat->raw.y / 32768.0 * 2000 / 180.0 * M_PI;
        gyro_dat->z = (float)gyro_dat->raw.z / 32768.0 * 2000 / 180.0 * M_PI;
    }

    if (accelerometer_dat != NULL) {
        accelerometer_dat->raw.x = (int16_t)(frame.dat[1] << 8) | frame.dat[0];
        accelerometer_dat->raw.y = (int16_t)(frame.dat[3] << 8) | frame.dat[2];
        accelerometer_dat->raw.z = (int16_t)(frame.dat[5] << 8) | frame.dat[4];

        accelerometer_dat->x = (float)accelerometer_dat->raw.x / 32768.0 * 16 * 9.8;
        accelerometer_dat->y = (float)accelerometer_dat->raw.y / 32768.0 * 16 * 9.8;
        accelerometer_dat->z = (float)accelerometer_dat->raw.z / 32768.0 * 16 * 9.8;
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
uint8_t AtkMs901m::SetLedState(atk_ms901m_led_state_t state, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_led_state_t state_recv;

    ret = WriteRegById(ATK_MS901M_FRAME_ID_REG_LEDOFF, 1, (uint8_t *)&state);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    ret = GetLedState(&state_recv, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    if (state_recv != state) {
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
uint8_t AtkMs901m::SetPortMode(atk_ms901m_port_t port, atk_ms901m_port_mode_t mode, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    atk_ms901m_port_mode_t mode_recv;

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

    ret = GetPortMode(port, &mode_recv, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    } else {
        if (mode_recv != mode) {
            return ATK_MS901M_ERROR;
        }
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
uint8_t AtkMs901m::SetPortPwmPulse(atk_ms901m_port_t port, uint16_t pulse, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    uint16_t pulse_recv;

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

    ret = GetPortPwmPulse(port, &pulse_recv, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    if (pulse_recv != pulse) {
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
uint8_t AtkMs901m::SetPortPwmPeriod(atk_ms901m_port_t port, uint16_t period, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    uint16_t period_recv;

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

    ret = GetPortPwmPeriod(port, &period_recv, timeout);
    if (ret != ATK_MS901M_EOK) {
        return ATK_MS901M_ERROR;
    }

    if (period_recv != period) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}
