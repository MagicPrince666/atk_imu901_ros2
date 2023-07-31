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

#include <cstring>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

#include "atk_imu901/atk_ms901m.h"
#include "atk_imu901/serial.h"

AtkMs901m::AtkMs901m() : atk_ms901m_frame_(128) {}

AtkMs901m::~AtkMs901m() {}

uint8_t AtkMs901m::Init(std::string uart, uint32_t baudrate)
{
    uint8_t ret;

    /* ATK-MS601M UART初始化 */
    // 创建通讯部件工厂,这一步可以优化到从lunch配置文件选择初始化不同的通讯部件工厂
    std::unique_ptr<CommFactory> factory(new SerialComm());
    // 通过工厂方法创建通讯产品
    std::shared_ptr<Communication> serial(factory->CreateCommTarget(uart, baudrate, false));
    serial_comm_ = serial;

    // SetBaudRate(UART_BAUD_RATE_921600);
    // SetFrequency(FREQUENCY_100);
    // exit(0);

    memset((uint8_t *)&atk_ms901m_buffer_, 0, sizeof(atk_ms901m_buffer_));

    serial_comm_->AddCallback(std::bind(&AtkMs901m::ReadBuff, this, std::placeholders::_1, std::placeholders::_2));

    /* 获取ATK-MS901M陀螺仪满量程 */
    ret = ReadRegById(ATK_MS901M_FRAME_ID_REG_GYROFSR, &atk_ms901m_fsr_.gyro, 100);
    spdlog::info("gyro = {}", atk_ms901m_fsr_.gyro);
    if (ret == 0) {
        return ATK_MS901M_ERROR;
    }

    /* 获取ATK-MS901M加速度计满量程 */
    ret = ReadRegById(ATK_MS901M_FRAME_ID_REG_ACCFSR, &atk_ms901m_fsr_.accelerometer, 100);
    spdlog::info("accelerometer = {}", atk_ms901m_fsr_.accelerometer);
    if (ret == 0) {
        return ATK_MS901M_ERROR;
    }

    return ATK_MS901M_EOK;
}

std::string AtkMs901m::Bytes2String(const uint8_t *data, uint32_t len)
{
    char temp[512];
    std::string str("");
    for (size_t i = 0; i < len; i++) {
        sprintf(temp, "%02x ", data[i]);
        str.append(temp);
    }
    return str;
}

void AtkMs901m::ReadBuff(const uint8_t *data, const uint32_t len)
{
    // spdlog::info("Serial ={}", Bytes2String(data, len));
    // rx_ring_buffer_.RingBufferIn(data, len);
    std::lock_guard<std::mutex> lck(imu_mtx_);
    if (len > (sizeof(atk_ms901m_buffer_.rx_buffer) - atk_ms901m_buffer_.size)) {
        spdlog::warn("Buff is full len = {} buff size = {}", len, atk_ms901m_buffer_.size);
        atk_ms901m_buffer_.size = 0;
    }

    // spdlog::info("start handle");
    memcpy(atk_ms901m_buffer_.rx_buffer + atk_ms901m_buffer_.size, data, len);
    atk_ms901m_buffer_.size += len;

    uint32_t lenght = atk_ms901m_buffer_.size;
    for (uint32_t i = 0; i < lenght; i++) {
        uint8_t *imu_buffer_ptr = atk_ms901m_buffer_.rx_buffer + i;
        uint32_t buf_size       = lenght - i;
        // 长度不足，跳过
        if (atk_ms901m_buffer_.size < 6) {
            // spdlog::info("Handle done size = {}", atk_ms901m_buffer_.size);
            uint8_t buffer[sizeof(atk_ms901m_buffer_.rx_buffer)];
            // 剩余未处理数据拷贝到临时变量
            memcpy(buffer, imu_buffer_ptr, buf_size);
            // 覆盖掉原来的buff
            memcpy(atk_ms901m_buffer_.rx_buffer, buffer, buf_size);
            // 更新长度
            atk_ms901m_buffer_.size = buf_size;
            break;
        }
        if (imu_buffer_ptr[0] == 0x55) {
            // 长度明显不符，跳过
            if (imu_buffer_ptr[3] > ATK_MS901M_FRAME_DAT_MAX_SIZE) {
                continue;
            }
            uint32_t imu_buf_size = imu_buffer_ptr[3] + 4;
            // 判断剩余的buf长度是否足够
            if (imu_buf_size > atk_ms901m_buffer_.size) {
                // spdlog::info("imu size = {} buff size = {}", imu_buf_size, atk_ms901m_buffer_.size);
                uint8_t buffer[sizeof(atk_ms901m_buffer_.rx_buffer)];
                // 剩余未处理数据拷贝到临时变量
                memcpy(buffer, imu_buffer_ptr, buf_size);
                // 覆盖掉原来的buff
                memcpy(atk_ms901m_buffer_.rx_buffer, buffer, buf_size);
                // 更新长度
                atk_ms901m_buffer_.size = buf_size;
                break;
            }
            uint8_t sum = 0;
            uint32_t j  = 0;
            for (; j < imu_buf_size; j++) {
                sum += imu_buffer_ptr[j];
            }
            // 确认是一整数据，放入队列
            if (sum == imu_buffer_ptr[j]) {
                atk_ms901m_frame_t frame = *(atk_ms901m_frame_t *)(imu_buffer_ptr);
                frame.check_sum          = imu_buffer_ptr[j];
                // spdlog::info("frame = {}", Bytes2String(imu_buffer_ptr, frame.len + 5));
                atk_ms901m_frame_.push(frame);
                atk_ms901m_buffer_.size -= frame.len + 5;
                i += frame.len + 4; // 避免重复扫描
                continue;
            }
        }
    }
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
    uint16_t timeout_index                 = 0;

    while (rclcpp::ok()) {
        if (timeout == 0) {
            return ATK_MS901M_ETIMEOUT;
        }

        if (atk_ms901m_frame_.size() == 0) {
            usleep(1000);
            timeout_index++;
            if (timeout_index == 1000) {
                timeout_index = 0;
                timeout--;
            }
            continue;
        }

        atk_ms901m_frame_t recv_frame = atk_ms901m_frame_.front();
        atk_ms901m_frame_.pop();

        if(id_type != ATK_MS901M_FRAME_ID_TYPE_UPLOAD && id_type != ATK_MS901M_FRAME_ID_TYPE_ACK) {
            spdlog::info("id_type = {} not found", id_type);
            return ATK_MS901M_EINVAL;
        }

        if(id == recv_frame.id) {
            memcpy(frame, &recv_frame, sizeof(recv_frame));
            break;
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

    quaternion_dat->w = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768.0;
    quaternion_dat->x = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768.0;
    quaternion_dat->y = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768.0;
    quaternion_dat->z = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 32768.0;

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

        gyro_dat->x = (float)gyro_dat->raw.x / 32768.0 * atk_ms901m_gyro_fsr_table_[atk_ms901m_fsr_.gyro] / 180.0 * M_PI;
        gyro_dat->y = (float)gyro_dat->raw.y / 32768.0 * atk_ms901m_gyro_fsr_table_[atk_ms901m_fsr_.gyro] / 180.0 * M_PI;
        gyro_dat->z = (float)gyro_dat->raw.z / 32768.0 * atk_ms901m_gyro_fsr_table_[atk_ms901m_fsr_.gyro] / 180.0 * M_PI;
    }

    if (accelerometer_dat != NULL) {
        accelerometer_dat->raw.x = (int16_t)(frame.dat[1] << 8) | frame.dat[0];
        accelerometer_dat->raw.y = (int16_t)(frame.dat[3] << 8) | frame.dat[2];
        accelerometer_dat->raw.z = (int16_t)(frame.dat[5] << 8) | frame.dat[4];

        accelerometer_dat->x = (float)accelerometer_dat->raw.x / 32768.0 * atk_ms901m_accelerometer_fsr_table_[atk_ms901m_fsr_.accelerometer] * 9.8;
        accelerometer_dat->y = (float)accelerometer_dat->raw.y / 32768.0 * atk_ms901m_accelerometer_fsr_table_[atk_ms901m_fsr_.accelerometer] * 9.8;
        accelerometer_dat->z = (float)accelerometer_dat->raw.z / 32768.0 * atk_ms901m_accelerometer_fsr_table_[atk_ms901m_fsr_.accelerometer] * 9.8;
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

uint8_t AtkMs901m::SetBaudRate(UartBaudRate rate)
{
    uint8_t buf[6];

    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_FRAME_ID_REG_BAUD;
    buf[3] = 0x01;
    buf[4] = rate;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    serial_comm_->SendBuffer(buf, 6);

    return ATK_MS901M_EOK;
}

uint8_t AtkMs901m::SetFrequency(Frequency freq)
{
    uint8_t buf[6];

    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_FRAME_ID_REG_RETURNRATE;
    buf[3] = 0x01;
    buf[4] = freq;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    serial_comm_->SendBuffer(buf, 6);

    return ATK_MS901M_EOK;
}