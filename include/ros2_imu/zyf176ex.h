/**
 * @file zyf176ex.h
 * @author Leo Huang (846863428@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-23
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __ZYF176EX_H__
#define __ZYF176EX_H__

#include "imu_interface.h"
#include "serial.h"
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

typedef struct
{
    uint16_t head;   /* 头部固定为0xA5A5 */
    int16_t acc_x;  /* 加速度x轴原始数据 */
    int16_t acc_y;  /* 加速度y轴原始数据 */
    int16_t acc_z;  /* 加速度z轴原始数据 */
    int16_t gyro_x; /* x轴角速度原始数据 */
    int16_t gyro_y; /* y轴角速度原始数据 */
    int16_t gyro_z; /* z轴角速度原始数据 */
    int16_t yaw;    /* 航向角yaw原始数据 */
    int16_t pitch;  /* 俯仰角pitch原始数据 */
    int16_t roll;   /* 横滚角roll原始数据 */
    uint16_t timer;  /* 时间戳原始数据 */
    uint16_t crc;    /* CRC校验和 */
} zyz_data_t;

class Zyf176ex : public ImuInterface
{
public:
    Zyf176ex(ImuConf conf);
    virtual ~Zyf176ex();

    bool Init();

    Imu GetImuData();

private:
    std::thread imu_thread_;
    std::shared_ptr<Communication> serial_comm_; // 通讯端口
    std::mutex data_lock_;
    Imu imu_data_;
    int32_t imu_coefficient_;
    std::condition_variable g_cv_; // 全局条件变量
    std::mutex g_mtx_;             // 全局互斥锁.

    struct
    {
        uint8_t rx_buffer[256];
        uint32_t size; // buf长度
    } zyz_176ex_buffer_;

    void ImuReader();

    void ReadBuffer(const uint8_t *buffer, const int length);

    zyz_data_t *SearchHearLE(uint8_t *data, uint32_t total_len, int &index);

    /**
     * @brief 欧拉角转四元数
     * @param roll
     * @param pitch
     * @param yaw
     * @param q
     */
    void Euler2Quaternion(double roll, double pitch, double yaw, Quaternion &q);

    std::string Bytes2String(uint8_t *data, uint32_t len);

    /**
     * @brief 角度清零
     */
    void ResetHanding();

    /**
     * @brief 重置陀螺仪零偏
     */
    void ResetBias();

    /**
     * @brief 重启
     */
    void SoftRest();

    /**
     * @brief 软件版本
     */
    void SoftVersion();
};

#endif
