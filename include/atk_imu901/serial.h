/**
 * @file serial.h
 * @author 黄李全 (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-07-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <functional>
#include <iostream>
#include <condition_variable>
#include <stdint.h>
#include <termios.h>

#include "comm_factory.h"
#include "ring_buffer.h"

#define DATA_LEN 121

class Serial : public Communication
{
private:
    int uart_fd_;
    struct termios original_port_settings_;

    RingBuffer tx_ring_buffer_;                                // 发送缓存队列

    std::function<void(const uint8_t *, const uint32_t)> read_function_; // 接收回调

    /**
     * @brief 打开串口
     * @baudrate 波特率
     * @return int
     */
    int OpenSerial();

    /**
     * @brief 串口关闭
     */
    void CloseSerial();

    /**
     * @brief 收到MCU数据产生的回调
     * @return int
     */
    int ReadCallback();

    /**
     * @brief 串口可写回调
     * @return int
     */
    int WriteCallback();

    /**
     * @brief 字节转文本
     * @param data
     * @param len
     * @return std::string
     */
    std::string Bytes2String(uint8_t *data, uint32_t len);

public:
    Serial(std::string dev = "/dev/ttyS2", const int baudrate = 115200, bool debug = false);
    ~Serial();

    /**
     * @brief 发送buf
     * @param buffer
     * @param length
     * @return int
     */
    int SendBuffer(const uint8_t *const buffer, const int length);

    /**
     * @brief 回调注册
     * @param handler 
     */
    void AddCallback(std::function<void(const uint8_t *, const uint32_t)> handler);
};

// 生产串口uart部件的工厂
class SerialComm : public CommFactory
{
public:
    Communication *CreateCommTarget(std::string dev, const int baudrate, bool debug)
    {
        return new Serial(dev, baudrate, debug);
    }
};

#endif
