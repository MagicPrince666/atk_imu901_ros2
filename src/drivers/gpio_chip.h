/**
 * @file gpiochip.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 通用IO控制
 * @version 0.1
 * @date 2024-05-08
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __GPIO_CHIP_H__
#define __GPIO_CHIP_H__

#include <iostream>
#include "gpio.h"
#if defined(__linux__)
#include <linux/gpio.h>
#endif

// #define GPIOHANDLE_REQUEST_BIAS_PULL_UP	(1UL << 5)
// #define GPIOHANDLE_REQUEST_BIAS_PULL_DOWN	(1UL << 6)
// #define GPIOHANDLE_REQUEST_BIAS_DISABLE	(1UL << 7)

class GpioChip : public Gpio
{
public:
    GpioChip(std::string gpio_chip, int gpio_line);
    ~GpioChip();

    bool Init();
    int OpenGpio();
    void SetValue(bool value);
    bool GetValue();
    bool SetDirection(bool io);

    // 配置极性
    void SetActiveLow(bool act_low);
    // 设置中断方式
    void SetEdge(EdgeType type);

private:
    std::string gpio_chip_;
    int gpio_line_;
    int gpiochip_fd_;
    bool gpio_value_;
    // std::shared_ptr<Poll> poll_ptr_;
#if defined(__linux__)
    struct gpiohandle_request handle_req_;
    struct gpioevent_request event_req_;
    std::unordered_map<EdgeType, uint64_t> interrupt_type_map_ = {
        {IRQ_TYPE_EDGE_RISING, GPIOEVENT_REQUEST_RISING_EDGE},
        {IRQ_TYPE_EDGE_FALLING, GPIOEVENT_REQUEST_FALLING_EDGE},
        {IRQ_TYPE_EDGE_BOTH, GPIOEVENT_REQUEST_BOTH_EDGES},
    };
#endif
    void GetChipInfo(void);
    void GetLineInfo(void);
    bool ReadValue();
    void RequestEvent(void);
    void RecvEvent(void);
};

#endif
