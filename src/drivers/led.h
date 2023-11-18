/**
 * @file led.h
 * @author Leo Huang (846863428@qq.com)
 * @brief led操作
 * @version 0.1
 * @date 2023-04-03
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __LED_H__
#define __LED_H__

#include <iostream>

class Led {
public:
    Led(std::string dev);
    ~Led();

    int SetGpioValue(bool value);

private:
    std::string led_dev_;
    int led_fd_;
};

#endif
