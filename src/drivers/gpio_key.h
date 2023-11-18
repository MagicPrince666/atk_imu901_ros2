/**
 * @file gpio_key.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 按键
 * @version 0.1
 * @date 2023-04-05
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __GPIO_KEY_H__
#define __GPIO_KEY_H__

#include <sys/time.h>
#include <iostream>

class GpioKey
{
public:
    GpioKey(std::string input);
    ~GpioKey();

    bool init();

private:
    int key_input_fd_;
    std::string dev_input_;

    int IRKey();
};

#endif
