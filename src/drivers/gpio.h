/**
 * @file gpio.h
 * @author Leo Huang (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-01-05
 * @copyright Copyright (c) {2021} 个人版权所有
 */
#ifndef __GPIO_H__
#define __GPIO_H__

#include <iostream>

class Gpio
{
public:
    Gpio(int pin, bool io);
    ~Gpio();

    int LightCtl(int leds, bool status);
    int SetGpioValue(bool value);
    int GetGpioValue();

private:
    int UnExportGpio();
    int SetupGpio();
    int SetGpioOut();
    int SetGpioIn();
    int GpioOutPut(bool io);
    int OpenGpio();
    int CloseGpio();
    std::string ReadFileIntoString(const std::string& path);
    char setpin[64] = {0};
    int gpio_pin_;
    int gpio_fd_;
};

#endif
