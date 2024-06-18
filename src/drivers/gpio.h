/**
 * @file gpio.h
 * @author 黄李全 (846863428@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-01-05
 * @copyright Copyright (c) {2021} 个人版权所有
 */
#ifndef __GPIO_H__
#define __GPIO_H__

#include <iostream>
#include <unordered_map>
#include <functional>

typedef enum {
    IRQ_TYPE_EDGE_RISING = 0x00, // 上升沿触发
    IRQ_TYPE_EDGE_FALLING = 0x01, // 下降沿触发
    IRQ_TYPE_EDGE_BOTH = 0x02,  // 设置为双边沿触发
    IRQ_TYPE_LEVEL_HIGH,   //高电平触发
    IRQ_TYPE_LEVEL_LOW,    //低电平触发
    IRQ_TYPE_NONE,         //默认值，无定义中断触发类型
} EdgeType;

class Gpio
{
public:
    Gpio() {}
    virtual ~Gpio() {}

    virtual bool Init() = 0;
    virtual int OpenGpio() = 0;
    // 设置输出值
    virtual void SetValue(bool value) = 0;
    // 获取gpio值
    virtual bool GetValue() = 0;
    // 修改输入输出方向
    virtual bool SetDirection(bool io) = 0;

    // 配置极性
    virtual void SetActiveLow(bool act_low) = 0;
    // 设置中断方式
    virtual void SetEdge(EdgeType type) = 0;

    void AddEvent(std::function<void(const bool, const uint64_t)> handler) {
        read_function_ = handler;
    }

protected:
    bool is_output_;
    std::function<void(const bool, const uint64_t)> read_function_; // 外部回调
};

#endif
