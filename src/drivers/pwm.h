/**
 * @file pwm.h
 * @author Leo Huang (846863428@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-05
 * @copyright Copyright (c) {2021} 个人版权所有
 */
#ifndef __PWM_H__
#define __PWM_H__

struct PwmPram {
    int chip = 0;
    int channel = 0;
    bool polarity = true;
    int period = 0;
    int dutycycle = 0;
};

class Pwm
{
public:
    Pwm(PwmPram parm);
    ~Pwm();
    int SetupPwm();
    int PwmPolarity(bool value);
    int PwmPeriod(uint32_t value);
    int PwmDutyCycle(uint32_t value);

private:
    int UnExportPwm();
    int PwmEnable(bool enable);
    int pwm_chip_;      // pwm控制器
    int pwm_channel_;   // pwm通道
};

#endif
