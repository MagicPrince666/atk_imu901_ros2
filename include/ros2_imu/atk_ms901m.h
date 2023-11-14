/**
 ****************************************************************************************************
 * @file        atk_ms901m.h
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

#ifndef __ATM_MS901M_H__
#define __ATM_MS901M_H__

#include "imu_interface.h"
#include "serial.h"
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

/* ATK-MS901M UART通讯帧数据最大长度 */
#define ATK_MS901M_FRAME_DAT_MAX_SIZE 28

/* ATK-MS901M UART通讯帧头 */
#define ATK_MS901M_FRAME_HEAD 0x5555
#define ATK_MS901M_FRAME_HEAD_L 0x55
#define ATK_MS901M_FRAME_HEAD_UPLOAD_H 0x55 /* 高位主动上传帧头 */
#define ATK_MS901M_FRAME_HEAD_ACK_H 0xAF    /* 高位应答帧头 */

/* ATK-MS901M读写寄存器ID */
#define ATK_MS901M_READ_REG_ID(id) (id | 0x80)
#define ATK_MS901M_WRITE_REG_ID(id) (id)

/* ATK-MS901M主动上传帧ID */
#define ATK_MS901M_FRAME_ID_ATTITUDE 0x01  /* 姿态角 */
#define ATK_MS901M_FRAME_ID_QUAT 0x02      /* 四元数 */
#define ATK_MS901M_FRAME_ID_GYRO_ACCE 0x03 /* 陀螺仪、加速度计 */
#define ATK_MS901M_FRAME_ID_MAG 0x04       /* 磁力计 */
#define ATK_MS901M_FRAME_ID_BARO 0x05      /* 气压计 */
#define ATK_MS901M_FRAME_ID_PORT 0x06      /* 端口 */

/* ATK-MS901M应答帧ID */
#define ATK_MS901M_FRAME_ID_REG_SAVE 0x00       /* （  W）保存当前配置到Flash */
#define ATK_MS901M_FRAME_ID_REG_SENCAL 0x01     /* （  W）设置传感器校准 */
#define ATK_MS901M_FRAME_ID_REG_SENSTA 0x02     /* （R  ）读取传感器校准状态 */
#define ATK_MS901M_FRAME_ID_REG_GYROFSR 0x03    /* （R/W）设置陀螺仪量程 */
#define ATK_MS901M_FRAME_ID_REG_ACCFSR 0x04     /* （R/W）设置加速度计量程 */
#define ATK_MS901M_FRAME_ID_REG_GYROBW 0x05     /* （R/W）设置陀螺仪带宽 */
#define ATK_MS901M_FRAME_ID_REG_ACCBW 0x06      /* （R/W）设置加速度计带宽 */
#define ATK_MS901M_FRAME_ID_REG_BAUD 0x07       /* （R/W）设置UART通讯波特率 */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET 0x08  /* （R/W）设置回传内容 */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET2 0x09 /* （R/W）设置回传内容2（保留） */
#define ATK_MS901M_FRAME_ID_REG_RETURNRATE 0x0A /* （R/W）设置回传速率 */
#define ATK_MS901M_FRAME_ID_REG_ALG 0x0B        /* （R/W）设置算法 */
#define ATK_MS901M_FRAME_ID_REG_ASM 0x0C        /* （R/W）设置安装方向 */
#define ATK_MS901M_FRAME_ID_REG_GAUCAL 0x0D     /* （R/W）设置陀螺仪自校准开关 */
#define ATK_MS901M_FRAME_ID_REG_BAUCAL 0x0E     /* （R/W）设置气压计自校准开关 */
#define ATK_MS901M_FRAME_ID_REG_LEDOFF 0x0F     /* （R/W）设置LED开关 */
#define ATK_MS901M_FRAME_ID_REG_D0MODE 0x10     /* （R/W）设置端口D0模式 */
#define ATK_MS901M_FRAME_ID_REG_D1MODE 0x11     /* （R/W）设置端口D1模式 */
#define ATK_MS901M_FRAME_ID_REG_D2MODE 0x12     /* （R/W）设置端口D2模式 */
#define ATK_MS901M_FRAME_ID_REG_D3MODE 0x13     /* （R/W）设置端口D3模式 */
#define ATK_MS901M_FRAME_ID_REG_D1PULSE 0x16    /* （R/W）设置端口D1 PWM高电平脉宽 */
#define ATK_MS901M_FRAME_ID_REG_D3PULSE 0x1A    /* （R/W）设置端口D3 PWM高电平脉宽 */
#define ATK_MS901M_FRAME_ID_REG_D1PERIOD 0x1F   /* （R/W）设置端口D1 PWM周期 */
#define ATK_MS901M_FRAME_ID_REG_D3PERIOD 0x23   /* （R/W）设置端口D3 PWM周期 */
#define ATK_MS901M_FRAME_ID_REG_RESET 0x7F      /* （  W）恢复默认设置 */

/* ATK-MS901M帧类型 */
#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD 0 /* ATK-MS901M主动上传帧ID */
#define ATK_MS901M_FRAME_ID_TYPE_ACK 1    /* ATK-MS901M应答帧ID */

/* 姿态角数据结构体 */
typedef struct
{
    float roll;  /* 横滚角，单位：rad */
    float pitch; /* 俯仰角，单位：rad */
    float yaw;   /* 航向角，单位：rad */
} atk_ms901m_attitude_data_t;

/* 四元数数据结构体 */
typedef struct
{
    float q0; /* Q0 */
    float q1; /* Q1 */
    float q2; /* Q2 */
    float q3; /* Q3 */
} atk_ms901m_quaternion_data_t;

/* 陀螺仪数据结构体 */
typedef struct
{
    struct
    {
        int16_t x; /* X轴原始数据 */
        int16_t y; /* Y轴原始数据 */
        int16_t z; /* Z轴原始数据 */
    } raw;
    float x; /* X轴旋转速率，单位：rad/s */
    float y; /* Y轴旋转速率，单位：rad/s */
    float z; /* Z轴旋转速率，单位：rad/s */
} atk_ms901m_gyro_data_t;

/* 加速度计数据结构体 */
typedef struct
{
    struct
    {
        int16_t x; /* X轴原始数据 */
        int16_t y; /* Y轴原始数据 */
        int16_t z; /* Z轴原始数据 */
    } raw;
    float x; /* X轴加速度，单位：G */
    float y; /* Y轴加速度，单位：G */
    float z; /* Z轴加速度，单位：G */
} atk_ms901m_accelerometer_data_t;

/* 磁力计数据结构体 */
typedef struct
{
    int16_t x;         /* X轴磁场强度 */
    int16_t y;         /* Y轴磁场强度 */
    int16_t z;         /* Z轴磁场强度 */
    float temperature; /* 温度，单位：℃ */
} atk_ms901m_magnetometer_data_t;

/* 气压计数据结构体 */
typedef struct
{
    int32_t pressure;  /* 气压，单位：Pa */
    int32_t altitude;  /* 海拔，单位：cm */
    float temperature; /* 温度，单位：℃ */
} atk_ms901m_barometer_data_t;

/* 端口数据结构体 */
typedef struct
{
    uint16_t d0; /* 端口D0数据 */
    uint16_t d1; /* 端口D1数据 */
    uint16_t d2; /* 端口D2数据 */
    uint16_t d3; /* 端口D3数据 */
} atk_ms901m_port_data_t;

typedef struct
{
    uint8_t head_l = 0; /* 低位帧头 */
    uint8_t head_h = 0; /* 高位帧头 */
    // uint16_t head                              = 0;
    uint8_t id                                 = 0;   /* 帧ID */
    uint8_t len                                = 0;   /* 数据长度 */
    uint8_t dat[ATK_MS901M_FRAME_DAT_MAX_SIZE] = {0}; /* 数据 */
    uint8_t check_sum                          = 0;   /* 校验和 */
} atk_ms901m_frame_t;                                 /* ATK-MS901M UART通讯帧结构体 */

class AtkMs901m : public ImuInterface
{
public:
    AtkMs901m(std::string type, std::string port, uint32_t rate);
    virtual ~AtkMs901m();

    bool Init(); /* ATK-MS901M初始化 */
    Imu GetImuData();

    uint8_t GetMagnetometer(atk_ms901m_magnetometer_data_t *magnetometer_dat, uint32_t timeout);                                          /* 获取ATK-MS901M磁力计数据 */
    uint8_t GetBarometer(atk_ms901m_barometer_data_t *barometer_dat, uint32_t timeout);                                                   /* 获取ATK-MS901M气压计数据 */

private:
    /* 错误代码 */
    const uint8_t ATK_MS901M_EOK      = 0; /* 没有错误 */
    const uint8_t ATK_MS901M_ERROR    = 1; /* 错误 */
    const uint8_t ATK_MS901M_EINVAL   = 2; /* 错误函数参数 */
    const uint8_t ATK_MS901M_ETIMEOUT = 3; /* 超时错误 */

    struct
    {
        uint8_t gyro;          /* 陀螺仪满量程 */
        uint8_t accelerometer; /* 加速度计满量程 */
    } atk_ms901m_fsr_;         /* ATK-MS901M满量程数据 */

    /* ATK-MS901M LED状态枚举 */
    typedef enum {
        ATK_MS901M_LED_STATE_ON  = 0x00, /* LED灯关闭 */
        ATK_MS901M_LED_STATE_OFF = 0x01, /* LED灯打开 */
    } atk_ms901m_led_state_t;

    /* ATK-MS901M端口枚举 */
    typedef enum {
        ATK_MS901M_PORT_D0 = 0x00, /* 端口D0 */
        ATK_MS901M_PORT_D1 = 0x01, /* 端口D1 */
        ATK_MS901M_PORT_D2 = 0x02, /* 端口D2 */
        ATK_MS901M_PORT_D3 = 0x03, /* 端口D3 */
    } atk_ms901m_port_t;

    /* ATK-MS901M端口模式枚举 */
    typedef enum {
        ATK_MS901M_PORT_MODE_ANALOG_INPUT = 0x00, /* 模拟输入 */
        ATK_MS901M_PORT_MODE_INPUT        = 0x01, /* 数字输入 */
        ATK_MS901M_PORT_MODE_OUTPUT_HIGH  = 0x02, /* 输出数字高电平 */
        ATK_MS901M_PORT_MODE_OUTPUT_LOW   = 0x03, /* 输出数字低电平 */
        ATK_MS901M_PORT_MODE_OUTPUT_PWM   = 0x04, /* 输出PWM */
    } atk_ms901m_port_mode_t;

    typedef enum {
        MS901_WAIT_FOR_HEAD_L = 0x00, /* 等待低位帧头 */
        MS901_WAIT_FOR_HEAD_H = 0x01, /* 等待高位帧头 */
        MS901_WAIT_FOR_ID     = 0x02, /* 等待帧ID */
        MS901_WAIT_FOR_LEN    = 0x04, /* 等待数据长度 */
        MS901_WAIT_FOR_DAT    = 0x08, /* 等待数据 */
        MS901_WAIT_FOR_SUM    = 0x16, /* 等待校验和 */
    } atk_ms901m_handle_state_t;      /* 帧处理状态机状态枚举 */

    /* 陀螺仪、加速度计满量程表 */
    const uint16_t atk_ms901m_gyro_fsr_table_[4]         = {250, 500, 1000, 2000};
    const uint8_t atk_ms901m_accelerometer_fsr_table_[4] = {2, 4, 8, 16};

    std::shared_ptr<Communication> serial_comm_; // 通讯端口

    struct
    {
        uint8_t rx_buffer[255];
        uint8_t size; // buf长度
    } atk_ms901m_buffer_;

    std::mutex data_lock_;
    std::thread imu_thread_;
    Imu imu_data_;

    std::condition_variable g_cv_; // 全局条件变量
    std::mutex g_mtx_;             // 全局互斥锁.

private:
    /* 操作函数 */
    uint8_t GetFrameById(atk_ms901m_frame_t *frame, uint8_t id, uint8_t id_type, uint32_t timeout);
    uint8_t ReadRegById(uint8_t id, uint8_t *dat, uint32_t timeout);                             /* 通过帧ID读取ATK-MS901M寄存器 */
    void ReadRegById(uint8_t id);                                                                /* 通过帧ID读取ATK-MS901M寄存器,异步方式 */
    uint8_t WriteRegById(uint8_t id, uint8_t len, uint8_t *dat);                                 /* 通过帧ID写入ATK-MS901M寄存器 */
    uint8_t GetPort(atk_ms901m_port_data_t *port_dat, uint32_t timeout);                         /* 获取ATK-MS901M端口数据 */
    uint8_t GetLedState(atk_ms901m_led_state_t *state, uint32_t timeout);                        /* 获取ATK-MS901M LED灯状态 */
    uint8_t SetLedState(atk_ms901m_led_state_t state, uint32_t timeout);                         /* 设置ATK-MS901M LED灯状态 */
    uint8_t GetPortMode(atk_ms901m_port_t port, atk_ms901m_port_mode_t *mode, uint32_t timeout); /* 获取ATK-MS901M指定端口模式 */
    uint8_t SetPortMode(atk_ms901m_port_t port, atk_ms901m_port_mode_t mode, uint32_t timeout);  /* 设置ATK-MS901M指定端口模式 */
    uint8_t GetPortPwmPulse(atk_ms901m_port_t port, uint16_t *pulse, uint32_t timeout);          /* 获取ATK-MS901M指定端口PWM高电平的宽度 */
    uint8_t SetPortPwmPulse(atk_ms901m_port_t port, uint16_t pulse, uint32_t timeout);           /* 设置ATK-MS901M指定端口PWM高电平的宽度 */
    uint8_t GetPortPwmPeriod(atk_ms901m_port_t port, uint16_t *period, uint32_t timeout);        /* 获取ATK-MS901M指定端口PWM周期 */
    uint8_t SetPortPwmPeriod(atk_ms901m_port_t port, uint16_t period, uint32_t timeout);         /* 设置ATK-MS901M指定端口PWM周期 */
    /**
     * @brief 小端系统协议头查找算法
     * @param data 包起始指针
     * @param len 包长度
     * @param index 地址偏移
     * @return int32_t 包头所在偏移
     */
    atk_ms901m_frame_t *SearchHearLE(uint8_t *data, uint32_t len, uint32_t &index);

    void ImuReader();

    void ReadBuffer(const uint8_t *buffer, const int length);

    std::string Bytes2String(uint8_t *data, uint32_t len);
};

#endif
