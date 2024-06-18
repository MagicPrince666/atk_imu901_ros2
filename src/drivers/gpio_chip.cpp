#include "gpio_chip.h"
#include "ros2_imu/xepoll.h"
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>
// #include <filesystem>
#include <iomanip>
#include <iostream>

GpioChip::GpioChip(std::string gpio_chip, int gpio_line) : gpio_chip_(gpio_chip), gpio_line_(gpio_line)
{
#if defined(__linux__)
    gpiochip_fd_ = -1;
    handle_req_.fd   = -1;
    event_req_.fd    = -1;
#endif
    is_output_   = false;
    // poll_ptr_    = std::make_shared<Poll>();
    // poll_ptr_->Init();
    std::cout << "Setup " << gpio_chip_ << " line: " << gpio_line_ << std::endl;
}

GpioChip::~GpioChip()
{
#if defined(__linux__)
    if (event_req_.fd > 0) {
        MY_EPOLL.EpollDel(event_req_.fd);
        // poll_ptr_->PollDel(event_req_.fd);
    }
    if (handle_req_.fd > 0) {
        close(handle_req_.fd);
    }
    if (gpiochip_fd_ > 0) {
        close(gpiochip_fd_);
    }
#endif
    // poll_ptr_->PollQuit();
    std::cout << "Close " << gpio_chip_ << " line: " << gpio_line_ << std::endl;
}

bool GpioChip::Init()
{
#if defined(__linux__)
    // 打开 GPIO 控制器设备文件
    gpiochip_fd_ = open(gpio_chip_.c_str(), O_RDWR);
    if (gpiochip_fd_ < 0) {
        std::cerr << "Failed to open GPIO chip." << std::endl;
        return false;
    }
#endif
    return true;
}

int GpioChip::OpenGpio()
{
#if 0
    if (!is_output_ && event_req_.fd > 0) {
        // SetEdge(IRQ_TYPE_EDGE_BOTH);
        RequestEvent();
        MY_EPOLL.EpollAddRead(event_req_.fd, std::bind(&GpioChip::RecvEvent, this));
        // poll_ptr_->PollAddRead(event_req_.fd, std::bind(&GpioChip::RecvEvent, this));
    }
#endif
    return 0;
}

void GpioChip::GetChipInfo(void)
{
#if defined(__linux__)
    struct gpiochip_info info;
    if (gpiochip_fd_ < 0) {
        return;
    }
    memset(&info, 0, sizeof(info));
    int rv = ioctl(gpiochip_fd_, GPIO_GET_CHIPINFO_IOCTL, info);
    if (rv < 0) {
        perror("GPIO_GET_CHIPINFO_IOCTL");
    } else {
        std::cout << "GPIO chip name " << info.name << std::endl;
    }
#endif
}

void GpioChip::GetLineInfo(void)
{
#if defined(__linux__)
    struct gpioline_info info;
    if (gpiochip_fd_ < 0) {
        return;
    }
    memset(&info, 0, sizeof(info));
    info.line_offset = gpio_line_;
    int rv           = ioctl(gpiochip_fd_, GPIO_GET_LINEINFO_IOCTL, &info);
    if (rv < 0) {
        perror("GPIO_GET_LINEINFO_IOCTL");
    } else {
        std::cout << "GPIO line flag " << info.flags << std::endl;
    }
#endif
}

bool GpioChip::SetDirection(bool io)
{
#if defined(__linux__)
    // 设置 GPIO 方向为输出
    if (gpiochip_fd_ < 0) {
        return false;
    }

    // GetChipInfo();
    // GetLineInfo();

    if (io) {
        if (handle_req_.fd > 0) {
            close(handle_req_.fd);
            handle_req_.fd = -1;
        }
        memset(&handle_req_, 0, sizeof(handle_req_));
        handle_req_.lines          = 1;          // 要请求的 GPIO 线路数量
        handle_req_.lineoffsets[0] = gpio_line_; // GPIO 线路号
        // req.lineoffsets[1] = gpio_line_ + 1;
        handle_req_.flags = GPIOHANDLE_REQUEST_OUTPUT; // 设置 GPIO 方向为输出
        // handle_req_.flags = GPIOHANDLE_REQUEST_OUTPUT | GPIOHANDLE_REQUEST_BIAS_PULL_UP;
        // handle_req_.flags = GPIOHANDLE_REQUEST_INPUT; // 设置 GPIO 方向为输入
        handle_req_.default_values[0] = 1; // 默认 GPIO 输出值为高电平
        snprintf(handle_req_.consumer_label, sizeof(handle_req_.consumer_label), "gpio_event_consumer");
        if (ioctl(gpiochip_fd_, GPIO_GET_LINEHANDLE_IOCTL, &handle_req_) < 0) {
            std::cerr << "Failed to get GPIO line handle." << std::endl;
            close(gpiochip_fd_);
            gpiochip_fd_ = -1;
            return false;
        }
    } else {
        if (event_req_.fd > 0) {
            MY_EPOLL.EpollDel(event_req_.fd);
            // poll_ptr_->PollDel(event_req_.fd);
            close(event_req_.fd);
            event_req_.fd = -1;
        }
        RequestEvent();
        MY_EPOLL.EpollAddRead(event_req_.fd, std::bind(&GpioChip::RecvEvent, this));
        // poll_ptr_->PollAddRead(event_req_.fd, std::bind(&GpioChip::RecvEvent, this));
    }

    if (is_output_ != io) {
        is_output_ = io;
    }
#endif
    return true;
}

void GpioChip::RecvEvent(void)
{
#if defined(__linux__)
    struct gpioevent_data event;
    int64_t rd = read(event_req_.fd, &event, sizeof(event));
    if (rd > 0) {
        if (event.id == GPIOEVENT_EVENT_FALLING_EDGE) {
            // 下降沿触发
            gpio_value_ = false;
        } else if (event.id == GPIOEVENT_EVENT_RISING_EDGE) {
            // 上升沿触发
            gpio_value_ = true;
        } else {
            std::cerr << "Unknow id" << event.id << std::endl;
        }
        if (read_function_) {
            read_function_(gpio_value_, event.timestamp);
        }
    }
#endif
}

void GpioChip::SetValue(bool value)
{
#if defined(__linux__)
    // 设置 GPIO 电平
    struct gpiohandle_data data;
    if (handle_req_.fd < 0) {
        return;
    }
    data.values[0] = value;
    if (ioctl(handle_req_.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) == -1) {
        perror("Failed to set GPIO value. ");
        close(handle_req_.fd);
        handle_req_.fd = -1;
    }
#endif
}

bool GpioChip::GetValue()
{
#if defined(__linux__)
    if (event_req_.fd < 0) {
        return ReadValue();
    }
#endif
    return gpio_value_;
}

bool GpioChip::ReadValue()
{
    bool value = false;
#if defined(__linux__)
    // 读取GPIO引脚的值
    struct gpiohandle_data data;
    if (handle_req_.fd < 0) {
        return false;
    }
    if (ioctl(handle_req_.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) < 0) {
        perror("GPIOHANDLE_GET_LINE_VALUES_IOCTL");
        close(handle_req_.fd);
    }
    value = data.values[0];
#endif
    return value;
}

void GpioChip::SetActiveLow(bool act_low)
{
    if (act_low) {
#if defined(__linux__)
        event_req_.handleflags |= GPIOHANDLE_REQUEST_ACTIVE_LOW;
#endif
    } else {
#if defined(__linux__)
        event_req_.handleflags &= ~GPIOHANDLE_REQUEST_ACTIVE_LOW;
#endif
    }
}

void GpioChip::RequestEvent(void)
{
#if defined(__linux__)
    if (gpiochip_fd_ < 0) {
        return;
    }
    memset(&event_req_, 0, sizeof(event_req_));
    event_req_.lineoffset  = gpio_line_;
    event_req_.handleflags = GPIOHANDLE_REQUEST_INPUT;
    // event_req_.handleflags = GPIOHANDLE_REQUEST_INPUT | GPIOHANDLE_REQUEST_BIAS_PULL_UP;
    event_req_.eventflags  = GPIOEVENT_REQUEST_BOTH_EDGES;
    snprintf(event_req_.consumer_label, sizeof(event_req_.consumer_label), "gpio_event_consumer");
    int rv = ioctl(gpiochip_fd_, GPIO_GET_LINEEVENT_IOCTL, &event_req_);
    if (rv < 0) {
        perror("GPIO set event fail ");
        return;
    }
#endif
}

void GpioChip::SetEdge(EdgeType type)
{
#if defined(__linux__)
    if (gpiochip_fd_ < 0) {
        return;
    }
    memset(&event_req_, 0, sizeof(event_req_));
    event_req_.lineoffset  = gpio_line_;
    event_req_.handleflags = GPIOHANDLE_REQUEST_INPUT;
    if (interrupt_type_map_.count(type)) {
        event_req_.eventflags = interrupt_type_map_[type];
    }
    snprintf(event_req_.consumer_label, sizeof(event_req_.consumer_label), "gpio_event_consumer");
    int rv = ioctl(gpiochip_fd_, GPIO_GET_LINEEVENT_IOCTL, &event_req_);
    if (rv < 0) {
        perror("GPIO set event fail ");
        return;
    }
#else
    if (type) {
    }
#endif
}
