#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include "led.h"

Led::Led(std::string dev)
{
    assert(dev != "");
    led_dev_ = "/sys/class/leds/" + dev + "/brightness";
    std::cout << "open device: " << led_dev_ << std::endl;
    led_fd_ = open(led_dev_.c_str(), O_RDWR);
    assert(led_fd_ > 0);
}

Led::~Led()
{
    if(led_fd_ > 0) {
        SetGpioValue(0);
        close(led_fd_);
    }
}

int Led::SetGpioValue(bool value)
{
    int ret = -1;
    
    if(value) {
        ret = write(led_fd_, "1", 1);
    } else {
        ret = write(led_fd_, "0", 1);
    }

    return ret;
}