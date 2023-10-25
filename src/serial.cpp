#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_imu/serial.h"
#include "ros2_imu/xepoll.h"

Serial::Serial(std::string dev, const int baudrate, bool debug)
    : Communication(dev, baudrate, debug),
      tx_ring_buffer_(128*1024), // 1M缓存
      rx_ring_buffer_(128*1024)
{
    uart_fd_ = OpenSerial();
    assert(uart_fd_ > 0);
    MY_EPOLL.EpollAddRead(uart_fd_, std::bind(&Serial::ReadCallback, this));
}

Serial::~Serial()
{
    CloseSerial();
}

int Serial::OpenSerial()
{
    int iFd = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (iFd < 0) {
        perror(device_.c_str());
        return -1;
    }

    int error = tcgetattr(iFd, &original_port_settings_);
    if (error == -1) {
        CloseSerial();
        std::cerr << "ERROR: Unable to read settings on " << device_ << "."
                  << std::endl;
        return -1;
    }

    int baudrate_code = B0; // Hangup.
    switch (comm_rate_) {
    case 50:
        baudrate_code = B50;
        break;
    case 75:
        baudrate_code = B75;
        break;
    case 110:
        baudrate_code = B110;
        break;
    case 134:
        baudrate_code = B134;
        break;
    case 150:
        baudrate_code = B150;
        break;
    case 200:
        baudrate_code = B200;
        break;
    case 300:
        baudrate_code = B300;
        break;
    case 600:
        baudrate_code = B600;
        break;
    case 1200:
        baudrate_code = B1200;
        break;
    case 1800:
        baudrate_code = B1800;
        break;
    case 2400:
        baudrate_code = B2400;
        break;
    case 4800:
        baudrate_code = B4800;
        break;
    case 9600:
        baudrate_code = B9600;
        break;
    case 19200:
        baudrate_code = B19200;
        break;
    case 38400:
        baudrate_code = B38400;
        break;
    case 57600:
        baudrate_code = B57600;
        break;
    case 115200:
        baudrate_code = B115200;
        break;
    case 230400:
        baudrate_code = B230400;
        break;
    case 460800:
        baudrate_code = B460800;
        break;
    case 500000:
        baudrate_code = B500000;
        break;
    case 576000:
        baudrate_code = B576000;
        break;
    case 921600:
        baudrate_code = B921600;
        break;
    case 1000000:
        baudrate_code = B1000000;
        break;
    default:
        std::cerr << "ERROR: Unable to adjust settings on " << device_ << "."
                  << std::endl;
        return -1;
        break;
    }

    cfsetispeed(&original_port_settings_, baudrate_code);
    cfsetospeed(&original_port_settings_, baudrate_code);

    /*
     * raw mode
     */
    original_port_settings_.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    original_port_settings_.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    original_port_settings_.c_oflag &= ~(OPOST);
    original_port_settings_.c_cflag &= ~(CSIZE | PARENB);
    original_port_settings_.c_cflag |= CS8;

    /*
     * 'DATA_LEN' bytes can be read by serial
     */
    original_port_settings_.c_cc[VMIN]  = DATA_LEN;
    original_port_settings_.c_cc[VTIME] = 150;

    if (tcsetattr(iFd, TCSANOW, &original_port_settings_) < 0) {
        return -1;
    }

    return iFd;
}

void Serial::CloseSerial()
{
    if (uart_fd_ < 0) {
        return;
    }
    MY_EPOLL.EpollDel(uart_fd_);
    tcsetattr(uart_fd_, TCSANOW, &original_port_settings_);
    close(uart_fd_);
    uart_fd_ = -1;
}

std::string Serial::Bytes2String(uint8_t *data, uint32_t len)
{
    char temp[512];
    std::string str("");
    for (size_t i = 0; i < len; i++) {
        sprintf(temp, "%02x ", data[i]);
        str.append(temp);
    }
    return str;
}

void Serial::ReadCallback()
{
#if 1
    uint8_t uart_rx_buf[DATA_LEN];
    int len = read(uart_fd_, uart_rx_buf, sizeof(uart_rx_buf));
    if (len > 0) {
        if (debug_) { // 调试用
            std::cout << Bytes2String(uart_rx_buf, len) << std::endl;
        }
        rx_ring_buffer_.RingBufferIn(uart_rx_buf, len);
    }
#else
    uint8_t uart_rx_buf[1024];
    int len = 0;
    do {
        len = read(uart_fd_, uart_rx_buf, sizeof(uart_rx_buf));
        if (len > 0) {
            rx_ring_buffer_.RingBufferIn(uart_rx_buf, len);
        }
    } while (len > 0);
#endif
}

void Serial::WriteCallback()
{
    int length = tx_ring_buffer_.RingBufferLen();

    if (length > 0) {
        uint8_t uart_tx_buf[DATA_LEN];
        int size = tx_ring_buffer_.RingBufferOut(uart_tx_buf, length);
        write(uart_fd_, uart_tx_buf, size);
    }
}

int Serial::ReadBuffer(uint8_t *const buffer, const int length)
{
    return rx_ring_buffer_.RingBufferOut(buffer, length);
}

int Serial::SendBuffer(const uint8_t *const buffer, const int length)
{
    int ret = write(uart_fd_, buffer, length);
    if(ret < 0) {
        // ROS_ERROR("Serial write with %d", ret);
        return -1;
    } else if(ret < length) {
        // ROS_WARN("%d buff send, total:%d", ret, length);
    }
    return ret;
    // return tx_ring_buffer_.RingBufferIn(buffer, length);
}