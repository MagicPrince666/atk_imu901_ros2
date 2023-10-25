#include "ros2_imu/ring_buffer.h"
#include <string.h>

#define min(a, b) (((a) < (b)) ? (a) : (b))

RingBuffer::RingBuffer(uint32_t size)
{
    fifo_buffer_ = new uint8_t[size];
    fifo_size_ = size;
    fifo_in_ = fifo_out_ = 0;
}

RingBuffer::~RingBuffer()
{
    delete[] fifo_buffer_;
    fifo_buffer_ = nullptr;
}

uint32_t RingBuffer::RingBufferIn(const void *in, uint32_t len)
{
    len = min(len, RingBufferAvail());

    /* First put the data starting from fifo_in_ to buffer end. */
    uint32_t l = min(len, fifo_size_ - (fifo_in_ & (fifo_size_ - 1)));
    memcpy(fifo_buffer_ + (fifo_in_ & (fifo_size_ - 1)), in, l);

    /* Then put the rest (if any) at the beginning of the buffer. */
    memcpy(fifo_buffer_, (uint8_t *)in + l, len - l);

    fifo_in_ += len;

    return len;
}

uint32_t RingBuffer::RingBufferOut(void *out, uint32_t len)
{
    len = min(len, RingBufferLen());

    /* First get the data from fifo_out_ until the end of the buffer. */
    uint32_t l = min(len, fifo_size_ - (fifo_out_ & (fifo_size_ - 1)));
    memcpy(out, fifo_buffer_ + (fifo_out_ & (fifo_size_ - 1)), l);

    /* Then get the rest (if any) from the beginning of the buffer. */
    memcpy((uint8_t *)out + l, fifo_buffer_, len - l);

    fifo_out_ += len;

    return len;
}
