/**
 * @file ring_buffer.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-17
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

class RingBuffer
{
public:
    RingBuffer(uint32_t size);
    ~RingBuffer();

    uint32_t RingBufferIn(const void *in, uint32_t len);
    uint32_t RingBufferOut(void *out, uint32_t len);

    uint32_t RingBufferLen()
    {
        return fifo_in_ - fifo_out_;
    }

    void RingBufferReset()
    {
        fifo_in_ = fifo_out_ = 0;
    }

private:
    uint8_t *fifo_buffer_;
    uint32_t fifo_size_;
    uint32_t fifo_in_;
    uint32_t fifo_out_;

    uint32_t RingBufferSize()
    {
        return fifo_size_;
    }

    uint32_t RingBufferAvail()
    {
        return RingBufferSize() - RingBufferLen();
    }

    bool RingBufferIsEmpty()
    {
        return RingBufferLen() == 0;
    }

    bool RingBufferIsFull()
    {
        return RingBufferAvail() == 0;
    }
};

#endif
