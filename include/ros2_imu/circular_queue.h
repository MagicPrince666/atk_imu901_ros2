/**
 * @file circular_queue.h
 * @author Leo Huang (846863428@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-26
 * @copyright 个人版权所有 Copyright (c) 2023
 */
#ifndef __CIRCULAR_QUEUE_H__
#define __CIRCULAR_QUEUE_H__

const int maxn = 100;

template <class T>
class CircularQueue
{
public:
    CircularQueue();
    CircularQueue(const int len);
    ~CircularQueue();

public:
    bool empty();
    bool full();
    void push(T x);
    void pop();
    T front();
    int size();

public:
    void resize();

private:
    T *m_data;
    int m_front;
    int m_rear;
    int m_length;
};

template <class T>
CircularQueue<T>::CircularQueue()
{
    m_front = 0;
    m_rear = 0;
    m_data = new T[maxn];
    m_length = maxn;
}

template <class T>
CircularQueue<T>::CircularQueue(const int len)
{
    m_front = 0;
    m_rear = 0;
    m_data = new T[len];
    m_length = len;
}

template <class T>
CircularQueue<T>::~CircularQueue()
{
    delete[] m_data;
    m_data = nullptr;
}

template <class T>
bool CircularQueue<T>::empty()
{
    return m_front == m_rear;
}

template <class T>
bool CircularQueue<T>::full()
{
    return m_front == (m_rear + 1) % m_length;
}

template <class T>
void CircularQueue<T>::push(T x)
{
    if (full())
    {
        resize();
    }
    m_data[m_rear] = x;
    m_rear = (m_rear + 1) % m_length;
}

template <class T>
void CircularQueue<T>::pop()
{
    if (empty())
    {
        return;
    }
    m_front = (m_front + 1) % m_length;
}

template <class T>
T CircularQueue<T>::front()
{
    if (!empty())
    {
        return m_data[m_front];
    }
    // return m_data[m_front];
}

template <class T>
int CircularQueue<T>::size()
{
    return (m_rear - m_front + m_length) % m_length;
}

template <class T>
void CircularQueue<T>::resize()
{
    int len = int(m_length * 1.5);
    T *tmp = new T[len];
    int count = 0;
    for (int i = m_front; i != m_rear; i = (i + 1) % m_length)
    {
        tmp[count++] = m_data[i];
    }
    m_front = 0;
    m_rear = count;
    m_length = len;
    delete[] m_data;
    m_data = tmp;
}

#endif
