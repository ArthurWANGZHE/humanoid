#pragma once
#include <atomic>
#include <cstddef>

/**
 * 单生产者-单消费者（SPSC）无锁环形缓冲区
 */
template<typename T, size_t Size>
class RingBuffer
{
public:
    RingBuffer()
    {
        static_assert(Size > 1 && (Size & (Size - 1)) == 0, 
              "Ring buffer size must be power of 2");
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    inline void push(const T& item)
    {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = increment(head);

        size_t tail = tail_.load(std::memory_order_acquire);

        if (next == tail)
        {
            tail_.store(increment(tail), std::memory_order_release);
        }

        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
    }

    inline bool pop(T& item)
    {
        size_t tail = tail_.load(std::memory_order_relaxed);
        size_t head = head_.load(std::memory_order_acquire);

        if (tail == head)
            return false;

        item = buffer_[tail];
        tail_.store(increment(tail), std::memory_order_release);
        return true;
    }

    inline bool empty() const
    {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    inline bool full() const
    {
        size_t head = head_.load(std::memory_order_acquire);
        size_t next = increment(head);
        return next == tail_.load(std::memory_order_acquire);
    }

    inline size_t size() const
    {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return (head + Size - tail) % Size;
    }

private:
    inline size_t increment(size_t index) const
    {
        return (index + 1) & (Size - 1);
    }

private:
    T buffer_[Size];
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
};

/* ================= 使用示例 =================

LockFreeRingBuffer<ZCAN_ReceiveFD_Data, 4096> rx_queue;

ZCAN_ReceiveFD_Data msg;
while (rx_queue.pop(msg))
{
}

================================================ */
