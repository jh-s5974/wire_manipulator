#pragma once

#include <cstddef>
#include <atomic>
#include <sched.h>    // sched_yield
#include <semaphore.h>
#include <boost/lockfree/queue.hpp>


namespace rtfw::internal {

    // RT-safe lock-free blocking queue.
    //
    // Design:
    //   - boost::lockfree::queue (bounded, MPMC, no heap allocation at runtime)
    //     handles concurrent push/pop without any mutex.
    //   - POSIX semaphore (sem_t) provides blocking pop for idle workers.
    //
    // Invariant: sem_count == number of items in the queue.
    //   push: enqueue succeeds → sem_post (count++)
    //   pop : sem_wait (count--) → dequeue
    //
    // Priority inversion: lock-free queues hold no lock, so PTHREAD_PRIO_INHERIT
    // is unnecessary — priority inversion cannot occur by construction.
    //
    // stop() semantics: identical to the previous mutex version.
    //   Remaining items are consumed before workers exit.
    template <typename T, size_t Capacity = 1024>
    class BlockingQueue {
    public:
        BlockingQueue() : _stop_token(false) {
            sem_init(&_sem, /*pshared=*/0, /*value=*/0);
        }

        ~BlockingQueue() {
            sem_destroy(&_sem);
        }

        BlockingQueue(const BlockingQueue&) = delete;
        BlockingQueue& operator=(const BlockingQueue&) = delete;

        void push(T value) {
            if (_stop_token.load(std::memory_order_relaxed)) return;

            // Bounded queue: spin if full (should not happen with adequate Capacity).
            while (!_queue.push(value)) {
                if (_stop_token.load(std::memory_order_relaxed)) return;
                sched_yield();
            }

            // Signal one waiting pop() that an item is available.
            sem_post(&_sem);
        }

        // Returns true and sets value on success.
        // Returns false when stop() has been called and the queue is empty.
        bool pop(T& value) {
            while (true) {
                // Block until an item is available or stop() is called.
                sem_wait(&_sem);

                if (_stop_token.load(std::memory_order_acquire)) {
                    // Drain any remaining items before stopping,
                    // mirroring the behaviour of the previous mutex version.
                    if (_queue.pop(value)) return true;
                    return false;
                }

                if (_queue.pop(value)) return true;
                // Another worker raced us on the same sem count; retry.
            }
        }

        void stop() {
            _stop_token.store(true, std::memory_order_release);

            // Wake all potentially blocked pop() callers.
            // Posting Capacity times guarantees every worker is unblocked
            // regardless of how many are currently waiting.
            for (size_t i = 0; i < Capacity; ++i) {
                sem_post(&_sem);
            }
        }

        size_t clear() {
            size_t count = 0;
            T item;
            while (_queue.pop(item)) {
                sem_trywait(&_sem); // keep sem_count in sync
                ++count;
            }
            return count;
        }

    private:
        boost::lockfree::queue<T, boost::lockfree::capacity<Capacity>> _queue;
        sem_t  _sem;
        std::atomic<bool> _stop_token;
    };
};
