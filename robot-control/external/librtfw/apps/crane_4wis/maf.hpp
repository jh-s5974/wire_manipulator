#pragma once
#include <deque>

template<typename T>
class MAF {
    std::deque<T> buf;
    int capacity;

public:
    MAF(int size=1) : capacity(size) {}

    bool is_full() {
        return buf.size() >= capacity;
    }

    T update(T data) {
        buf.push_back(data);
        if (buf.size() > capacity)
            buf.pop_front();

        T maf = buf.front();
        for (auto i=1; i<buf.size(); i++)
            maf = maf + buf[i];
        return maf / buf.size();
    }

    void reset() {
        buf.clear();
    }

    void resize(int size) {
        capacity = size;
        while (buf.size() > capacity)
            buf.pop_front();
    }
};