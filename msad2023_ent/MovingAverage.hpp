/*
    MovingAverage.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef MovingAverage_hpp
#define MovingAverage_hpp

#include <math.h>

#define _stdev(cnt, sum, ssq) sqrt((((double)(cnt))*ssq-pow((double)(sum),2)) / ((double)(cnt)*((double)(cnt)-1)))

template<typename T, int CAPACITY> class MovingAverage {
private:
    T elements[CAPACITY];
    int index;
    bool filled;
    T sum, ssq;
public:
    MovingAverage();
    void clear();
    T push(T element);
    T mean();
    T stdev();
};

template<typename T, int CAPACITY>
MovingAverage<T, CAPACITY>::MovingAverage() {
    assert (CAPACITY > 0);
    clear();
}

template<typename T, int CAPACITY>
void MovingAverage<T, CAPACITY>::clear() {
    index = 0;
    sum = 0;
    ssq = 0;
    filled = false;
}

template<typename T, int CAPACITY>
T MovingAverage<T, CAPACITY>::push(T element) {
    if (index == CAPACITY) {
        index = 0;
        filled = true;
    }
    if (filled) {
        sum = sum - elements[index] + element;
        ssq = ssq - elements[index] * elements[index] + element * element;
        elements[index++] = element;
        return sum / CAPACITY;
    } else {
        sum = sum + element;
        ssq = ssq + element * element;
        elements[index++] = element;
        return sum / index;
    }
}

template<typename T, int CAPACITY>
T MovingAverage<T, CAPACITY>::mean() {
    if (filled) {
        return sum / CAPACITY;
    } else {
        if (index == 0) {
            return 0;
        } else {
            return sum / index;
        }
    }
}

template<typename T, int CAPACITY>
T MovingAverage<T, CAPACITY>::stdev() {
    if (filled) {
        return _stdev(CAPACITY, sum, ssq);
    } else {
        if (index == 0 || index == 1) {
            return 0;
        } else {
            return _stdev(index, sum, ssq);
        }
    }
}

#endif /* MovingAverage_hpp */