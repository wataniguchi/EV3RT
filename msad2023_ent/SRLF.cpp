/*
    SRLF.cpp
    Srew Rate Limiter Filter
    for trapezoidal motion

    Copyright © 2021 MS Mode 2. All rights reserved.
*/

#include "SRLF.hpp"
#include <assert.h>

SRLF::SRLF(const double rate) : prevXin(0.0) {
    srewRate = rate;
}

double SRLF::setRate(const double rate) {
    assert(rate >= 0.0);
    double currentRate = srewRate;
    srewRate = rate;
    return currentRate;
}