/*
    PIDcalculator.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "appusr.hpp"

PIDcalculator::PIDcalculator(double p, double i, double d, int16_t t, int16_t min, int16_t max) {
    kp = p;
    ki = i;
    kd = d;
    diff[1] = INT16_MAX; // initialize diff[1]
    deltaT = t;
    minimum = min;
    maximum = max;
    traceCnt = 0;
}

PIDcalculator::~PIDcalculator() {}

int16_t PIDcalculator::math_limit(int16_t input, int16_t min, int16_t max) {
    if (input < min) {
        return min;
    } else if (input > max) {
        return max;
    }
    return input;
}

int16_t PIDcalculator::compute(int16_t sensor, int16_t target) {
    double p, i, d;
    
    if ( diff[1] == INT16_MAX ) {
	    diff[0] = diff[1] = sensor - target;
    } else {
        diff[0] = diff[1];
        diff[1] = sensor - target;
    }
    integral += (double)(diff[0] + diff[1]) / 2.0 * deltaT / 1000000.0;
    
    p = kp * diff[1];
    i = ki * integral;
    d = kd * (diff[1] - diff[0]) * 1000000.0 / deltaT;

    return math_limit(p + i + d, minimum, maximum);
}