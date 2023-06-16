/*
    PIDcalculator.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "appusr.hpp"

PIDcalculator::PIDcalculator(double p, double i, double d, int t, int min, int max) {
    kp = p;
    ki = i;
    kd = d;
    diff[1] = INT_MAX; // initialize diff[1]
    deltaT = t;
    minimum = min;
    maximum = max;
    integral = 0.0;
}

PIDcalculator::~PIDcalculator() {}

int PIDcalculator::math_limit(double input, int min, int max) {
    if (input < min) {
        return min;
    } else if (input > max) {
        return max;
    }
    return (int16_t)input;
}

int PIDcalculator::compute(int sensor, int target) {
    double p, i, d;
    
    if ( diff[1] == INT_MAX ) {
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

