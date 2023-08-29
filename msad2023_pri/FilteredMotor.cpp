/*
    FilteredMotor.cpp

    Copyright © 2021 MS Mode 2. All rights reserved.
*/
#include "FilteredMotor.hpp"

FilteredMotor::FilteredMotor(ePortM port) : Motor(port, true, MEDIUM_MOTOR),fil(nullptr),adjFactor(1.0) {}

void FilteredMotor::setPWMFilter(Filter *filter) {
    fil = filter;
}

void FilteredMotor::setAdjustmentFactor(double d) {
    adjFactor = d;
}

void FilteredMotor::drive() {
    /* process pwm by the Filter */
    if (fil == nullptr) {
        filtered_pwm = original_pwm;
    } else {
        filtered_pwm = fil->apply(original_pwm);
    }
    ev3api::Motor::setPWM(filtered_pwm);
}
