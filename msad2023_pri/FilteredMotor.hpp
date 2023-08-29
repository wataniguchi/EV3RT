/*
    FilteredMotor.hpp

    Copyright © 2021 MS Mode 2. All rights reserved.
*/
#ifndef FilteredMotor_hpp
#define FilteredMotor_hpp

#include "Motor.h"
#include "Filter.hpp"

class FilteredMotor : public ev3api::Motor {
public:
    FilteredMotor(ePortM port);
    inline int getPWM() const;
    inline void setPWM(int pwm);
    void setPWMFilter(Filter *filter);
    void setAdjustmentFactor(double d);
    void drive();
protected:
    Filter *fil;
    int original_pwm, filtered_pwm;
    double adjFactor;
};

inline int FilteredMotor::getPWM() const {
    return filtered_pwm;
}

inline void FilteredMotor::setPWM(int pwm) {
    original_pwm = adjFactor * pwm;
}

#endif /* FilteredMotor_hpp */
