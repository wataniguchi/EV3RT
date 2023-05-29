/*
    PIDcalculator.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef PIDcalculator_hpp
#define PIDcalculator_hpp

class PIDcalculator {
private:
    double kp, ki, kd;   /* PID constant */
    int16_t diff[2], deltaT, minimum, maximum, traceCnt;
    double integral;
    int16_t math_limit(int16_t input, int16_t min, int16_t max);
public:
    PIDcalculator(double p, double i, double d, int16_t t, int16_t min, int16_t max);
    int16_t compute(int16_t sensor, int16_t target);
    ~PIDcalculator();
};

#endif /* PIDcalculator_hpp */