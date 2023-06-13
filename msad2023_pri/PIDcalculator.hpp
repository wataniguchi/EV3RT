/*
    PIDcalculator.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef PIDcalculator_hpp
#define PIDcalculator_hpp

class PIDcalculator {
private:
    double kp, ki, kd;   /* PID constant */
    int diff[2], deltaT, minimum, maximum;
    double integral;
    int math_limit(double input, int min, int max);
public:
    PIDcalculator(double p, double i, double d, int t, int min, int max);
    int compute(int sensor, int target);
    ~PIDcalculator();
};

#endif /* PIDcalculator_hpp */
