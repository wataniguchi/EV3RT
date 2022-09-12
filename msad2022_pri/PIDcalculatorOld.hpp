/*
    PIDcalculatorOld.hpp
    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef PIDcalculatorOld_hpp
#define PIDcalculatorOld_hpp

class PIDcalculatorOld {
private:
    double kp, ki, kd;   /* PID constant */
    int16_t diff[2], deltaT, minimum, maximum, traceCnt;
    double integral;
    int16_t math_limit(int16_t input, int16_t min, int16_t max);
public:
    PIDcalculatorOld(double p, double i, double d, int16_t t, int16_t min, int16_t max);
    int16_t compute(int16_t sensor, int16_t target);
    ~PIDcalculatorOld();
};

#endif /* PIDcalculatorOld_hpp */