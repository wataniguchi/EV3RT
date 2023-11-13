/*
    Plotter.hpp

    Copyright © 2023 MSAD Mode2P. All rights reserved.
*/
#ifndef Plotter_hpp
#define Plotter_hpp

#define TIRE_DIAMETER    100.0F  /* diameter of tire in milimater           */
#define WHEEL_TREAD      128.0F  /* distance between right and left wheels  */

/* use the native alignof implementation with c++11 or above,
   instead of the macro defined in RasPike/include/t_stddef.h */
#if __cplusplus >= 201102L
#ifndef alignof
#define alignof(type) alignof(type)
#endif /* alignof */
#endif /* __cplusplus */

#include "GyroSensor.h"
#include "Motor.h"

/* M_PI and M_TWOPI is NOT available even with math header file under -std=c++11
   because they are not strictly comforming to C++11 standards
   this program is compiled under -std=gnu++11 option */
#include <math.h>

#ifndef M_TWOPI
#define M_TWOPI         (M_PI * 2.0)
#endif

class Plotter {
public:
    Plotter(ev3api::Motor* lm, ev3api::Motor* rm, ev3api::GyroSensor* gs);
    int getDistance();
    int getAzimuth();
    int getDegree();
    int getLocX();
    int getLocY();
    int getAngL();
    int getAngR();
    void setDegree(int deg);
    void plot();
protected:
    ev3api::Motor *leftMotor, *rightMotor;
    ev3api::GyroSensor *gyroSensor;
    double distance, azimuth, locX, locY;
    int prevAngL, prevAngR;
};

#endif /* Plotter_hpp */
