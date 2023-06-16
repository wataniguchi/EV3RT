/*
    Plotter.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "Plotter.hpp"

Plotter::Plotter(ev3api::Motor* lm, ev3api::Motor* rm, ev3api::GyroSensor* gs) :
distance(0.0),azimuth(0.0),locX(0.0),locY(0.0),leftMotor(lm),rightMotor(rm),gyroSensor(gs) {
    /* reset motor encoders */
    leftMotor->reset();
    rightMotor->reset();
    /* reset gyro sensor */
    gyroSensor->reset();
    /* initialize variables */
    prevAngL = leftMotor->getCount();
    prevAngR = rightMotor->getCount();
}

int32_t Plotter::getDistance() {
    return (int32_t)distance;
}

int16_t Plotter::getAzimuth() {
    return (int32_t)azimuth;
}

int16_t Plotter::getDegree() {
    // degree = 360.0 * radian / M_TWOPI;
    int16_t degree = (360.0 * azimuth / M_TWOPI);
    return degree;
}

int32_t Plotter::getLocX() {
    return (int32_t)locX;
}

int32_t Plotter::getLocY() {
    return (int32_t)locY;
}

int32_t Plotter::getAngL() {
    return prevAngL;
}

int32_t Plotter::getAngR() {
    return prevAngR;
}

void Plotter::plot() {
    /* accumulate distance */
    int32_t curAngL = leftMotor->getCount();
    int32_t curAngR = rightMotor->getCount();
    double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
    double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;
    double deltaDist = (deltaDistL + deltaDistR) / 2.0;
    if (deltaDist >= 0) { /* cumulative distance must be always positive */
      distance += deltaDist;
    } else {
      distance -= deltaDist;
    }
    prevAngL = curAngL;
    prevAngR = curAngR;
    /* calculate azimuth */
    double deltaAzi = (deltaDistL - deltaDistR) / WHEEL_TREAD;
    azimuth += deltaAzi;
    if (azimuth > M_TWOPI) {
        azimuth -= M_TWOPI;
    } else if (azimuth < 0.0) {
        azimuth += M_TWOPI;
    }
    /* estimate location */
    locX += (deltaDist * sin(azimuth));
    locY += (deltaDist * cos(azimuth));
}
