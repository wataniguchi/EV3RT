/*
    Plotter.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "Plotter.hpp"

Plotter::Plotter(ev3api::Motor* lm, ev3api::Motor* rm, ev3api::GyroSensor* gs) :
leftMotor(lm),rightMotor(rm),gyroSensor(gs) {
    /* reset motor encoders */
    leftMotor->reset();
    rightMotor->reset();
    /* reset gyro sensor */
    gyroSensor->reset();
    /* initialize variables */
    prevAngL = leftMotor->getCount();
    prevAngR = rightMotor->getCount();
    distance = azimuth = locX = locY = 0.0;
}

int Plotter::getDistance() {
    return (int)distance;
}

int Plotter::getAzimuth() {
    return (int)azimuth;
}

int Plotter::getDegree() {
    // degree = 360.0 * radian / M_TWOPI;
    double degree = (360.0 * azimuth / M_TWOPI);
    if (degree == 360) degree = 0; /* guarantee 0 <= degree <= 359 */ 
    return (int)degree;
}

int Plotter::getLocX() {
    return (int)locX;
}

int Plotter::getLocY() {
    return (int)locY;
}

int Plotter::getAngL() {
    return prevAngL;
}

int Plotter::getAngR() {
    return prevAngR;
}

void Plotter::setDegree(int deg) {
  // double radian = degree * M_TWOPI / 360.0
  azimuth = (double) deg * M_TWOPI / 360.0;
}

void Plotter::plot() {
    /* accumulate distance */
    int32_t curAngL = leftMotor->getCount();
    int32_t curAngR = rightMotor->getCount();
    double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
    double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;
    double deltaDist = (deltaDistL + deltaDistR) / 2.0;
    if (deltaDistL * deltaDistR >= 0) { /* cumulate only when both wheels spin in the same direction */  
      if (deltaDist >= 0) { /* cumulative distance must be always positive */
	distance += deltaDist;
      } else {
	distance -= deltaDist;
      }
    }
    prevAngL = curAngL;
    prevAngR = curAngR;
    /* calculate azimuth */
    double deltaAzi = (deltaDistL - deltaDistR) / WHEEL_TREAD;
    azimuth += deltaAzi;
    if (azimuth >= M_TWOPI) {
        azimuth -= M_TWOPI;
    } else if (azimuth < 0.0) {
        azimuth += M_TWOPI;
    }
    if (deltaDistL * deltaDistR >= 0) { /* change location only when both wheels spin in the same direction */  
      /* estimate location */
      locX += (deltaDist * sin(azimuth));
      locY += (deltaDist * cos(azimuth));
    }
}
