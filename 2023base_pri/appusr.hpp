/*
    appusr.hpp

    Copyright © 2023 MSAD Mode2P. All rights reserved.
*/
#ifndef appusr_hpp
#define appusr_hpp

/* use the native alignof implementation with c++11 or above,
   instead of the macro defined in RasPike/include/t_stddef.h */
#if __cplusplus >= 201102L
#ifndef alignof
#define alignof(type) alignof(type)
#endif /* alignof */
#endif /* __cplusplus */

#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Steering.h"
#include "Clock.h"
using namespace ev3api;

#if defined(MAKE_SIM)
#include "etroboc_ext.h"
#endif

/* M_PI and M_TWOPI is NOT available even with math header file under -std=c++11
   because they are not strictly comforming to C++11 standards
   this program is compiled under -std=gnu++11 option */
#include <math.h>

#include "FilteredMotor.hpp"
#include "SRLF.hpp"
#include "FilteredColorSensor.hpp"
#include "FIR.hpp"
#include "Plotter.hpp"
#include "PIDcalculator.hpp"

/* global variables */
extern FILE*        bt;
extern Clock*       ev3clock;
extern TouchSensor* touchSensor;
extern SonarSensor* sonarSensor;
extern FilteredColorSensor* colorSensor;
extern GyroSensor*  gyroSensor;
extern SRLF*        srlf_l;
extern FilteredMotor*       leftMotor;
extern SRLF*        srlf_r;
extern FilteredMotor*       rightMotor;
extern Motor*       armMotor;
extern Plotter*     plotter;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//#define LOG_ON_CONSOL

/* ##__VA_ARGS__ is gcc proprietary extention.
   this is also where -std=gnu++11 option is necessary */
#ifdef LOG_ON_CONSOL
#define _log(fmt, ...) \
    syslog(LOG_NOTICE, "%08u, %s: " fmt, \
    ev3clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define _logNoAsp(fmt, ...) \
    syslog(LOG_NOTICE, "%08u, %s: " fmt, \
    0, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#else
#define _log(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    ev3clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
    // temp fix 2022/6/20 W.Taniguchi, as Bluetooth not implemented yet
    /* fprintf(bt, "%08u, %s: " fmt "\n", \ */
#define _logNoAsp(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    0, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#endif

/* macro to covert an enumeration constant to a string */
#define STR(var) #var

#if defined(MAKE_SIM)
  /* macro for making program compatible for both left and right courses.
   the default is left course. */ 
  #if defined(MAKE_RIGHT)
    static const int _COURSE = -1;
  #else
    static const int _COURSE = 1;
  #endif /* defined(MAKE_RIGHT) */
#else
static int _COURSE = 1;
#endif

//#ifndef LOG_INTERVAL
//#define LOG_INTERVAL            0
//#endif

#define SONAR_ALERT_DISTANCE    100     /* in millimeter                           */
#define ARM_INITIAL_ANGLE       -58
#define ARM_SHIFT_PWM           100
#define JUNCTION_LOWER_THRESHOLD 45
#define JUNCTION_UPPER_THRESHOLD 60

enum Color {
    CL_JETBLACK,
    CL_BLACK,
    CL_BLUE,
    CL_BLUE2,
    CL_RED,
    CL_YELLOW,
    CL_GREEN,
    CL_GRAY,
    CL_WHITE,
};

enum BoardItem {
    LOCX, /* horizontal location    */
    LOCY, /* virtical   location    */
    DIST, /* accumulated distance   */
};

enum State {
    ST_INITIAL,
    ST_CALIBRATION,
    ST_RUN,
    ST_BLOCK,
    ST_ENDING,
    ST_END,
};

enum TraceSide {
    TS_NORMAL = 0,
    TS_OPPOSITE = 1,
    TS_CENTER = 2,
};

enum JState {
    JST_INITIAL, /* internally used by IsJunction class */
    JST_JOINING,
    JST_JOINED,
    JST_FORKING,
    JST_FORKED,
};

#include <string.h>

#ifndef ENUMPAIR_TYPE_DEFINED
#define ENUMPAIR_TYPE_DEFINED
typedef struct { const char *name; int num; } EnumPair;
#endif

#define EPAIR(arg) {#arg, arg}

static int EnumStringToNum(const EnumPair *enum_data, const char *name, int *out_num) {
  for (; enum_data->name; enum_data++) {
    if (strcmp(enum_data->name, name) == 0) {
      *out_num = enum_data->num;
      return 1; /* success */
    }
  }
  return 0; /* failure */
}

const EnumPair gEnumPairs[] = {
  EPAIR(TS_NORMAL),
  EPAIR(TS_OPPOSITE),
  EPAIR(TS_CENTER),
  { NULL /* terminator */ }
};

#endif /* appusr_hpp */
