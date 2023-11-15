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

#define _debug(x, level) do { \
    if (_DEBUG_LEVEL >= (level)) { x; }		\
  } while (0)

#ifndef LOG_INTERVAL
#define LOG_INTERVAL            10 /* _intervalLog macro prints once every
				       LOG_INTERVAL times of update process execution */
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
#define _intervalLog(fmt, ...) do { \
  if (upd_process_count % LOG_INTERVAL == 0) \
    syslog(LOG_NOTICE, "%08u, %s: " fmt, \
    ev3clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__); \
  } while (0)
#else
#define _log(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    ev3clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define _logNoAsp(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    0, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define _intervalLog(fmt, ...) do { \
  if (upd_process_count % LOG_INTERVAL == 0) \
    printf("%08u, %s: " fmt "\n", \
    ev3clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__); \
  } while (0)
#endif

/* macro to covert an enumeration constant to a string */
#define STR(var) #var

#define SONAR_ALERT_DISTANCE    100     /* in millimeter                           */
#define ARM_SHIFT_PWM            30
#define TVL_ROTATE_POWER         60
#define TVL_HIGH_SPEED           45
#define TVL_ROT_90               70
#define TVL_ROT_180             170
#define TVL_INTER_CIRCLE_DIST_MIN 150
#define TVL_INTER_CIRCLE_DIST_MAX 300
#define TVL_SWEEP_OFF_DIST      100

enum Color {
    CL_JETBLACK,
    CL_BLACK,
    CL_BLUE,
    CL_RED,
    CL_YELLOW,
    CL_GREEN,
    CL_WHITE,
};

enum State {
    ST_INITIAL,
    ST_CALIBRATION,
    ST_RUN,
    ST_BLOCK1,
    ST_BLOCK2,
    ST_BLOCK3,
    ST_BLOCK4,
    ST_BLOCK5,
    ST_BLOCK6,
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

enum ArmDirection {
    AD_UP = -1,
    AD_DOWN = 1,
};

enum BlockType {
    BT_TREASURE = 0,
    BT_DECOY = 1,
    BT_DECOY_ASIF_TREASURE = 2,
};

#ifndef ENUMPAIR_TYPE_DEFINED
#define ENUMPAIR_TYPE_DEFINED
typedef struct { const char *name; int num; } EnumPair;
#endif

#define EPAIR(arg) {#arg, arg}

const EnumPair gEnumPairs[] = {
  EPAIR(TS_NORMAL),
  EPAIR(TS_OPPOSITE),
  EPAIR(TS_CENTER),
  { NULL /* terminator */ }
};

int EnumStringToNum(const EnumPair *enum_data, const char *name, int *out_num);

/* global variables */
extern FILE*        bt;
extern Clock*       ev3clock;
extern TouchSensor* touchSensor;
extern SonarSensor* sonarSensor;
#ifdef WITH_FILTER
extern FilteredColorSensor* colorSensor;
#else
extern ColorSensor* colorSensor;
#endif
extern GyroSensor*  gyroSensor;
extern SRLF*        srlf_l;
extern FilteredMotor*       leftMotor;
extern SRLF*        srlf_r;
extern FilteredMotor*       rightMotor;
extern Motor*       armMotor;
extern Plotter*     plotter;
extern int          _COURSE;
extern int          _DEBUG_LEVEL;
extern int          upd_process_count;

#endif /* appusr_hpp */
