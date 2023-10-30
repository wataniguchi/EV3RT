/*
    app.cpp

    Copyright © 2023 MSAD Mode2P. All rights reserved.
*/
#include "BrainTree.h"
#include "Profile.hpp"
#include "Video.hpp"
/*
    BrainTree.h must present before ev3api.h on RasPike environment.
    Note that ev3api.h is included by app.h.
*/
#include "app.h"
#include "appusr.hpp"
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <list>
#include <numeric>
#include <math.h>
#include <signal.h>

/* behavior tree stanza files */
#include "tr_calibration.h"
#include "tr_run.h"
#include "tr_block.h"

/* this is to avoid linker error, undefined reference to `__sync_synchronize' */
extern "C" void __sync_synchronize() {}

/* global variables */
FILE*           bt;
Profile*        prof;
Clock*          ev3clock;
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
#ifdef WITH_FILTER
FilteredColorSensor*    colorSensor;
#else
ColorSensor*    colorSensor;
#endif
GyroSensor*     gyroSensor;
SRLF*           srlfL;
FilteredMotor*  leftMotor;
SRLF*           srlfR;
FilteredMotor*  rightMotor;
Motor*          armMotor;
Plotter*        plotter;
Video*          video;
int16_t         guideAngle = 0;
int             guideLocX = 0, guideLocY = 0;
int             vLineRow = 0, vLineColumn = 0, decoyMoved = 0;
int             _COURSE; /* -1 for R course and 1 for L course */
int             _DEBUG_LEVEL; /* used in _debug macro in appusr.hpp */
int             upd_process_count = 0; /* used in _intervalLog macro and
                                          also for BENCHMARK */

BrainTree::BehaviorTree* tr_calibration = nullptr;
BrainTree::BehaviorTree* tr_run         = nullptr;
BrainTree::BehaviorTree* tr_block1      = nullptr;
BrainTree::BehaviorTree* tr_block2      = nullptr;
BrainTree::BehaviorTree* tr_block3      = nullptr;
BrainTree::BehaviorTree* tr_block4      = nullptr;
BrainTree::BehaviorTree* tr_block5      = nullptr;
BrainTree::BehaviorTree* tr_block6      = nullptr;
State state = ST_INITIAL;

std::chrono::system_clock::time_point ts_upd;
#if defined(BENCHMARK)
std::vector<std::uint32_t> upd_interval;
#endif

int vcap_thd_count = 0;
int vcal_thd_count = 0;
int vshow_thd_count = 0;
/* variables for critical section 1 */
std::mutex mut1;
Mat frame_in;
std::chrono::system_clock::time_point te_cap;
/* variables for critical section 2 */
std::mutex mut2;
Mat frame_out;
std::chrono::system_clock::time_point te_cap_copy, te_cal;

int EnumStringToNum(const EnumPair *enum_data, const char *name, int *out_num) {
  for (; enum_data->name; enum_data++) {
    if (strcmp(enum_data->name, name) == 0) {
      *out_num = enum_data->num;
      return 1; /* success */
    }
  }
  return 0; /* failure */
}

/*
    === NODE CLASS DEFINITION STARTS HERE ===
    A Node class serves like a LEGO block while a Behavior Tree serves as a blueprint for the LEGO object built using the LEGO blocks.
*/

/*
    usage:
    ".leaf<ResetClock>()"
    is to reset the clock.
*/
class ResetClock : public BrainTree::Node {
public:
    Status update() override {
        ev3clock->reset();
        _log("clock reset.");
        return Status::Success;
    }
};

/*
    usage:
    ".leaf<ResetArm>()"
    is to reset arm angle.
*/
class ResetArm : public BrainTree::Node {
public:
    ResetArm() : count(0) {}
    Status update() override {
        if (count++ == 0) {
            armMotor->reset();
            _log("arm resetting...");
	} else if (count > 3) { /* give enough time to armMotor */
            _log("arm reset complete.");
            return Status::Success;
	}
        return Status::Running;
    }
protected:
    int count;
};

/*
    usage:
    ".leaf<StopNow>()"
    is to stop the robot.
*/
class StopNow : public BrainTree::Node {
public:
    Status update() override {
        leftMotor->setPWM(0);
        rightMotor->setPWM(0);
        _log("robot stopped.");
        return Status::Success;
    }
};

/*
    usage:
    ".leaf<SetMotorAdjustmentFactors>(adj_factors)"
    is to set adjustment factor for left and right motor
    in order to absorb difference between motors.
    adj_factors is a vector of factors, the first item for the left motor and the second for the right.
*/
class SetMotorAdjustmentFactors : public BrainTree::Node {
public:
  SetMotorAdjustmentFactors(std::vector<double> adj_factors) {
    assert(adj_factors.size() == 2);
    adjFactorLeft  = adj_factors[0];
    adjFactorRight = adj_factors[1];
  }
  Status update() override {
    leftMotor->setAdjustmentFactor(adjFactorLeft);
    rightMotor->setAdjustmentFactor(adjFactorRight);
    _log("motor adjustment factors set");
    return Status::Success;
  }
protected:
  double adjFactorLeft, adjFactorRight;
};

/*
    usage:
    ".leaf<SetPlotterDegree>(deg)"
    is to forcefully set the current degree Plotter class is updating by calculation
    in order to correct cumulated calculation error.
*/
class SetPlotterDegree : public BrainTree::Node {
public:
  SetPlotterDegree(int deg) {
    degree = deg;
  }
  Status update() override {
    int origDeg = plotter->getDegree();
    plotter->setDegree(degree);
    _log("ODO=%05d, Plotter degree forcefully changed from %d to %d.", plotter->getDistance(), origDeg, degree);
    return Status::Success;
  }
protected:
  int degree;
};

/*
    usage:
    ".leaf<IsTouchOn>()"
    is to check if the touch sensor gets pressed.
*/
class IsTouchOn : public BrainTree::Node {
public:
    Status update() override {
        if (touchSensor->isPressed()) {
            _log("touch sensor pressed.");
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
};

/*
    usage:
    ".leaf<IsEnterOn>()"
    is to check if the center button gets pressed.
*/
class IsEnterOn : public BrainTree::Node {
public:
    Status update() override {
        if (ev3_button_is_pressed(LEFT_BUTTON)) {
            _log("center button pressed.");
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
};

/*
    usage:
    ".leaf<IsSonarOn>(distance)"
    is to determine if the robot is closer than the spedified alert distance
    to an object in front of sonar sensor.
    dist is in millimeter.
*/
class IsSonarOn : public BrainTree::Node {
public:
    IsSonarOn(int32_t d) : alertDistance(d) {}
    Status update() override {
        int32_t distance = 10 * (sonarSensor->getDistance());
        if ((distance <= alertDistance) && (distance >= 0)) {
            _log("sonar alert at %d", distance);
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
protected:
    int32_t alertDistance;
};

/*
    usage:
    ".leaf<IsDistanceEarned>(dist)"
    is to determine if the robot has accumulated for the specified distance since update() was invoked for the first time.
    dist is in millimeter.
*/
class IsDistanceEarned : public BrainTree::Node {
public:
    IsDistanceEarned(int32_t d) : deltaDistTarget(d) {
        updated = false;
        earned = false;
    }
    Status update() override {
        if (!updated) {
            originalDist = plotter->getDistance();
            _log("ODO=%05d, Distance accumulation started.", originalDist);
            updated = true;
        }
        int32_t deltaDist = plotter->getDistance() - originalDist;
        
        if ((deltaDist >= deltaDistTarget) || (-deltaDist <= -deltaDistTarget)) {
            if (!earned) {
                _log("ODO=%05d, Delta distance %d is earned.", plotter->getDistance(), deltaDistTarget);
                earned = true;
            }
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    int32_t deltaDistTarget, originalDist;
    bool updated, earned;
};

/*
    usage:
    ".leaf<IsTimeEarned>(time)"
    is to determine if the robot has accumulated for the specified time since update() was invoked for the first time.
    time is in microsecond = 1/1,000,000 second.
*/
class IsTimeEarned : public BrainTree::Node {
public:
    IsTimeEarned(int32_t t) : deltaTimeTarget(t) {
        updated = false;
        earned = false;
    }
    Status update() override {
        if (!updated) {
            originalTime = ev3clock->now();
            _log("ODO=%05d, Time accumulation started.", plotter->getDistance());
             updated = true;
        }
        int32_t deltaTime = ev3clock->now() - originalTime;

        if (deltaTime >= deltaTimeTarget) {
            if (!earned) {
                 _log("ODO=%05d, Delta time %d is earned.", plotter->getDistance(), deltaTimeTarget);
                earned = true;
            }
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    int32_t deltaTimeTarget, originalTime;
    bool updated, earned;
};

bool isColor(Color c, rgb_raw_t cur_rgb) {
  switch(c){
  case CL_BLACK:
    if (cur_rgb.r <= 50 && cur_rgb.g <= 50 && cur_rgb.b <= 50) return true;
    break;
  case CL_BLUE:
    if (cur_rgb.r < 60 && cur_rgb.g < 100 && cur_rgb.b >= 100 &&
	cur_rgb.b - cur_rgb.r >= 60 && cur_rgb.g - cur_rgb.r >= 30) return true;
    break;
  case CL_RED:
    if (cur_rgb.r > 60 && cur_rgb.g < 70 && cur_rgb.b < 70 &&
	cur_rgb.r - cur_rgb.g >= 30) return true;
    break;
  case CL_YELLOW:
    if (cur_rgb.r >= 140 && cur_rgb.g >= 120 && cur_rgb.b <= 120 &&
	cur_rgb.r - cur_rgb.g >= 20 && cur_rgb.g - cur_rgb.b >= 20) return true;
    break;
  case CL_GREEN:
    if (cur_rgb.r <= 60 && cur_rgb.g >= 70 && cur_rgb.b <= 70 &&
	cur_rgb.g - cur_rgb.r >= 30 && cur_rgb.g - cur_rgb.b >= 15) return true;
    break;
  case CL_WHITE:
    if (cur_rgb.r >= 150 && cur_rgb.g >= 150 && cur_rgb.b >= 150) return true;
    break;
  default:
    break;
  }
  return false;
}
  
/*
    usage:
    ".leaf<IsColorDetected>(color)"
    is to determine if the specified color gets detected.
    For possible color that can be specified as the argument, see enum Color in "appusr.hpp".
*/
class IsColorDetected : public BrainTree::Node {
public:
    static Color garageColor;
    IsColorDetected(Color c) : color(c) {
        updated = false;
    }
    Status update() override {
        int currentDist = plotter->getDistance();
        if (!updated) {
            _log("ODO=%05d, Color detection started.", currentDist);
            updated = true;
        }
        rgb_raw_t cur_rgb;
        colorSensor->getRawColor(cur_rgb);
	_debug(_log("ODO=%05d, rgb(%03d,%03d,%03d)", currentDist, cur_rgb.r, cur_rgb.g, cur_rgb.b),3);  /* if _DEBUG_LEVEL >= 3 */

	if (isColor(color, cur_rgb)) {
	  switch(color){
	  case CL_BLACK:
	    _log("ODO=%05d, CL_BLACK detected.", currentDist);
	    break;
	  case CL_BLUE:
	    _log("ODO=%05d, CL_BLUE detected.", currentDist);
	    break;
	  case CL_RED:
	    _log("ODO=%05d, CL_RED detected.", currentDist);
	    break;
	  case CL_YELLOW:
	    _log("ODO=%05d, CL_YELLOW detected.", currentDist);
	    break;
	  case CL_GREEN:
	    _log("ODO=%05d, CL_GREEN detected.", currentDist);
	    break;
	  case CL_WHITE:
	    _log("ODO=%05d, CL_WHITE detected.", currentDist);
	    break;
	  default:
	    break;
	  }
	  return Status::Success;
        } else {
	  return Status::Running;
	}
    }
protected:
    Color color;
    bool updated;
};

/*
    usage:
    ".leaf<IsJunction>(jstate)"
    is to determine if a junction is captured in sight as the specified state.
    jstate is one of the following:
      JST_JOINING: lines are joining
      JST_JOINED: the join completed
      JST_FORKING: lines are forking
      JST_FORKED: the fork completed
*/
class IsJunction : public BrainTree::Node {
public:
    IsJunction(JState s) : targetState(s) {
        updated = false;
	reached = false;
	prevRoe = 0;
        currentState = JST_INITIAL;
    }
    Status update() override {
        if (!updated) {
            _log("ODO=%05d, Junction scan started.", plotter->getDistance());
             updated = true;
        }
	
	int roe = video->getRangeOfEdges();
	_debug(_intervalLog("ODO=%05d, roe = %d", plotter->getDistance(), roe),2); /* if _DEBUG_LEVEL >= 2 */

	if (roe != 0) {
	  switch (currentState) {
	  case JST_INITIAL:
	    if ((targetState == JST_JOINING ||
		 targetState == JST_JOINED) &&
		roe >= JUNCTION_UPPER_THRESHOLD &&
		prevRoe <= JUNCTION_LOWER_THRESHOLD) {
	      currentState = JST_JOINING;
	      _log("ODO=%05d, lines are joining.", plotter->getDistance());
	    } else if ((targetState == JST_FORKING ||
			targetState == JST_FORKED) &&
		       roe >= JUNCTION_LOWER_THRESHOLD &&
		       prevRoe <= JUNCTION_LOWER_THRESHOLD) {
	      currentState = JST_FORKING;
	      _log("ODO=%05d, lines are forking.", plotter->getDistance());
	    }
	    break;
	  case JST_JOINING:
	    if (roe <= JUNCTION_LOWER_THRESHOLD) {
	      currentState = JST_JOINED;
	      _log("ODO=%05d, the join completed.", plotter->getDistance());
	    }
	    break;
	  case JST_FORKING:
	    if (roe <= JUNCTION_LOWER_THRESHOLD &&
		prevRoe >= JUNCTION_UPPER_THRESHOLD) {
	      currentState = JST_FORKED;
	      _log("ODO=%05d, the fork completed.", plotter->getDistance());
	    }
	    break;
	  case JST_JOINED:
	  case JST_FORKED:
	  default:
	    break;
	  }
	}
	prevRoe = roe;

        if (currentState == targetState) {
            if (!reached) {
                 _log("ODO=%05d, Junction state is reached.", plotter->getDistance());
                reached = true;
            }
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    JState targetState, currentState;
    int prevRoe;
    bool updated, reached;
};

/*
    usage:
    ".leaf<IsFoundBlock>(gs_min, gs_max, bgr_min_tre, bgr_max_tre, bgr_min_dec, bgr_max_dec)"
    is to determine the RED block in sight.
    gs_min, gs_max are grayscale threshold for object recognition binalization.
    bgr_min_tre, bgr_max_tre are bgr vector threshold for identifying RED objects.
    bgr_min_dec, bgr_max_dec are bgr vector threshold for identifying BLUE objects.
*/
class IsFoundBlock : public BrainTree::Node {
public:
  IsFoundBlock(int gs_min, int gs_max,
	       std::vector<double> bgr_min_tre, std::vector<double> bgr_max_tre,
	       std::vector<double> bgr_min_dec, std::vector<double> bgr_max_dec) :
    gsMin(gs_min),gsMax(gs_max),bgrMinTre(bgr_min_tre),bgrMaxTre(bgr_max_tre),
    bgrMinDec(bgr_min_dec),bgrMaxDec(bgr_max_dec) {
        updated = false;
	count = inSightCount = 0;
    }
    ~IsFoundBlock() {}
    Status update() override {
        if (!updated) {
	    video->setTraceTargetType(TT_BLKS);
	    video->setThresholds(gsMin, gsMax);
	    video->setMaskThresholds(bgrMinTre, bgrMaxTre, bgrMinDec, bgrMaxDec, {0,0,0}, {0,0,0});
            updated = true;
        }
	if (count++ < 10) {
	  if (video->isTargetInSight()) inSightCount++;
	  return Status::Running;
	} else {
	  if (inSightCount >= 8) {
	    /* when target in sight 8 times out of 10 attempts */
	    _log("ODO=%05d, target IN SIGHT at x=%d:y=%d:deg=%d:gyro=%d",
		 plotter->getDistance(), plotter->getLocX(), plotter->getLocY(),
		 plotter->getDegree(), gyroSensor->getAngle());
	    count = inSightCount = 0;
            return Status::Success;
	  } else {
	    _log("ODO=%05d, target NOT in sight at x=%d:y=%d:deg=%d:gyro=%d",
		 plotter->getDistance(), plotter->getLocX(), plotter->getLocY(),
		 plotter->getDegree(), gyroSensor->getAngle());
	    count = inSightCount = 0;
            return Status::Failure;
	  }
        }
    }
protected:
    int gsMin, gsMax, count, inSightCount;
    std::vector<double> bgrMinTre, bgrMaxTre, bgrMinDec, bgrMaxDec;
    bool updated;
};

/*
    usage:
    ".leaf<ApproachBlock>(speed, pid, gs_min, gs_max, bgr_min_tre, bgr_max_tre, bgr_min_dec, bgr_max_dec)"
    is to instruct the robot to come closer the RED block at the given speed.
    pid is a vector of three constants for PID control.
    gs_min, gs_max are grayscale threshold for object recognition binalization.
    bgr_min_tre, bgr_max_tre are bgr vector threshold for identifying RED objects.
    bgr_min_dec, bgr_max_dec are bgr vector threshold for identifying BLUE objects.
*/
class ApproachBlock : public BrainTree::Node {
public:
  ApproachBlock(int s, std::vector<double> pid, int gs_min, int gs_max,
		std::vector<double> bgr_min_tre, std::vector<double> bgr_max_tre,
		std::vector<double> bgr_min_dec, std::vector<double> bgr_max_dec) :
    speed(s),gsMin(gs_min),gsMax(gs_max),bgrMinTre(bgr_min_tre),bgrMaxTre(bgr_max_tre),
    bgrMinDec(bgr_min_dec),bgrMaxDec(bgr_max_dec) {
        updated = false;
	assert(pid.size() == 3);
        ltPid = new PIDcalculator(pid[0], pid[1], pid[2], PERIOD_UPD_TSK, -speed, speed);
	count = hasCaughtCount = 0;
    }
    ~ApproachBlock() {
        delete ltPid;
    }
    Status update() override {
        if (!updated) {
	    video->setTraceTargetType(TT_BLKS);
	    video->setThresholds(gsMin, gsMax);
	    video->setMaskThresholds(bgrMinTre, bgrMaxTre, bgrMinDec, bgrMaxDec, {0,0,0}, {0,0,0});
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, Approach Block run started.", plotter->getDistance());
            updated = true;
        }

        int8_t forward, turn, pwmL, pwmR;
	int theta = video->getTheta();
	_debug(_log("ODO=%05d, theta = %d", plotter->getDistance(), theta),3); /* if _DEBUG_LEVEL >= 3 */
	
        /* compute necessary amount of steering by PID control */
        turn = (-1) * ltPid->compute(theta, 0); /* 0 is the center */
	_debug(_log("ODO=%05d, turn = %d", plotter->getDistance(), turn),3); /* if _DEBUG_LEVEL >= 3 */
        forward = speed;
        /* steer EV3 by setting different speed to the motors */
        pwmL = forward - turn;
        pwmR = forward + turn;
        leftMotor->setPWM(pwmL);
        rightMotor->setPWM(pwmR);

	if (count++ < 5) {
	  if (video->hasCaughtTarget()) hasCaughtCount++;
	  return Status::Running;
	} else {
	  if (hasCaughtCount > 3) {
	    /* when target determined has caught more than 3 times out of 4 attempts */
	    leftMotor->setPWM(0);
	    rightMotor->setPWM(0);
	    _log("ODO=%05d, Approach Block run ended as CAUGHT TARGET.", plotter->getDistance());
	    count = hasCaughtCount = 0;
	    return Status::Success;
	  } else {
	    count = hasCaughtCount = 0;
	    return Status::Running;
	  }
        }
    }
protected:
    int speed, gsMin, gsMax, count, hasCaughtCount;
    PIDcalculator* ltPid;
    std::vector<double> bgrMinTre, bgrMaxTre, bgrMinDec, bgrMaxDec;
    bool updated;
};

/*
    usage:
    ".leaf<RotateEV3>(30, speed, srew_rate)"
    is to rotate robot 30 degrees (=clockwise in L course and counter-clockwise in R course)
    at the specified speed.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
*/
class RotateEV3 : public BrainTree::Node {
public:
    RotateEV3(int degree, int s, double srew_rate) : deltaDegreeTarget(degree),speed(s),srewRate(srew_rate) {
        updated = false;
	assert(degree >= -180 && degree <= 180);
	deltaDegreeTarget = _COURSE * degree; /* _COURSE = -1 when R course */
        if (deltaDegreeTarget > 0) {
	  clockwise = 1; 
        } else {
	  clockwise = -1;
        }
    }
    Status update() override {
        if (!updated) {
	    originalDegree = plotter->getDegree();
            srlfL->setRate(0.0);
            srlfR->setRate(0.0);
            /* stop the robot at start */
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            _log("ODO=%05d, Rotation for %d started. Current angle = %d", plotter->getDistance(), deltaDegreeTarget, originalDegree);
            updated = true;
            return Status::Running;
        }

	int currentDegree = plotter->getDegree();
	int deltaDegree = (currentDegree - originalDegree);
	/* when    0 <= deltaDegree <= 180,  -90 <= deltaDegree < 269 */
	if (deltaDegreeTarget >= 0 && deltaDegree <  -90) deltaDegree += 360;
	/* when -180 <= deltaDegree <    0, -270 <= deltaDegree <  89 */
	if (deltaDegreeTarget <  0 && deltaDegree >=  90) deltaDegree -= 360;
	  
        if (clockwise * deltaDegree < clockwise * deltaDegreeTarget) {
            if ((srewRate != 0.0) && (clockwise * deltaDegree >= clockwise * deltaDegreeTarget - 5)) {
                /* when comes to the half-way, start decreazing the speed by tropezoidal motion */    
                leftMotor->setPWM(clockwise * 3);
                rightMotor->setPWM(-clockwise * 3);
            } else {
                leftMotor->setPWM(clockwise * speed);
                rightMotor->setPWM((-clockwise) * speed);
            }
            return Status::Running;
        } else {
            /* stop the robot at end */
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            _log("ODO=%05d, Rotation ended. Current angle = %d", plotter->getDistance(), plotter->getDegree());
            return Status::Success;
        }
    }
private:
    int deltaDegreeTarget;
    int16_t originalDegree;
    int clockwise, speed;
    bool updated;
    double srewRate;
};

/*
    usage:
    ".leaf<SetVLineColumn>(column)"
    is to set the current column number of Block Challenge area
    that can be used in TraverseVLine action class.
*/
class SetVLineColumn : public BrainTree::Node {
public:
    SetVLineColumn(int c) : column(c) {} 
    Status update() override {
        vLineColumn = column;
        _log("ODO=%05d, VLine column set to %d", plotter->getDistance(), vLineColumn);
        return Status::Success;
    }
protected:
  int column;
};

/*
    usage:
    ".leaf<TraverseVLine>(speed, target, pidSen, pidCam , gs_min, gs_max, bgr_min_tre, bgr_max_tre, bgr_min_dec, bgr_max_dec, bgr_min_lin, bgr_max_lin)"
    is to instruct the robot to trace back and forth the most plausible virtual line at the given speed while recognizing blocks on the line.
    Note that this class uses both camera and color sensor for trace.

    target is the brightness level for the ideal line for color sensor trace.
    pidSen / pidCam is a vector of three constants for PID control for line trace using color sensor / camera.
    gs_min, gs_max are grayscale threshold for object recognition binalization.
    bgr_min_tre, bgr_max_tre are bgr vector threshold for identifying RED objects.
    bgr_min_dec, bgr_max_dec are bgr vector threshold for identifying BLUE objects.
    bgr_min_lin, bgr_max_lin are bgr vector threshold for identifying BLACK line segments.
    trace_side = TS_NORMAL   when in R(L) course and tracing the right(left) side of the line.
    trace_side = TS_OPPOSITE when in R(L) course and tracing the left(right) side of the line.
    trace_side = TS_CENTER   when tracing the center of the line.
*/
class TraverseVLine : public BrainTree::Node {
public:
  TraverseVLine(int s, int t, std::vector<double> pidSen, std::vector<double> pidCam, int gs_min, int gs_max,
		std::vector<double> bgr_min_tre, std::vector<double> bgr_max_tre,
		std::vector<double> bgr_min_dec, std::vector<double> bgr_max_dec,
		std::vector<double> bgr_min_lin, std::vector<double> bgr_max_lin,
		TraceSide trace_side) :
    speed(s),target(t),gsMin(gs_min),gsMax(gs_max),side(trace_side),
    bgrMinTre(bgr_min_tre),bgrMaxTre(bgr_max_tre),
    bgrMinDec(bgr_min_dec),bgrMaxDec(bgr_max_dec),
    bgrMinLin(bgr_min_lin),bgrMaxLin(bgr_max_lin) {
        updated = false;
	assert(pidSen.size() == 3);
        ltPidSen = new PIDcalculator(pidSen[0], pidSen[1], pidSen[2], PERIOD_UPD_TSK, -speed, speed);
	assert(pidCam.size() == 3);
        ltPidCam = new PIDcalculator(pidCam[0], pidCam[1], pidCam[2], PERIOD_UPD_TSK, -speed, speed);
    }
    ~TraverseVLine() {
        delete ltPidCam;
        delete ltPidSen;
    }
    Status update() override {
        int currentDist = plotter->getDistance();
        if (!updated) {
	    video->setTraceTargetType(TT_VLINE);
	    video->setThresholds(gsMin, gsMax);
	    if (side == TS_NORMAL) {
	      if (_COURSE == -1) { /* right course */
	        video->setTraceSide(1);
	      } else {
	        video->setTraceSide(0);
	      }
	    } else if (side == TS_OPPOSITE) {
	      if (_COURSE == -1) { /* right course */
		video->setTraceSide(0);
	      } else {
		video->setTraceSide(1);
	      }
	    } else {
	      video->setTraceSide(2);
	    }
	    video->setMaskThresholds(bgrMinTre, bgrMaxTre, bgrMinDec, bgrMaxDec, bgrMinLin, bgrMaxLin);
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, VLine traversal started.", currentDist);
	    st = TVLST_INITIAL;
	    circleColor = CL_WHITE;
	    initDist = currentDist;
	    countBlack = countWhite = 0;
	    vLineRow = 0; /* global variable */
	    direction = 1; /* direction = 1 is forward while -1 is reverse */
            updated = true;
        }

        colorSensor->getRawColor(cur_rgb);
	_intervalLog("ODO=%05d, rgb(%03d,%03d,%03d)", currentDist, cur_rgb.r, cur_rgb.g, cur_rgb.b);
	_debug(_log("ODO=%05d, rgb(%03d,%03d,%03d)", currentDist, cur_rgb.r, cur_rgb.g, cur_rgb.b),3); /* if _DEBUG_LEVEL >= 3 */

	switch(st) {
	case TVLST_INITIAL:
	case TVLST_ON_LINE:
	  if (isColor(CL_BLUE, cur_rgb)) {
	    vLineRow += direction;
	    circleDist = currentDist;
	    _log("ODO=%05d, circle CL_BLUE detected with rgb(%03d,%03d,%03d) at Row %d", circleDist, cur_rgb.r, cur_rgb.g, cur_rgb.b, vLineRow);
	    circleColor = CL_BLUE;
	    st = TVLST_ON_CIRCLE;
	  } else if (isColor(CL_RED, cur_rgb)) {
	    vLineRow += direction;
	    circleDist = currentDist;
	    _log("ODO=%05d, circle CL_RED detected with rgb(%03d,%03d,%03d) at Row %d", circleDist, cur_rgb.r, cur_rgb.g, cur_rgb.b, vLineRow);
	    circleColor = CL_RED;
	    st = TVLST_ON_CIRCLE;
	  } else if (isColor(CL_YELLOW, cur_rgb)) {
	    vLineRow += direction;
	    circleDist = currentDist;
	    _log("ODO=%05d, circle CL_YELLOW detected with rgb(%03d,%03d,%03d) at Row %d", circleDist, cur_rgb.r, cur_rgb.g, cur_rgb.b, vLineRow);
	    circleColor = CL_YELLOW;
	    st = TVLST_ON_CIRCLE;
	  } else if (isColor(CL_GREEN, cur_rgb)) {
	    vLineRow += direction;
	    circleDist = currentDist;
	    _log("ODO=%05d, circle CL_GREEN detected with rgb(%03d,%03d,%03d) at Row %d", circleDist, cur_rgb.r, cur_rgb.g, cur_rgb.b, vLineRow);
	    circleColor = CL_GREEN;
	    st = TVLST_ON_CIRCLE;
	  } else if (isColor(CL_BLACK, cur_rgb)) {
	    countWhite = 0; /* reset white counter */
	    if (++countBlack >= 3 && st == TVLST_INITIAL) { /* when CL_BLACK is consequtively detected */
		vLineRow = 1;
		_log("ODO=%05d, appear to be ON LINE. no circle detected at Row 1", currentDist);
		st = TVLST_ON_LINE;
	    } else if (countBlack == 10) { /* force Plotter degree when tracing is stable */
	      int origDeg = plotter->getDegree();
	      int newDeg = 180 + 90 * direction * _COURSE; /* _COURSE = -1 when R course */
	      plotter->setDegree(newDeg);
	      _log("ODO=%05d, Plotter degree forcefully changed from %d to %d.", currentDist, origDeg, newDeg);
 	    }
	  } else if (isColor(CL_WHITE, cur_rgb) && st == TVLST_ON_LINE) {
	    countBlack = 0; /* reset black counter */	    
	    if (++countWhite >= 30) { /* when CL_WHITE is consequtively detected */
	      if (countWhite % 30 == 0) _log("ODO=%05d, *** WARNING - line LOST.", currentDist);
	      //st = TVLST_UNKNOWN;
	    }
	  } else if (st == TVLST_INITIAL && (currentDist - initDist) >= 250) {
	    vLineRow = 1;
	    _log("ODO=%05d, assumed to be ON LINE. no circle detected at Row 1", currentDist);
	    st = TVLST_ON_LINE;
	  }
	  if ( (direction ==  1 && vLineRow >= 4) ||
	       (direction == -1 && vLineRow <= 1) ) {
	    ndChild = new RotateEV3(180, 56, 0.0); /* To-Do: magic numbers */
	    st = TVLST_ABOUT_FACE;
	  }
	  break;
	case TVLST_ON_CIRCLE:
	  if (currentDist - circleDist >= 110) {
	     _log("ODO=%05d, ON LINE is assumed without CL_BLACK detection", currentDist);
	      st = TVLST_ON_LINE;
	  } else if (isColor(CL_BLACK, cur_rgb)) {
	    if (++countBlack >= 3) { /* when CL_BLACK is consequtively detected */
	      _log("ODO=%05d, determined to be ON LINE.", currentDist);
	      st = TVLST_ON_LINE;
	    }
	  } else if (isColor(CL_WHITE, cur_rgb)) {
	    countBlack = 0; /* reset black counter */
	  } else if (isColor(CL_BLUE, cur_rgb)) {
	    if (circleColor == CL_BLUE) {
	      countBlack = 0; /* reset black counter */
	    } else {
	      _log("ODO=%05d, *** WARNING - unexpected CL_BLUE detected with rgb(%03d,%03d,%03d) at Row %d", currentDist, cur_rgb.r, cur_rgb.g, cur_rgb.b, vLineRow);
	      //st = TVLST_UNKNOWN;
	    }
	  } else if (isColor(CL_RED, cur_rgb)) {
	    if (circleColor == CL_RED) {
	      countBlack = 0; /* reset black counter */
	    } else {  
	      _log("ODO=%05d, *** WARNING - unexpected CL_RED detected with rgb(%03d,%03d,%03d) at Row %d", currentDist, cur_rgb.r, cur_rgb.g, cur_rgb.b, vLineRow);
	      //st = TVLST_UNKNOWN;
	    }
	  } else if (isColor(CL_YELLOW, cur_rgb)) {
	    if (circleColor == CL_YELLOW) {
	      countBlack = 0; /* reset black counter */
	    } else {  
	      _log("ODO=%05d, *** WARNING - unexpected CL_YELLOW detected with rgb(%03d,%03d,%03d) at Row %d", currentDist, cur_rgb.r, cur_rgb.g, cur_rgb.b, vLineRow);
	      //st = TVLST_UNKNOWN;
	    }
	  } else if (isColor(CL_GREEN, cur_rgb)) {
	    if (circleColor == CL_GREEN) {
	      countBlack = 0; /* reset black counter */
	    } else {  
	      _log("ODO=%05d, *** WARNING - unexpected CL_GREEN detected with rgb(%03d,%03d,%03d) at Row %d", currentDist, cur_rgb.r, cur_rgb.g, cur_rgb.b, vLineRow);
	      //st = TVLST_UNKNOWN;
	    }
	  }
	  break;
	case TVLST_ABOUT_FACE:
	  stsChild = ndChild->update();
	  if (stsChild == Status::Running) return stsChild;
	  if (stsChild == Status::Success) {
	    delete ndChild;
	    direction *= -1;
	    circleDist = currentDist;
	    _log("ODO=%05d, circleDist is set after rotation", currentDist);
	    st = TVLST_ON_CIRCLE;
	  }
	  break;
	default:
	  break;
	}

	if (st == TVLST_END) {
	    leftMotor->setPWM(0);
	    rightMotor->setPWM(0);
            _log("ODO=%05d, VLine traversal ended.", currentDist);
	    return Status::Success;
	} else if (st == TVLST_UNKNOWN) {
	    leftMotor->setPWM(0);
	    rightMotor->setPWM(0);
            _log("ODO=%05d, VLine traversal FAILED with UNKNOWN state.", currentDist);
	    return Status::Failure;
	} else {
	  int sensor;
	  int forward, turn, pwmL, pwmR;
	
	  if ( (direction ==  1 && (vLineRow < 3 || (vLineRow >= 3 && st != TVLST_ON_LINE))) ||
	       (direction == -1 && (vLineRow > 2 || (vLineRow <= 2 && st != TVLST_ON_LINE))) ) {
	    /* trace line using camera until reaching to Row 3 in forward
	       and until reaching to Row 2 in reverse */
	    int theta = video->getTheta();
	    _debug(_log("ODO=%05d, theta = %d", currentDist, theta),3); /* if _DEBUG_LEVEL >= 3 */
	
	    /* compute necessary amount of steering by PID control */
	    turn = (-1) * ltPidCam->compute(theta, 0); /* 0 is the center */
	  } else { /* row >= 3 && on line in forward
		      and row <=2 && on line in reverse */
	    /* trace line using color sensor from Row 3 to Row 4 in foward
	       and from Row 2 to Row 1 in reverse */
	    sensor = cur_rgb.r;	
	    /* compute necessary amount of steering by PID control */
	    if (side == TS_NORMAL) {
	      turn = (-1) * _COURSE * ltPidSen->compute(sensor, target);
	    } else { /* side == TS_OPPOSITE */
	      turn = _COURSE * ltPidSen->compute(sensor, target);
	    }
	    _intervalLog("ODO=%05d, color sensor trace with r = %d, target_r = %d, turn = %d", currentDist, sensor, target, turn);
	  }
	  
	  _debug(_log("ODO=%05d, turn = %d", currentDist, turn),3); /* if _DEBUG_LEVEL >= 3 */
	  forward = speed;
	  /* steer EV3 by setting different speed to the motors */
	  pwmL = forward - turn;
	  pwmR = forward + turn;
	  leftMotor->setPWM(pwmL);
	  rightMotor->setPWM(pwmR);
	  return Status::Running;
        }
    }
protected:
    enum TVLState {
      TVLST_INITIAL,
      TVLST_ON_CIRCLE,
      TVLST_ON_LINE,
      TVLST_UNKNOWN,
      TVLST_ABOUT_FACE,
      TVLST_END,
    };
    int speed, target, gsMin, gsMax, initDist, circleDist, countBlack, countWhite, direction;
    PIDcalculator *ltPidSen, *ltPidCam;
    TraceSide side;
    std::vector<double> bgrMinTre, bgrMaxTre, bgrMinDec, bgrMaxDec, bgrMinLin, bgrMaxLin;
    TVLState st;
    Color circleColor;
    Node* ndChild;
    Status stsChild;
    rgb_raw_t cur_rgb;
    bool updated;
};

/*
    usage:
    ".leaf<TraceLineCam>(speed, pid, gs_min, gs_max, srew_rate, trace_side)"
    is to instruct the robot to trace the line at the given speed.
    pid is a vector of three constants for PID control.
    gs_min, gs_max are grayscale threshold for line recognition binalization.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
    trace_side = TS_NORMAL   when in R(L) course and tracing the right(left) side of the line.
    trace_side = TS_OPPOSITE when in R(L) course and tracing the left(right) side of the line.
    trace_side = TS_CENTER   when tracing the center of the line.
*/
class TraceLineCam : public BrainTree::Node {
public:
  TraceLineCam(int s, std::vector<double> pid, int gs_min, int gs_max, double srew_rate, TraceSide trace_side) : speed(s),gsMin(gs_min),gsMax(gs_max),srewRate(srew_rate),side(trace_side) {
        updated = false;
	assert(pid.size() == 3);
        ltPid = new PIDcalculator(pid[0], pid[1], pid[2], PERIOD_UPD_TSK, -speed, speed);
	targetType = TT_LINE;
	algo = BA_NORMAL;
    }
    ~TraceLineCam() {
        delete ltPid;
    }
    Status update() override {
        if (!updated) {
	    video->setTraceTargetType(targetType);
	    video->setBinarizationAlgorithm(algo);
	    video->setThresholds(gsMin, gsMax);
	    if (side == TS_NORMAL) {
	      if (_COURSE == -1) { /* right course */
	        video->setTraceSide(1);
	      } else {
	        video->setTraceSide(0);
	      }
	    } else if (side == TS_OPPOSITE) {
	      if (_COURSE == -1) { /* right course */
		video->setTraceSide(0);
	      } else {
		video->setTraceSide(1);
	      }
	    } else {
	      video->setTraceSide(2);
	    }
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, Camera Trace run started. TS=%1d", plotter->getDistance(), (int)side);
            updated = true;
        }

        int8_t forward, turn, pwmL, pwmR;
	int theta = video->getTheta();
	_debug(_log("ODO=%05d, theta = %d", plotter->getDistance(), theta),3); /* if _DEBUG_LEVEL >= 3 */
	
        /* compute necessary amount of steering by PID control */
        turn = (-1) * ltPid->compute(theta, 0); /* 0 is the center */
	_debug(_log("ODO=%05d, turn = %d", plotter->getDistance(), turn),3); /* if _DEBUG_LEVEL >= 3 */
        forward = speed;
        /* steer EV3 by setting different speed to the motors */
	/* TODO: take care of overflow case pwm <= 100 */
        pwmL = forward - turn;
        pwmR = forward + turn;
        srlfL->setRate(srewRate);
        leftMotor->setPWM(pwmL);
        srlfR->setRate(srewRate);
        rightMotor->setPWM(pwmR);
        return Status::Running;
    }
protected:
    int speed, gsMin, gsMax;
    PIDcalculator* ltPid;
    double srewRate;
    TraceSide side;
    bool updated;
    TargetType targetType;
    BinarizationAlgorithm algo;
};

/*
    usage:
    ".leaf<TraceLineCamWithBlockInArm>(speed, pid, gs_min, gs_max, srew_rate, trace_side)"
    is to instruct the robot to trace the line at the given speed,
    while keeping a block in the arm.
    Usage is same as its base class, TraceLineCam 
*/
class TraceLineCamWithBlockInArm : public TraceLineCam {
public:
  TraceLineCamWithBlockInArm (int s, std::vector<double> pid, int gs_min, int gs_max, double srew_rate, TraceSide trace_side) : TraceLineCam(s,pid,gs_min,gs_max,srew_rate,trace_side) {
    targetType = TT_LINE_WITH_BLK;
    algo = BA_NORMAL;
  }
};

/*
    usage:
    ".leaf<TraceLine>(speed, target, pid, srew_rate, trace_side)"
    is to instruct the robot to trace the line at the given speed.
    pid is a vector of three constants for PID control.
    target is the brightness level for the ideal line for trace.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
    trace_side = TS_NORMAL   when in R(L) course and tracing the right(left) side of the line.
    trace_side = TS_OPPOSITE when in R(L) course and tracing the left(right) side of the line.
*/
class TraceLine : public BrainTree::Node {
public:
  TraceLine(int s, int t, std::vector<double> pid, double srew_rate, TraceSide trace_side) : speed(s),target(t),srewRate(srew_rate),side(trace_side) {
        updated = false;
	assert(pid.size() == 3);
        ltPid = new PIDcalculator(pid[0], pid[1], pid[2], PERIOD_UPD_TSK, -speed, speed);
    }
    ~TraceLine() {
        delete ltPid;
    }
    Status update() override {
        if (!updated) {
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, Trace run started.", plotter->getDistance());
            updated = true;
        }

        int sensor;
        int8_t forward, turn, pwmL, pwmR;
        rgb_raw_t cur_rgb;

        colorSensor->getRawColor(cur_rgb);
        sensor = cur_rgb.r;	
        /* compute necessary amount of steering by PID control */
        if (side == TS_NORMAL) {
            turn = (-1) * _COURSE * ltPid->compute(sensor, target);
        } else { /* side == TS_OPPOSITE */
            turn = _COURSE * ltPid->compute(sensor, target);
        }
	_debug(_log("ODO=%05d, turn = %d", plotter->getDistance(), turn),3); /* if _DEBUG_LEVEL >= 3 */
        forward = speed;
        /* steer EV3 by setting different speed to the motors */
        pwmL = forward - turn;
        pwmR = forward + turn;
        srlfL->setRate(srewRate);
        leftMotor->setPWM(pwmL);
        srlfR->setRate(srewRate);
        rightMotor->setPWM(pwmR);
        return Status::Running;
    }
protected:
    int speed, target;
    PIDcalculator* ltPid;
    double srewRate;
    TraceSide side;
    bool updated;
};

/*
    usage:
    ".leaf<RunAsInstructed>(pwm_l, pwm_r, srew_rate)"
    is to move the robot at the instructed speed.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
*/
class RunAsInstructed : public BrainTree::Node {
public:
    RunAsInstructed(int pwm_l, int pwm_r, double srew_rate) : pwmL(pwm_l),pwmR(pwm_r),srewRate(srew_rate) {
        updated = false;
        if (_COURSE == -1) {
            int pwm = pwmL;
            pwmL = pwmR;
            pwmR = pwm;            
        }     
    }
    Status update() override {
        if (!updated) {
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, Instructed run started.", plotter->getDistance());
            updated = true;
        }
        srlfL->setRate(srewRate);
        leftMotor->setPWM(pwmL);
        srlfR->setRate(srewRate);
        rightMotor->setPWM(pwmR);
        return Status::Running;
    }
protected:
    int pwmL, pwmR;
    double srewRate;
    bool updated;
};

/*
    usage:
    ".leaf<SetGuideLocation>()"
    is to remember the current location that can be used later as a guide / basis.
*/
class SetGuideLocation : public BrainTree::Node {
public:
    Status update() override {
        guideLocX = plotter->getLocX();
        guideLocY = plotter->getLocY();
        _log("ODO=%05d, Guide position set as X = %d, Y = %d", plotter->getDistance(), guideLocX, guideLocY);
        return Status::Success;
    }
};

/*
    usage:
    ".leaf<IsXdiffFromGuideLocationLarger>(value)"
    is to determine if the X-difference between the robot Guide Location is larger than the specified value.
*/
class IsXdiffFromGuideLocationLarger : public BrainTree::Node {
public:
    IsXdiffFromGuideLocationLarger(int v) : value(v) {
        updated = false;
        earned = false;
    }
    Status update() override {
        if (!updated) {
	    _log("ODO=%05d, Xdiff comparison to %d started at X = %d, Y = %d.", plotter->getDistance(), value,
	       plotter->getLocX(), plotter->getLocY());
            updated = true;
        }
	int currentLocX = plotter->getLocX();
	int currentLocY = plotter->getLocY();
        int delta = currentLocX - guideLocX;
	if (delta < 0) delta = -delta;
        
        if (delta >= value) {
            if (!earned) {
	        _log("ODO=%05d, Xdiff is larger than %d at X = %d, Y = %d.", plotter->getDistance(), value,
		     currentLocX, currentLocY);
                earned = true;
            }
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
protected:
    int value;
    bool updated, earned;
};

/*
    usage:
    ".leaf<IsYdiffFromGuideLocationLarger>(value)"
    is to determine if the Y-difference between the robot Guide Location is larger than the specified value.
*/
class IsYdiffFromGuideLocationLarger : public BrainTree::Node {
public:
    IsYdiffFromGuideLocationLarger(int v) : value(v) {
        updated = false;
        earned = false;
    }
    Status update() override {
        if (!updated) {
	    _log("ODO=%05d, Ydiff comparison to %d started at X = %d, Y = %d.", plotter->getDistance(), value,
	       plotter->getLocX(), plotter->getLocY());
            updated = true;
        }
	int currentLocX = plotter->getLocX();
	int currentLocY = plotter->getLocY();
        int delta = currentLocY - guideLocY;
	if (delta < 0) delta = -delta;
        
        if (delta >= value) {
            if (!earned) {
	        _log("ODO=%05d, Ydiff is larger than %d at X = %d, Y = %d.", plotter->getDistance(), value,
		     currentLocX, currentLocY);
                earned = true;
            }
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
protected:
    int value;
    bool updated, earned;
};

/*
    usage:
    ".leaf<SetGuideAngle>()"
    is to remember the direction that can be used later as a guide / basis.
*/
class SetGuideAngle : public BrainTree::Node {
public:
    Status update() override {
        guideAngle = plotter->getDegree();
        _log("ODO=%05d, Guide angle set as %d", plotter->getDistance(), guideAngle);
        return Status::Success;
    }
};

/*
    usage:
    ".leaf<RunPerGuideAngle>(offset, speed, pid)"
    is to move robot toward the saved guide angle with offset at the specified speed.
    pid is a vector of three constants for PID control.
*/
class RunPerGuideAngle : public BrainTree::Node {
public:
    RunPerGuideAngle(int off, int s, std::vector<double> pid) : offset(off),speed(s) {
        updated = false;
	assert(pid.size() == 3);
        ltPid = new PIDcalculator(pid[0], pid[1], pid[2], PERIOD_UPD_TSK, -speed, speed);
        assert(off >= -180 && off <= 180);
    }
    Status update() override {
        if (!updated) {
  	    targetAngle = _COURSE * offset + guideAngle; /* _COURSE = -1 when R course */
            if (targetAngle > 180) {
                targetAngle -= 360;
            } else if (targetAngle < -180) {
                targetAngle += 360;
            }
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, MovePerGuideAngle started. Target angle = %d, Current angle = %d", plotter->getDistance(), targetAngle, plotter->getDegree());
            updated = true;
            return Status::Running;
        }

        int deltaAngle = targetAngle - plotter->getDegree();
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle < -180) {
            deltaAngle += 360;
        }

        int turn = ltPid->compute(0, deltaAngle);
        /* steer EV3 by setting different speed to the motors */
        int pwmL = speed - turn;
        int pwmR = speed + turn;
        //srlfL->setRate(0.0);
        leftMotor->setPWM(pwmL);
        //srlfR->setRate(0.0);
        rightMotor->setPWM(pwmR);
        return Status::Running;
    }
private:
    int offset, speed, targetAngle;
    PIDcalculator* ltPid;
    bool updated;
};

/*
    usage:
    ".leaf<ScanBlock>(max_rotate, degree, speed, pid, gs_min, gs_max, bgr_min_tre, bgr_max_tre, bgr_min_dec, bgr_max_dec)"
    is to instruct the robot to scan the RED block in certain move pattern using IsFoundBlock and RotateEV3 class.
    max_rotate is a number of rotating move per a scan.
    degree is for a per move. e.g., max_rotate = 4, degree = 45 sets 4*45 = 180 scan range.
    gs_min, gs_max are grayscale threshold for object recognition binalization.
    bgr_min_tre, bgr_max_tre are bgr vector threshold for identifying RED objects.
    bgr_min_dec, bgr_max_dec are bgr vector threshold for identifying BLUE objects.
*/
class ScanBlock : public BrainTree::Node {
protected:
enum ChildType {
  CT_ISFOUND, /* IsFoundBlock   */
  CT_ROTATE, /* RotateEV3 */
};
public:
  ScanBlock(int max_rotate, int d, int s, int gs_min, int gs_max,
		std::vector<double> bgr_min_tre, std::vector<double> bgr_max_tre,
	    std::vector<double> bgr_min_dec, std::vector<double> bgr_max_dec) : maxRotate(max_rotate),degree(d),speed(s),gsMin(gs_min),gsMax(gs_max) {
        updated = false;
	assert(bgr_min_tre.size() == 3);
	assert(bgr_max_tre.size() == 3);
	assert(bgr_min_dec.size() == 3);
	assert(bgr_max_dec.size() == 3);
	vBgrMinTre = bgr_min_tre;
	vBgrMaxTre = bgr_max_tre;
	vBgrMinDec = bgr_min_dec;
	vBgrMaxDec = bgr_max_dec;
    }
    ~ScanBlock() {}
    Status update() override {
        if (!updated) {
	    cntRotate = 0;
	    ndChild = new IsFoundBlock(gsMin, gsMax, vBgrMinTre, vBgrMaxTre, vBgrMinDec, vBgrMaxDec);
	    ct = CT_ISFOUND;
            _log("ODO=%05d, ScanBlock started.", plotter->getDistance());
            updated = true;
        }
	status = ndChild->update();
	if (status == Status::Running) return status;
	if (status == Status::Failure && ct == CT_ISFOUND) {
	  delete ndChild;
	  if (cntRotate++ >= maxRotate) {
            _log("ODO=%05d, ScanBlock failed!!!", plotter->getDistance());
  	    return Status::Failure;
	    //cntRotate = 0;
	    //degree *= -1; /* switch rotating direction */
	  }
	  ndChild = new RotateEV3(degree, speed, 0.0);
	  ct = CT_ROTATE;
	  return Status::Running;
	}
	if (status == Status::Success && ct == CT_ISFOUND) {
	  delete ndChild;
          _log("ODO=%05d, ScanBlock ended.", plotter->getDistance());
	  return status;
	}
	if (status == Status::Success && ct == CT_ROTATE) {
	  delete ndChild;
	  ndChild = new IsFoundBlock(gsMin, gsMax, vBgrMinTre, vBgrMaxTre, vBgrMinDec, vBgrMaxDec);
	  ct = CT_ISFOUND;
	  return Status::Running;
	}
        _log("ODO=%05d, ScanBlock *** invalid state ***", plotter->getDistance());
	return Status::Invalid;
    }
protected:
    int maxRotate, cntRotate, degree, speed, gsMin, gsMax;
    std::vector<double> vBgrMinTre, vBgrMaxTre, vBgrMinDec, vBgrMaxDec;
    Node* ndChild;
    ChildType ct;
    Status status;
    bool updated;
};

/*
    usage:
    ".leaf<SetArmPosition>(target_degree, pwm)"
    is to shift the robot arm to the specified degree by the spefied power.
    target_angle should be between about 0 (highest) and 110 (lowest). 
*/
class SetArmPosition : public BrainTree::Node {
public:
    SetArmPosition(int32_t target_degree, int pwm) : targetDegree(target_degree),pwmA(pwm) {
        updated = false;
    }
    Status update() override {
        int32_t currentDegree = armMotor->getCount();
        if (!updated) {
            _log("ODO=%05d, Arm position is moving from %d to %d.", plotter->getDistance(), currentDegree, targetDegree);
            if (currentDegree == targetDegree) {
	        armMotor->stop();
                return Status::Success; /* do nothing */
            } else if (currentDegree < targetDegree) {
                clockwise = 1;
            } else {
                clockwise = -1;
            }
            armMotor->setPWM(clockwise * pwmA);
            updated = true;
            return Status::Running;
        }
        if (((clockwise ==  1) && (currentDegree >= targetDegree)) ||
            ((clockwise == -1) && (currentDegree <= targetDegree))) {
	    armMotor->stop();
            _log("ODO=%05d, Arm position set to %d.", plotter->getDistance(), currentDegree);
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
private:
    int32_t targetDegree;
    int pwmA, clockwise;
    bool updated;
};

/*
    usage:
    ".leaf<ArmUpDownFull>(direction)"
    is to raise or lower the arm to the full.
    direction = AD_UP for raising the arm.
    direction = AD_DOWN for lowering the arm.
*/
class ArmUpDownFull : public BrainTree::Node {
public:
    ArmUpDownFull(int d) : degree(INT32_MAX),direction(d) {}
    Status update() override {
        int32_t currentDegree = armMotor->getCount();
	if (currentDegree == degree) { /* arm gets stuck */
	  if (count++ > 10) {
	      armMotor->stop();
              _log("ODO=%05d, Arm position set to %d.", plotter->getDistance(), currentDegree);
              return Status::Success;
	  }
	} else {
	  count = 0;
	}
	degree = currentDegree;
	armMotor->setPWM(ARM_SHIFT_PWM * direction);
	return Status::Running;
    }
protected:
    int32_t degree;
    int count;
    int direction;
};

/*
    === NODE CLASS DEFINITION ENDS HERE ===
*/

/* classes running as sub-threads */
class vcap_thd {
public:
  void operator()(int unused) {
    Mat f;
#if defined(BENCHMARK)
    std::vector<std::uint32_t> elaps_read;
#endif
    while (state != ST_END && state != ST_ENDING) {
#if defined(BENCHMARK)
      std::chrono::system_clock::time_point ts_cap_local = std::chrono::system_clock::now();
#endif
      f = video->readFrame();
      std::chrono::system_clock::time_point te_cap_local = std::chrono::system_clock::now();
      /* critical section 1 */
      if ( mut1.try_lock() ) {
	frame_in = f.clone();
	te_cap = te_cap_local;
	mut1.unlock();
#if defined(BENCHMARK)
	  elaps_read.push_back(std::chrono::duration_cast<std::chrono::microseconds>(te_cap_local - ts_cap_local).count());
#endif
	vcap_thd_count++;
      }
      std::this_thread::yield();
    }
    _logNoAsp("sub-thread ready to join. # of execution = %d", vcap_thd_count);
#if defined(BENCHMARK)
    _logNoAsp("elapsed time for reading frames (micro sec): max = %d, min = %d, mean = %d",
	 (int)(*std::max_element(std::begin(elaps_read),std::end(elaps_read))),
	 (int)(*std::min_element(std::begin(elaps_read),std::end(elaps_read))),
	 (int)(std::accumulate(std::begin(elaps_read),std::end(elaps_read),0) / vcap_thd_count));
#endif
  }
};

class vcal_thd {
public:
  void operator()(int unused) {
#if defined(BENCHMARK)
    std::vector<std::uint32_t> elaps_till_cal;
#endif
    std::chrono::system_clock::time_point te_cap_local;
    while (state != ST_END && state != ST_ENDING) {
      Mat f;
      /* critical section 1 */
      mut1.lock();
      if (te_cap == te_cap_local) {
	mut1.unlock(); /* if frame is not changed, skip it */
      } else {
	f = frame_in.clone();
	te_cap_local = te_cap;
	mut1.unlock();
	f = video->calculateTarget(f);
	std::chrono::system_clock::time_point te_cal_local = std::chrono::system_clock::now();
	/* critical section 2 */
	if ( mut2.try_lock() ) {
	  frame_out = f.clone();
	  te_cap_copy = te_cap_local;
	  te_cal = te_cal_local;
	  mut2.unlock();
#if defined(BENCHMARK)
	  elaps_till_cal.push_back(std::chrono::duration_cast<std::chrono::microseconds>(te_cal_local - te_cap_local).count());
#endif
	  vcal_thd_count++;
	}
      }
      std::this_thread::yield();
    }
    _logNoAsp("sub-thread ready to join. # of execution = %d", vcal_thd_count);
#if defined(BENCHMARK)
    _logNoAsp("elapsed time from capture to calculation (micro sec): max = %d, min = %d, mean = %d",
	 (int)(*std::max_element(std::begin(elaps_till_cal),std::end(elaps_till_cal))),
	 (int)(*std::min_element(std::begin(elaps_till_cal),std::end(elaps_till_cal))),
	 (int)(std::accumulate(std::begin(elaps_till_cal),std::end(elaps_till_cal),0) / vcal_thd_count));
#endif
  }
};

class vshow_thd {
public:
  void operator()(int unused) {
#if defined(BENCHMARK)
    std::vector<std::uint32_t> elaps_till_show;
#endif
    std::chrono::system_clock::time_point te_cap_local;
    while (state != ST_END && state != ST_ENDING) {
      std::chrono::system_clock::time_point te_cal_local, te_show;
      Mat f;
      /* critical section 2 */
      if ( mut2.try_lock() ) {
	if (te_cap_copy != te_cap_local) { /* new frame? */
	  f = frame_out.clone();
	  te_cap_local = te_cap_copy;
	  te_cal_local = te_cal;
	  mut2.unlock();
	  video->writeFrame(f);
	  video->show();
	  te_show = std::chrono::system_clock::now();
#if defined(BENCHMARK)
	  elaps_till_show.push_back(std::chrono::duration_cast<std::chrono::microseconds>(te_show - te_cap_local).count());
#endif
	  vshow_thd_count++;
	} else {
	  mut2.unlock();
	}
	std::this_thread::yield();
      }
    }
    _logNoAsp("sub-thread ready to join. # of execution = %d", vshow_thd_count);
#if defined(BENCHMARK)
    _logNoAsp("elapsed time from capture to transmission (micro sec): max = %d, min = %d, mean = %d",
	 (int)(*std::max_element(std::begin(elaps_till_show),std::end(elaps_till_show))),
	 (int)(*std::min_element(std::begin(elaps_till_show),std::end(elaps_till_show))),
	 (int)(std::accumulate(std::begin(elaps_till_show),std::end(elaps_till_show),0) / vshow_thd_count));
#endif
  }
};

/* a cyclic handler to activate a task */
void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK || E_QOVR);
    if (ercd != E_OK) {
        syslog(LOG_NOTICE, "act_tsk() returned %d", ercd);
    }
}

/* The main task */
void main_task(intptr_t unused) {
    /* create and initialize EV3 objects */
    ev3clock    = new Clock();
    _log("initialization started.");
    /* mask signals */
    sigset_t ss;  
    sigemptyset(&ss);
    sigaddset(&ss, SIGUSR2);
    sigaddset(&ss, SIGALRM);
    sigaddset(&ss, SIGPOLL);
    sigaddset(&ss, SIGIO);
    _log("masking signals while instantiating Video object...");
    pthread_sigmask(SIG_BLOCK, &ss, 0);
    video       = new Video();
    pthread_sigmask(SIG_UNBLOCK, &ss, 0);
    _log("Video object instantiated.");
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_3);
#ifdef WITH_FILTER
    colorSensor = new FilteredColorSensor(PORT_2);
#else
    colorSensor = new ColorSensor(PORT_2);
#endif
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new FilteredMotor(PORT_C);
    rightMotor  = new FilteredMotor(PORT_B);
    armMotor    = new Motor(PORT_A);
    plotter     = new Plotter(leftMotor, rightMotor, gyroSensor);
    /* read profile files and make the profile object ready */
    prof        = new Profile("msad2023_pri/*profile.txt");
    /* determine the course L or R */
    if (prof->getValueAsStr("COURSE") == "R") {
      _COURSE = -1;
    } else {
      _COURSE = 1;
    }
    _DEBUG_LEVEL = prof->getValueAsNum("DEBUG_LEVEL");

#ifdef WITH_FILTER
    /* FIR parameters for a low-pass filter with normalized cut-off frequency of 0.2
        using a function of the Hamming Window */
    const int FIR_ORDER = 4; 
    const double hn[FIR_ORDER+1] = { 7.483914270309116e-03, 1.634745733863819e-01, 4.000000000000000e-01, 1.634745733863819e-01, 7.483914270309116e-03 };
    /* set filters to FilteredColorSensor */
    Filter *lpf_r = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_g = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_b = new FIR_Transposed(hn, FIR_ORDER);
    colorSensor->setRawColorFilters(lpf_r, lpf_g, lpf_b);
#endif
    
    gyroSensor->reset();
    leftMotor->reset();
    srlfL = new SRLF(0.0);
    leftMotor->setPWMFilter(srlfL);
    leftMotor->setPWM(0);
    rightMotor->reset();
    srlfR = new SRLF(0.0);
    rightMotor->setPWMFilter(srlfR);
    rightMotor->setPWM(0);
    armMotor->reset();

/*
    === BEHAVIOR TREE DEFINITION STARTS HERE ===
    A Behavior Tree serves as a blueprint for a LEGO object
    while a Node class serves as each Lego block used in the object.
*/
    tr_calibration = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_CALIBRATION .build();

    if (prof->getValueAsStr("COURSE") == "R") {
      tr_run   = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_RUN_R   .build();
      tr_block1 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK1_R .build();
      tr_block2 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK2_R .build();
      tr_block3 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK3_R .build();
      tr_block4 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK4_R .build();
      tr_block5 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK5_R .build();
      tr_block6 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK6_R .build();
    } else {
      tr_run   = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_RUN_L   .build();
      tr_block1 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK1_L .build();
      tr_block2 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK2_L .build();
      tr_block3 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK3_L .build();
      tr_block4 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK4_L .build();
      tr_block5 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK5_L .build();
      tr_block6 = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK6_L .build();
    }
/*
    === BEHAVIOR TREE DEFINITION ENDS HERE ===
*/
    /* start sub-threads not managed by ASP */
    std::vector<std::thread> thds;
    int iUnused = 0;
    _log("masking signals while starting sub-threads...");
    pthread_sigmask(SIG_BLOCK, &ss, 0);
    thds.emplace_back(vcap_thd(), iUnused);
    thds.emplace_back(vshow_thd(), iUnused);
    thds.emplace_back(vcal_thd(), iUnused);
    pthread_sigmask(SIG_UNBLOCK, &ss, 0); /* let ASP manage the main thread */
    _log("sub-threads started.");
    
    /* register cyclic handler to EV3RT */
    _log("starting update task...");
    sta_cyc(CYC_UPD_TSK);

    /* indicate initialization completion by LED color */
    _log("initialization completed.");
    state = ST_CALIBRATION;

    /* the main task sleep until being waken up and let the registered cyclic handler to traverse the behavir trees */
    _log("going to sleep...");
    ER ercd = slp_tsk();
    assert(ercd == E_OK);
    if (ercd != E_OK) {
        syslog(LOG_NOTICE, "slp_tsk() returned %d", ercd);
    }

    _log("stopping update task...");
    /* deregister cyclic handler from EV3RT */
    stp_cyc(CYC_UPD_TSK);

    _log("wait for update task to cease, going to sleep 100 milli secs");
    ev3clock->sleep(100000);
    _log("update process stopped. # of execution = %d", upd_process_count);
#if defined(BENCHMARK)
    _log("update process interval (micro sec): max = %d, min = %d, mean = %d",
	 (int)(*std::max_element(std::begin(upd_interval),std::end(upd_interval))),
	 (int)(*std::min_element(std::begin(upd_interval),std::end(upd_interval))),
	 (int)(std::accumulate(std::begin(upd_interval),std::end(upd_interval),0) / upd_process_count));
#endif
    
    /* join the sub-threads */
    _log("joining sub-threads...");
    for (auto& t : thds) {
      t.join();
    }

    /* destroy behavior tree */
    delete tr_block6;
    delete tr_block5;
    delete tr_block4;
    delete tr_block3;
    delete tr_block2;
    delete tr_block1;
    delete tr_run;
    delete tr_calibration;
    /* destroy profile object */
    delete prof;
    /* destroy EV3 objects */
#ifdef WITH_FILTER
    delete lpf_b;
    delete lpf_g;
    delete lpf_r;
#endif
    delete plotter;
    delete armMotor;
    delete rightMotor;
    delete leftMotor;
    delete gyroSensor;
    delete colorSensor;
    delete sonarSensor;
    delete touchSensor;
    delete video;
    _log("being terminated...");
    delete ev3clock;
    ext_tsk();
}

/* periodic task to update the behavior tree */
void update_task(intptr_t unused) {
    BrainTree::Node::Status status;
    ER ercd;
    std::chrono::system_clock::time_point ts_upd_local = std::chrono::system_clock::now();
#if defined(BENCHMARK)
    if ( ts_upd.time_since_epoch().count() != 0 ) {
      upd_interval.push_back(std::chrono::duration_cast<std::chrono::microseconds>(ts_upd_local - ts_upd).count());
    }
#endif
    ts_upd = ts_upd_local;
    upd_process_count++;

#ifdef WITH_FILTER
    colorSensor->sense();
#endif
    plotter->plot();

/*
    === STATE MACHINE DEFINITION STARTS HERE ===
    The robot behavior is defined using HFSM (Hierarchical Finite State Machine) with two hierarchies as a whole where:
    - The upper layer is implemented as a state machine here.
    - The lower layer is implemented using Behavior Tree where each tree gets traversed within each corresponding state of the state machine.
*/
    switch (state) {
    case ST_CALIBRATION:
        if (tr_calibration != nullptr) {
            status = tr_calibration->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_RUN;
                _log("State changed: ST_CALIBRATION to ST_RUN");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_CALIBRATION to ST_ENDING");
                break;
            default:
                break;
            }
        }
        break;
    case ST_RUN:
        if (tr_run != nullptr) {
            status = tr_run->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_BLOCK1;
                _log("State changed: ST_RUN to ST_BLOCK1");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_RUN to ST_ENDING");
                break;
            default:
                break;
            }
        }
        break;
    case ST_BLOCK1:
        if (tr_block1 != nullptr) {
            status = tr_block1->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_BLOCK4;
                _log("State changed: ST_BLOCK1 to ST_BLOCK4");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_BLOCK2;
                _log("State changed: ST_BLOCK1 to ST_BLOCK2");
                break;
            default:
                break;
            }
        }
        break;
    case ST_BLOCK2:
        if (tr_block2 != nullptr) {
            status = tr_block2->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_BLOCK4;
                _log("State changed: ST_BLOCK2 to ST_BLOCK4");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_BLOCK3;
                _log("State changed: ST_BLOCK2 to ST_BLOCK3");
                break;
            default:
                break;
            }
        }
        break;
    case ST_BLOCK3:
        if (tr_block3 != nullptr) {
            status = tr_block3->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_BLOCK4;
                _log("State changed: ST_BLOCK3 to ST_BLOCK4");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_BLOCK3 to ST_ENDING");
                break;
            default:
                break;
            }
        }
        break;
    case ST_BLOCK4:
        if (tr_block4 != nullptr) {
            status = tr_block4->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_BLOCK5;
                _log("State changed: ST_BLOCK4 to ST_BLOCK5");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_BLOCK6;
                _log("State changed: ST_BLOCK4 to ST_BLOCK6");
                break;
            default:
                break;
            }
        }
        break;
    case ST_BLOCK5:
        if (tr_block5 != nullptr) {
            status = tr_block5->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_BLOCK6;
                _log("State changed: ST_BLOCK5 to ST_BLOCK6");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_BLOCK5 to ST_ENDING");
                break;
            default:
                break;
            }
        }
        break;
    case ST_BLOCK6:
        if (tr_block6 != nullptr) {
            status = tr_block6->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_BLOCK6 to ST_ENDING");
                break;
            default:
                break;
            }
        }
        break;
    case ST_ENDING:
        _log("waking up main...");
        /* wake up the main task */
        ercd = wup_tsk(MAIN_TASK);
        assert(ercd == E_OK);
        if (ercd != E_OK) {
            syslog(LOG_NOTICE, "wup_tsk() returned %d", ercd);
        }
        state = ST_END;
        _log("State changed: ST_ENDING to ST_END");
        break;    
    case ST_INITIAL:    /* do nothing */
    case ST_END:        /* do nothing */
    default:            /* do nothing */
        break;
    }
/*
    === STATE MACHINE DEFINITION ENDS HERE ===
*/

    rightMotor->drive();
    leftMotor->drive();

    _debug({
      rgb_raw_t cur_rgb;
      colorSensor->getRawColor(cur_rgb);
      _intervalLog("R=%03d, G=%03d, B=%03d", cur_rgb.r, cur_rgb.g, cur_rgb.b);
    },2); /* if _DEBUG_LEVEL >= 2 */
    
    /* detect if the execution of update task is taking too long */
    std::chrono::system_clock::time_point te_upd_local = std::chrono::system_clock::now();
    std::uint32_t t_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(te_upd_local - ts_upd_local).count();
    if ( t_elapsed > PERIOD_UPD_TSK ) {
      _log("elapsed: %04d > PERIOD_UPD_TSK: %04d msec", t_elapsed/1000, PERIOD_UPD_TSK/1000);
      if (state != ST_INITIAL)
	_debug(assert(0),1); /* if _DEBUG_LEVEL >= 1 */
    }
}
