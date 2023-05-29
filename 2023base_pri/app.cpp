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
FilteredColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
SRLF*           srlfL;
FilteredMotor*  leftMotor;
SRLF*           srlfR;
FilteredMotor*  rightMotor;
Motor*          armMotor;
Plotter*        plotter;
Video*          video;

BrainTree::BehaviorTree* tr_calibration = nullptr;
BrainTree::BehaviorTree* tr_run         = nullptr;
BrainTree::BehaviorTree* tr_block       = nullptr;
State state = ST_INITIAL;

int upd_process_count = 0;
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

/*
    === NODE CLASS DEFINITION STARTS HERE ===
    A Node class serves like a LEGO block while a Behavior Tree serves as a blueprint for the LEGO object built using the LEGO blocks.
*/

/*
    usage:
    ".leaf<ResetClock>()"
    is to reset the clock and indicate the robot departure by LED color.
*/
class ResetClock : public BrainTree::Node {
public:
    Status update() override {
        ev3clock->reset();
        _log("clock reset.");
        ev3_led_set_color(LED_GREEN);
        return Status::Success;
    }
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
    ".leaf<IsBackOn>()"
    is to check if the back button gets pressed.
*/
class IsBackOn : public BrainTree::Node {
public:
    Status update() override {
        if (ev3_button_is_pressed(BACK_BUTTON)) {
            _log("back button pressed.");
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
    ".leaf<IsAngleLarger>(angle)"
    is to determine if the angular location of the robot measured by the gyro sensor is larger than the spedified angular value.
    angle is in degree.
*/
class IsAngleLarger : public BrainTree::Node {
public:
    IsAngleLarger(int ang) : angle(ang) {}
    Status update() override {
        int32_t curAngle = gyroSensor->getAngle(); 
        if (curAngle >= angle){
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    int32_t angle;
};

/*
    usage:
    ".leaf<IsAngleSmaller>(angle)"
    is to determine if the angular location of the robot measured by the gyro sensor is smaller than the spedified angular value.
    angle is in degree.
*/
class IsAngleSmaller : public BrainTree::Node {
public:
    IsAngleSmaller(int ang) : angle(ang) {}
    Status update() override {
        int32_t curAngle = gyroSensor->getAngle(); 
        if (curAngle <= angle){
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    int32_t angle;
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
        if (!updated) {
            _log("ODO=%05d, Color detection started.", plotter->getDistance());
            updated = true;
        }
        rgb_raw_t cur_rgb;
        colorSensor->getRawColor(cur_rgb);

        switch(color){
            case CL_JETBLACK:
                if (cur_rgb.r <= 8 && cur_rgb.g <= 8 && cur_rgb.b <= 8) { 
                    _log("ODO=%05d, CL_JETBLACK detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
	    case CL_BLACK:
                if (cur_rgb.r <= 50 && cur_rgb.g <= 45 && cur_rgb.b <= 60) {
                    _log("ODO=%05d, CL_BLACK detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_BLUE:
               if (cur_rgb.b - cur_rgb.r > 35 && cur_rgb.g >= 55 && cur_rgb.b <= 100 && cur_rgb.b >= 70) {
                    _log("ODO=%05d, CL_BLUE detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_BLUE2:
                 if (cur_rgb.r <= 25 && cur_rgb.g <= 55 && cur_rgb.b >= 55) {
                     _log("ODO=%05d, CL_BLUE2 detected.", plotter->getDistance());
                     return Status::Success;
                 }
                 break;
            case CL_RED:
                if (cur_rgb.r - cur_rgb.b >= 25 && cur_rgb.r > 85 && cur_rgb.g < 60) {
                    _log("ODO=%05d, CL_RED detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_YELLOW:
                 if (cur_rgb.r >= 110 &&  cur_rgb.g >= 90 && cur_rgb.b >= 50 && cur_rgb.b <= 120 ) {
                     _log("ODO=%05d, CL_YELLOW detected.", plotter->getDistance());
                     return Status::Success;
                 }
                 break;
            case CL_GREEN:
                 if (cur_rgb.b - cur_rgb.r < 30 && cur_rgb.g >= 30 && cur_rgb.b <= 80) {
                     _log("ODO=%05d, CL_GREEN detected.", plotter->getDistance());
                     return Status::Success;
                 }   
                 break;
	    case CL_GRAY:
                if (cur_rgb.r >= 45 && cur_rgb.g <=60 && cur_rgb.b <= 65 && cur_rgb.r <= 52 && cur_rgb.b >= 53) {
                    _log("ODO=%05d, CL_GRAY detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_WHITE:
	        if (cur_rgb.r >= 100 && cur_rgb.g >= 100 && cur_rgb.b >= 100 ) {
                    _log("ODO=%05d, CL_WHITE detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            default:
                break;
        }
        return Status::Running;
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
	//_log("ODO=%05d, roe = %d", plotter->getDistance(), roe);

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
    ".leaf<TraceLineCam>(speed, p, i, d, gs_min, gs_max, srew_rate, trace_side)"
    is to instruct the robot to trace the line in backward at the given speed.
    p, i, d are constants for PID control.
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
  TraceLineCam(int s, double p, double i, double d, int gs_min, int gs_max, double srew_rate, TraceSide trace_side) : speed(s),gsMin(gs_min),gsMax(gs_max),srewRate(srew_rate),side(trace_side) {
        updated = false;
        ltPid = new PIDcalculator(p, i, d, PERIOD_UPD_TSK, -speed, speed);
    }
    ~TraceLineCam() {
        delete ltPid;
    }
    Status update() override {
        if (!updated) {
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
            _log("ODO=%05d, Camera Trace run started.", plotter->getDistance());
            updated = true;
        }

        int8_t backward, turn, pwmL, pwmR;
	int theta = video->getTheta();
	//_log("ODO=%05d, theta = %d", plotter->getDistance(), theta);
	
        /* compute necessary amount of steering by PID control */
        turn = (-1) * ltPid->compute(theta, 0); /* 0 is the center */
	//_log("ODO=%05d, turn = %d", plotter->getDistance(), turn);
        backward = -speed;
        /* steer EV3 by setting different speed to the motors */
        pwmL = backward - turn;
        pwmR = backward + turn;
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
};


/*
    usage:
    ".leaf<TraceLine>(speed, target, p, i, d, srew_rate, trace_side)"
    is to instruct the robot to trace the line at the given speed.
    p, i, d are constants for PID control.
    target is the brightness level for the ideal line for trace.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
    trace_side = TS_NORMAL   when in R(L) course and tracing the right(left) side of the line.
    trace_side = TS_OPPOSITE when in R(L) course and tracing the left(right) side of the line.
*/
class TraceLine : public BrainTree::Node {
public:
    TraceLine(int s, int t, double p, double i, double d, double srew_rate, TraceSide trace_side) : speed(s),target(t),srewRate(srew_rate),side(trace_side) {
        updated = false;
        ltPid = new PIDcalculator(p, i, d, PERIOD_UPD_TSK, -speed, speed);
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
    ".leaf<RotateEV3>(30, speed, srew_rate)"
    is to rotate robot 30 degrees (=clockwise) at the specified speed.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
*/
class RotateEV3 : public BrainTree::Node {
public:
    RotateEV3(int16_t degree, int s, double srew_rate) : deltaDegreeTarget(degree),speed(s),srewRate(srew_rate) {
        updated = false;
        assert(degree >= -180 && degree <= 180);
        if (degree > 0) {
            clockwise = 1;
        } else {
            clockwise = -1;
        }
    }
    Status update() override {
        if (!updated) {
            originalDegree = plotter->getDegree();
            srlfL->setRate(srewRate);
            srlfR->setRate(srewRate);
            /* stop the robot at start */
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            _log("ODO=%05d, Rotation started. Current degree = %d", plotter->getDistance(), originalDegree);
            updated = true;
            return Status::Running;
        }

        int16_t deltaDegree = plotter->getDegree() - originalDegree;
        if (deltaDegree > 180) {
            deltaDegree -= 360;
        } else if (deltaDegree < -180) {
            deltaDegree += 360;
        }
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
            _log("ODO=%05d, Rotation ended. Current degree = %d", plotter->getDistance(), plotter->getDegree());
            return Status::Success;
        }
    }
private:
    int16_t deltaDegreeTarget, originalDegree;
    int clockwise, speed;
    bool updated;
    double srewRate;
};

/*
    usage:
    ".leaf<SetArmPosition>(target_degree, pwm)"
    is to shift the robot arm to the specified degree by the spefied power.
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
            armMotor->setPWM(0);
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
    === NODE CLASS DEFINITION ENDS HERE ===
*/

/* classes running as sub-threads */
class vcap_thd {
public:
  void operator()(int unused) {
    Mat f;
    while (state != ST_END && state != ST_ENDING) {
      f = video->readFrame();
      std::chrono::system_clock::time_point te_cap_local = std::chrono::system_clock::now();
      /* critical section 1 */
      if ( mut1.try_lock() ) {
#if defined(WITH_OPENCV)
	frame_in = f.clone();
#endif
	te_cap = te_cap_local;
	mut1.unlock();
	vcap_thd_count++;
      }
      std::this_thread::yield();
    }
    _logNoAsp("sub-thread ready to join. # of execution = %d", vcap_thd_count);
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
#if defined(WITH_OPENCV)
	f = frame_in.clone();
#endif
	te_cap_local = te_cap;
	mut1.unlock();
	f = video->calculateTarget(f);
	std::chrono::system_clock::time_point te_cal_local = std::chrono::system_clock::now();
	/* critical section 2 */
	if ( mut2.try_lock() ) {
#if defined(WITH_OPENCV)
	  frame_out = f.clone();
#endif
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
#if defined(WITH_OPENCV)
	  f = frame_out.clone();
#endif
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
    // temp fix 2022/6/20 W.Taniguchi, as Bluetooth not implemented yet
    //bt = ev3_serial_open_file(EV3_SERIAL_BT);
    //assert(bt != NULL);
    /* create and initialize EV3 objects */
    ev3clock    = new Clock();
    _log("initialization started.");
    video       = new Video();
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_3);
    colorSensor = new FilteredColorSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new FilteredMotor(PORT_C);
    rightMotor  = new FilteredMotor(PORT_B);
    armMotor    = new Motor(PORT_A);
    plotter     = new Plotter(leftMotor, rightMotor, gyroSensor);
    /* read profile files and make the profile object ready */
    prof        = new Profile("2023base_pri/*profile.txt");
    /* determine the course L or R */
    if (prof->getValueAsStr("COURSE") == "R") {
      _COURSE = -1;
    } else {
      _COURSE = 1;
    }
 
    /* FIR parameters for a low-pass filter with normalized cut-off frequency of 0.2
        using a function of the Hamming Window */
    const int FIR_ORDER = 4; 
    const double hn[FIR_ORDER+1] = { 7.483914270309116e-03, 1.634745733863819e-01, 4.000000000000000e-01, 1.634745733863819e-01, 7.483914270309116e-03 };
    /* set filters to FilteredColorSensor */
    Filter *lpf_r = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_g = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_b = new FIR_Transposed(hn, FIR_ORDER);
    colorSensor->setRawColorFilters(lpf_r, lpf_g, lpf_b);

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
      tr_block = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK_R .build();
    } else {
      tr_run   = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_RUN_L   .build();
      tr_block = (BrainTree::BehaviorTree*) BrainTree::Builder() TR_BLOCK_L .build();
    }
/*
    === BEHAVIOR TREE DEFINITION ENDS HERE ===
*/

    /* start sub-threads not managed by ASP */
    _log("starting sub-threads...");
    sigset_t ss;  
    sigemptyset(&ss);
    sigaddset(&ss, SIGUSR2);
    sigaddset(&ss, SIGALRM);
    sigaddset(&ss, SIGPOLL);
    sigaddset(&ss, SIGIO);

    std::vector<std::thread> thds;
    int iUnused = 0;
    pthread_sigmask(SIG_BLOCK, &ss, 0);
    thds.emplace_back(vcap_thd(), iUnused);
    thds.emplace_back(vshow_thd(), iUnused);
    thds.emplace_back(vcal_thd(), iUnused);
    pthread_sigmask(SIG_UNBLOCK, &ss, 0); /* let ASP manage the main thread */
    
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
    delete tr_block;
    delete tr_run;
    delete tr_calibration;
    /* destroy profile object */
    delete prof;
    /* destroy EV3 objects */
    delete lpf_b;
    delete lpf_g;
    delete lpf_r;
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
    // temp fix 2022/6/20 W.Taniguchi, as Bluetooth not implemented yet
    //fclose(bt);
#if defined(MAKE_SIM)    
    ETRoboc_notifyCompletedToSimulator();
#endif
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

    colorSensor->sense();
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
                state = ST_BLOCK;
                _log("State changed: ST_RUN to ST_BLOCK");
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
    case ST_BLOCK:
        if (tr_block != nullptr) {
            status = tr_block->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_BLOCK to ST_ENDING");
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
    //logger->outputLog(LOG_INTERVAL);

    /* ensure that the execution of update task is taking too long */
    std::chrono::system_clock::time_point te_upd_local = std::chrono::system_clock::now();
    std::uint32_t t_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(te_upd_local - ts_upd_local).count();
    if ( t_elapsed > PERIOD_UPD_TSK ) {
      _log("elapsed: %04d > PERIOD_UPD_TSK: %04d msec", t_elapsed/1000, PERIOD_UPD_TSK/1000);
      if (state != ST_INITIAL)
	_debug(assert(0)); /* if DEBUG is defined in appusr.hpp,
			      execution gets stopped when update task is too long */
    }
}
