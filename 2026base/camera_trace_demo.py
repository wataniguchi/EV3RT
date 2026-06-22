import sys
import argparse
import time
import threading
import signal
import math
from enum import IntEnum, Enum, auto
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor
from simple_pid import PID
from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Parallel
from py_trees.common import ParallelPolicy
from py_trees import (
    display as display_tree,
    logging as log_tree
)
from py_etrobo_util import Video, TraceSide, TargetInterested, Plotter, SymmetricClamper, Color, ColorClassifier, LowPassFilter

EXEC_INTERVAL: float = 0.02
VIDEO_INTERVAL: float = 0.02
ARM_SHIFT_PWM = 35
JUNCT_UPPER_THREAH = 50
JUNCT_LOWER_THREAH = 30
SPIN_MAX_POWER = 57
SPIN_MIN_POWER = 47
TRACELINE_TARGET_V = 75

class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1

class JState(Enum):
    INITIAL = auto()
    JOINING = auto()
    JOINED = auto()
    FORKING = auto()
    FORKED = auto()

class HeadingType(Enum):
    ABSOLUTE = "absolute"
    RELATIVE = "relative"

g_plotter: Plotter = None
g_hub: Hub = None
g_arm_motor: Motor = None
g_right_motor: Motor = None
g_left_motor: Motor = None
g_touch_sensor: TouchSensor = None
g_color_sensor: ColorSensor = None
g_sonar_sensor: SonarSensor = None
g_gyro_sensor: GyroSensor = None
g_course: int = 0
g_key: str = None


class TheEnd(Behaviour):
    def __init__(self, name: str):
        super(TheEnd, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.RUNNING


class ResetDevice(Behaviour):
    def __init__(self, name: str):
        super(ResetDevice, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.count = 0

    def update(self) -> Status:
        if self.count == 0:
            g_arm_motor.reset_count()
            g_right_motor.reset_count()
            g_left_motor.reset_count()
            g_gyro_sensor.reset()
            self.logger.info("%+06d %s.resetting..." % (g_plotter.get_distance(), self.__class__.__name__))
            self.logger.info("%+06d %s.waiting for IMU to be stationary..." % (g_plotter.get_distance(), self.__class__.__name__))
        elif self.count > 3:
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        if g_hub.hub_imu_is_stationary():
            self.count += 1
        return Status.RUNNING


class ArmUpDownFull(Behaviour):
    def __init__(self, name: str, direction: ArmDirection):
        super(ArmUpDownFull, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.direction = direction
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.prev_degree = g_arm_motor.get_count()
            self.logger.info("%+06d %s.start position is %d" % (g_plotter.get_distance(), self.__class__.__name__, self.prev_degree))
            self.count = 0
            g_arm_motor.set_power(ARM_SHIFT_PWM * self.direction)
        else:
            cur_degree = g_arm_motor.get_count()
            if abs(cur_degree - self.prev_degree) < 5:
                if self.count > 20:
                    g_arm_motor.set_power(0)
                    g_arm_motor.set_brake(True)
                    self.logger.info("%+06d %s.position set to %d" % (g_plotter.get_distance(), self.__class__.__name__, cur_degree))
                    return Status.SUCCESS
                else:
                    self.count += 1
            self.prev_degree = cur_degree
        return Status.RUNNING


class ReadKey(Behaviour):
    def __init__(self, name: str):
        super(ReadKey, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            return Status.RUNNING
        else:
            global g_key
            g_key = input("Enter the given key for decryption: ")
            # check the length of the key, it should be 4 characters long
            if len(g_key) != 4:
                self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long." % (g_plotter.get_distance(), self.__class__.__name__, len(g_key)))
                return Status.RUNNING
            else:
                # display the entered key and ask for confirmation
                self.logger.info("%+06d %s.entered key: %s" % (g_plotter.get_distance(), self.__class__.__name__, g_key))
                confirmation = input("Is the entered key correct? (y/n): ")
                if confirmation.lower() == 'y':
                    self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))
                    return Status.SUCCESS
                else:
                    self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))
                    return Status.RUNNING


class IsTimePassed(Behaviour):
    def __init__(self, name: str, delta_time: int):
        super(IsTimePassed, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_time = delta_time
        self.running = False
        self.earned = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.orig_time = time.time()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_time, self.__class__.__name__, self.delta_time))
        cur_time = time.time()
        earned_time = cur_time - self.orig_time
        if earned_time >= self.delta_time:
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta time passed" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsDistanceEarned(Behaviour):
    def __init__(self, name: str, delta_dist: int):
        super(IsDistanceEarned, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_dist = delta_dist
        self.running = False
        self.earned = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.orig_dist = g_plotter.get_distance()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, self.delta_dist))
        cur_dist = g_plotter.get_distance()
        earned_dist = cur_dist - self.orig_dist
        if (earned_dist >= self.delta_dist or -earned_dist <= -self.delta_dist):
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta distance earned" % (cur_dist, self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsColorDetected(Behaviour):
    def __init__(self, name: str, color: Color):
        super(IsColorDetected, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.color = color
        self.prevColor = Color.UNKNOWN
        self.classifier = ColorClassifier()
        self.running = False
        self.detected = False

    def update(self) -> Status:
        cur_dist = g_plotter.get_distance()
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for color=%s" % (cur_dist, self.__class__.__name__, self.color.value))
        h, s, v = g_color_sensor.get_raw_color_hsv()

        detected_color = self.classifier.classify(h, s, v)
        if detected_color == self.color:
            if not self.detected:
                self.detected = True
                self.logger.info("%+06d %s.color=%s detected" % (cur_dist, self.__class__.__name__, self.color.value))
            return Status.SUCCESS
        else:
            if detected_color != self.prevColor:
                # do not log UNKNOWN color to reduce log clutter
                if detected_color != Color.UNKNOWN or self.prevColor != Color.UNKNOWN:
                    self.logger.info("%+06d %s.color changed from %s to %s" % (cur_dist, self.__class__.__name__, self.prevColor.value, detected_color.value))
                    self.prevColor = detected_color
            return Status.RUNNING


class IsQRDecoded(Behaviour):
    def __init__(self, name: str):
        super(IsQRDecoded, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False
        self.detected = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            g_video.set_target_interested(TargetInterested.QRCODE)
            self.logger.info("%+06d %s.detection started for QR code" % (g_plotter.get_distance(), self.__class__.__name__))
        text = g_video.get_QR_text()
        if text != "":
            if not self.detected:
                self.detected = True
                self.logger.info("%+06d %s.QR code decoded: %s" % (g_plotter.get_distance(), self.__class__.__name__, text))
            return Status.SUCCESS
        else:
            return Status.RUNNING

class IsSonarOn(Behaviour):
    def __init__(self, name: str, alert_dist: int):
        super(IsSonarOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.alert_dist = alert_dist
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, self.alert_dist))
        
        dist = g_sonar_sensor.get_distance()
        if (dist <= self.alert_dist and dist > 0):
            self.logger.info("%+06d %s.alerted at dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, dist))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsTouchOn(Behaviour):
    def __init__(self, name: str):
        super(IsTouchOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.waiting for touch..." % (g_plotter.get_distance(), self.__class__.__name__))
        if g_touch_sensor.is_pressed():
            self.logger.info("%+06d %s.pressed" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class StopNow(Behaviour):
    def __init__(self, name: str):
        super(StopNow, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        g_right_motor.set_power(0)
        g_right_motor.set_brake(True)
        g_left_motor.set_power(0)
        g_left_motor.set_brake(True)
        self.logger.info("%+06d %s.motors stopped" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.SUCCESS


class RunAsInstructed(Behaviour):
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        super(RunAsInstructed, self).__init__(name)
        self.pwm_l = g_course * pwm_l
        self.pwm_r = g_course * pwm_r
        self.running = False


    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))
        g_right_motor.set_power(self.pwm_r)
        g_left_motor.set_power(self.pwm_l)
        return Status.RUNNING


class TraceLine(Behaviour):
    def __init__(self, name: str, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,
                 trace_side: TraceSide,
                 # low-pass filter parameters
                 cutoff_hz: float = 12.0, median_window: int = 0,
                 # adaptive speed parameters
                 power_min: int = None,          # floor speed; None = constant speed (old behaviour)
                 err_lo: float = 6.0,            # rolling |err| at/below which we run full speed
                 err_hi: float = 22.0,           # rolling |err| at/above which we run power_min
                 accel_per_s: float = 60.0,      # how fast we may speed up   (gentle)
                 decel_per_s: float = 180.0,     # how fast we may slow down  (quick = pseudo lookahead)
                 metric_hz: float = 2.0,
                 # ---- gain scheduling: interpolate gains on current speed ----
                 # give (Kp, Kd) at the slow (curve) end and the fast (straight) end;
                 # None -> no scheduling, the fixed pid_p/pid_d above are used everywhere.
                 gains_slow: tuple = None,       # (Kp, Kd) at power_min
                 gains_fast: tuple = None,       # (Kp, Kd) at power_max
                 # ---- line-lost recovery (outer-edge curve rescue) ----
                 recover_v: int = None,           # bright-rail v that means "line lost to floor"; None = off
                 recover_after: int = 3,          # consecutive lost samples before hard recovery
                 recover_turn: int = None         # recovery steering magnitude; None = power_max
                ) -> None:
        super(TraceLine, self).__init__(name)
        # power is treated as the MAX/nominal speed. The PID output limit is
        # pinned to +/-power_max so steering authority does NOT shrink when the
        # base speed drops on a curve (that was the trap in the original code).
        self.power_max = power
        self.power_min = power if power_min is None else power_min
        self.power = power
        self.adapt = power_min is not None
        self.target = target
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-self.power_max, self.power_max))
        self.trace_side = trace_side
        self.lpf = (LowPassFilter(cutoff_hz, EXEC_INTERVAL, median_window) if cutoff_hz else None) # when cutoff_hz = None, no low-pass filter is applied and the raw PID output is used
        # instability/curviness estimate = smoothed |tracking error|
        self.err_lo, self.err_hi = err_lo, err_hi
        self.metric_lpf = LowPassFilter(metric_hz, EXEC_INTERVAL)
        self.err_metric = 0.0
        # per-step power slew limits (asymmetric: brake fast, accelerate slow)
        self.accel_step = accel_per_s * EXEC_INTERVAL
        self.decel_step = decel_per_s * EXEC_INTERVAL
        # gain schedule: linearly interpolate (Kp, Kd) between the slow and fast
        # anchors as a function of self.power. Ki is left fixed at pid_i.
        self.gains_slow = gains_slow
        self.gains_fast = gains_fast
        self.schedule = (gains_slow is not None and gains_fast is not None
                         and self.power_max > self.power_min)
        # line-lost recovery
        self.recover_v = recover_v
        self.recover_after = recover_after
        self.recover_turn = recover_turn
        self._lost_count = 0

        self.running = False

    def update(self) -> Status:
        if not self.running:
            if self.lpf:
                self.lpf.reset()
            self.metric_lpf.reset()
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))

        h, s, v_raw = g_color_sensor.get_raw_color_hsv()
        v = self.lpf(v_raw) if self.lpf else v_raw

        # ---- adaptive base speed -------------------------------------------
        # Use the TRUE tracking error (target - raw v) as the instability metric,
        # smoothed so the speed reacts to course shape, not to every wobble.
        self.err_metric = self.metric_lpf(abs(self.target - v_raw))
        if self.adapt:
            # map smoothed |error| in [err_lo, err_hi] -> power in [max, min]
            frac = (self.err_metric - self.err_lo) / (self.err_hi - self.err_lo)
            frac = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)
            target_power = self.power_max - frac * (self.power_max - self.power_min)
            # rate-limit the change (slow down quickly, speed up gently)
            dp = target_power - self.power
            if dp > self.accel_step:
                dp = self.accel_step
            elif dp < -self.decel_step:
                dp = -self.decel_step
            self.power += dp
        # ---- gain scheduling: gains track the current speed -----------------
        kp_now, kd_now = self.pid.Kp, self.pid.Kd
        if self.schedule:
            f = (self.power - self.power_min) / (self.power_max - self.power_min)
            f = 0.0 if f < 0.0 else (1.0 if f > 1.0 else f)
            kp_now = self.gains_slow[0] + f * (self.gains_fast[0] - self.gains_slow[0])
            kd_now = self.gains_slow[1] + f * (self.gains_fast[1] - self.gains_slow[1])
            self.pid.tunings = (kp_now, self.pid.Ki, kd_now)
        # ---- steering (PID already clamped to +/-power_max) ----------------
        if self.trace_side == TraceSide.NORMAL:
            turn = (-1) * g_course * int(self.pid(v))
        else: # TraceSide.OPPOSITE
            turn = g_course * int(self.pid(v))

        # ---- line-lost recovery --------------------------------------------
        # When the sensor pins at the bright rail, target=75 only yields a weak
        # clamped-P turn (Kp*(75-100) ~= -16), too gentle to curl back to a line
        # that curved away on an OUTER edge -> the robot drives off. Detect a
        # SUSTAINED bright-rail pin (not a 1-2 sample weave touch) and steer at
        # full authority in the direction P already (correctly) chose, until the
        # edge is reacquired. The dark rail already recovers on its own.
        if self.recover_v is not None:
            if v_raw >= self.recover_v:
                self._lost_count += 1
            else:
                self._lost_count = 0
            if self._lost_count >= self.recover_after and turn != 0:
                mag = self.power_max if self.recover_turn is None else self.recover_turn
                turn = int(math.copysign(mag, turn))

        # On a sharp slow curve, |turn| may exceed the reduced base speed, so the
        # inner wheel can go to zero/negative -> a tight pivot. That's desired.
        p = int(round(self.power))
        left  = max(-100, min(100, p + turn))    # motors cap at +-100
        right = max(-100, min(100, p - turn))
        g_right_motor.set_power(right)
        g_left_motor.set_power(left)

        # log raw v, filtered vf, error-metric, commanded power, gains, and turn
        #self.logger.info("%+06d %s.color sensor HSV=(%d, %d, %d) vf=%d, em=%d, pwr=%d, kp=%.3f, kd=%.3f, turn=%d" % (
        #    g_plotter.get_distance(), self.__class__.__name__,
        #    h, s, v_raw, int(v), int(self.err_metric), p, kp_now, kd_now, turn))

        return Status.RUNNING


class SpinAndLocateLine(Behaviour):
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, trace_side: TraceSide) -> None:
        super(SpinAndLocateLine, self).__init__(name)
        self.target = target
        self.spin_direction = 1 if trace_side == TraceSide.NORMAL else -1
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power)
        self.move_away = True
        self.running = False

    def update(self) -> Status:
        # first to spin to move away from the line, then to locate the line by spinning in the opposite direction
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            # spin for 30 degrees to move away from the line
            self.target_heading = current_heading + self.spin_direction * 30
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        if self.move_away:
            # spin in the normal direction to move away from the line
            error = float(self.target_heading) - current_heading
            # normalize error to [-180, 180]
            if error > 180.0:
                error -= 360.0
            if error < -180.0:
                error += 360.0
            if abs(error) < 2.0:
                self.logger.info("%+06d %s.move away spin ended at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))
                self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target, sample_time=EXEC_INTERVAL)
                self.spin_direction *= -1
                self.move_away = False
                return Status.RUNNING
            power = int(self.clamper.clamp(self.pid(current_heading)))
        else:             # spin in the opposite direction to locate the line
            h, s, v = g_color_sensor.get_raw_color_hsv()
            error = float(self.target) - v
            if abs(error) < 5.0:
                self.logger.info("%+06d %s.line located at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))        
                return Status.SUCCESS
            power = int(self.clamper.clamp(self.pid(v))) * self.spin_direction * (-1)
        g_right_motor.set_power(g_course * power)
        g_left_motor.set_power((-1) * g_course * power)
        return Status.RUNNING    


class SpinAround(Behaviour):
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(SpinAround, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        error = float(self.target_heading) - current_heading
        # normalize error to [-180, 180]
        if error > 180.0:
            error -= 360.0
        if error < -180.0:
            error += 360.0
        if abs(error) < 2.0:
            self.logger.info("%+06d %s.spin ended at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))
            return Status.SUCCESS
        power = int(self.clamper.clamp(self.pid(current_heading)))
        g_right_motor.set_power(g_course * power)
        g_left_motor.set_power((-1) * g_course * power)
        return Status.RUNNING    


class RunByGyro(Behaviour):
    def __init__(self, name: str, target: int, power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(RunByGyro, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.last_log_time = None
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        # log every 1 second
        if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            self.logger.info("%+06d %s.current heading=%d" % (g_plotter.get_distance(), self.__class__.__name__, current_heading))
            self.last_log_time = time.time()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
            self.logger.info("%+06d %s.gyro run started toward heading=%d" % (g_plotter.get_distance(),
                                                                              self.__class__.__name__, self.target_heading))
            self.running = True
        turn = int(self.pid(current_heading))
        g_right_motor.set_power(self.power + g_course * turn)
        g_left_motor.set_power(self.power - g_course * turn)
        return Status.RUNNING


class TraceLineCam(Behaviour):
    def __init__(self, name: str, power: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide) -> None:
        super(TraceLineCam, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.gs_min = gs_min
        self.gs_max = gs_max
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            g_video.set_thresholds(self.gs_min, self.gs_max)
            g_video.set_target_interested(TargetInterested.LINE)
            if self.trace_side == TraceSide.NORMAL:
                if g_course == -1: # right course
                    g_video.set_trace_side(TraceSide.RIGHT)
                else:
                    g_video.set_trace_side(TraceSide.LEFT)
            elif self.trace_side == TraceSide.OPPOSITE: 
                if g_course == -1: # right course
                    g_video.set_trace_side(TraceSide.LEFT)
                else:
                    g_video.set_trace_side(TraceSide.RIGHT)
            else: # TraceSide.CENTER
                g_video.set_trace_side(TraceSide.CENTER)
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))
        theta, fid, cap_t = g_video.get_theta_stamped()
        turn = (-1) * int(self.pid(theta))
        p, i, d = self.pid.components          # simple-pid P/I/D split
        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
        self.logger.info(
            "%+06d CAM fid=%06d theta=%+06.1f P=%+.1f I=%+.1f D=%+.1f turn=%+d L=%d R=%d roe=%03d insight=%d age=%.1f" % (
                g_plotter.get_distance(), fid, theta, p, i, d, turn,
                self.power + turn, self.power - turn,
                g_video.get_range_of_edges(), int(g_video.is_target_insight()),
                (time.time() - cap_t) * 1000))
        return Status.RUNNING


class IsJunction(Behaviour):
    def __init__(self, name: str, target_state: JState) -> None:
        super(IsJunction, self).__init__(name)
        self.target_state = target_state
        self.reached = False
        self.prev_roe = 0
        self.state:JState = JState.INITIAL
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.scan started" % (g_plotter.get_distance(), self.__class__.__name__))
        roe = g_video.get_range_of_edges()
        if roe != 0:
            if self.state == JState.INITIAL:
                if (self.target_state == JState.JOINING or self.target_state == JState.JOINED) and roe >= JUNCT_UPPER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.lines are joining" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.JOINING
                elif (self.target_state == JState.FORKING or self.target_state == JState.FORKED) and roe >= JUNCT_LOWER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.lines are forking" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.FORKING
            elif self.state == JState.JOINING:
                if roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.the join completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.JOINED
                    
            elif self.state == JState.FORKING:
                if roe <= JUNCT_LOWER_THRESH and self.prev_roe >= JUNCT_UPPER_THRESH:
                    self.logger.info("%+06d %s.the fork completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.FORKED
            else:
                pass
        self.prev_roe = roe

        if not self.reached and self.state == self.target_state:
            self.reached = True
            self.logger.info("%+06d %s.target state reached" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class TraverseBehaviourTree(object):
    def __init__(self, tree: BehaviourTree) -> None:
        self.tree = tree
        self.last_log_time = None
        self.running = False
    def __call__(
        self,
        hub: Hub,
        arm_motor: Motor,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
        sonar_sensor: SonarSensor,
        gyro_sensor: GyroSensor,
    ) -> None:
        global g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor, g_gyro_sensor, g_plotter
        if not self.running:
            g_hub = hub
            g_arm_motor = arm_motor
            g_right_motor = right_motor
            g_left_motor = left_motor
            g_touch_sensor = touch_sensor
            g_color_sensor = color_sensor
            g_sonar_sensor = sonar_sensor
            g_gyro_sensor = gyro_sensor
            g_plotter = Plotter()
            print(" -- TraverseBehaviorTree initialization complete")
            self.running = True
        else:
            self.tree.tick_once()
            g_plotter.plot(hub, arm_motor, right_motor, left_motor, touch_sensor, color_sensor, sonar_sensor, gyro_sensor)
            # log estimated position every 1 second
            #if self.last_log_time == None or time.time() - self.last_log_time >= 1.0:
            #    print(" --  estimated position X=%d, Y=%d" % (g_plotter.get_loc_x(), g_plotter.get_loc_y()))
            #    self.last_log_time = time.time()


class VideoThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()
        self.prev_time = time.time()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            g_video.process(g_plotter, g_hub, g_arm_motor, g_right_motor, g_left_motor, g_color_sensor, g_sonar_sensor, g_gyro_sensor)
            current_time = time.time()
            elapsed_time = current_time - self.prev_time
            self.prev_time = current_time
            if elapsed_time < VIDEO_INTERVAL:
                time.sleep(VIDEO_INTERVAL - elapsed_time)


def build_behaviour_tree() -> BehaviourTree:
    root = Sequence(name="2026 base", memory=True)
    calibration = Sequence(name="calibration", memory=True)
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne())
    loop = Parallel(name="loop", policy=ParallelPolicy.SuccessOnOne())
    calibration.add_children(
        [
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
            IsTimePassed(name="wait for a moment", delta_time=0.5),
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            ResetDevice(name="device reset"),
            #ReadKey(name="read key"),
        ]
    )
    start.add_children(
        [
            IsTouchOn(name="touch start"),
        ]
    )
    loop.add_children(
        [
            TraceLineCam(name="camera trace normal edge", power=45,
                pid_p=2.0, pid_i=0.0012, pid_d=0.18,
                gs_min=0, gs_max=60, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist=3500),
        ]
    )
    root.add_children(
        [
            calibration,
            start,
            loop,
            StopNow(name="stop"),
            TheEnd(name="end"),
        ]
    )
    return root

def initialize_etrobo(backend: str) -> ETRobo:
    return (ETRobo(backend=backend)
            .add_hub('hub')
            .add_device('arm_motor', device_type=Motor, port='C')
            .add_device('right_motor', device_type=Motor, port='A')
            .add_device('left_motor', device_type=Motor, port='B')
            .add_device('touch_sensor', device_type=TouchSensor, port='D')
            .add_device('color_sensor', device_type=ColorSensor, port='E')
            .add_device('sonar_sensor', device_type=SonarSensor, port='F')
            .add_device('gyro_sensor', device_type=GyroSensor, port='')
    )

def setup_thread():
    global g_video, g_video_thread
    g_video = Video()

    print(" -- starting VideoThread...")
    g_video_thread = VideoThread()
    g_video_thread.start()

def cleanup_thread():
    global g_video, g_video_thread
    print(" -- stopping VideoThread...")
    g_video_thread.stop()
    g_video_thread.join()

    del g_video

def sig_handler(signum, frame) -> None:
    sys.exit(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('course', choices=['right', 'left'], help='Course to run')
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()

    if args.course == 'right':
        g_course = -1
    else:
        g_course = 1

    setup_thread()

    #log_tree.level = log_tree.Level.DEBUG
    tree = build_behaviour_tree()
    #display_tree.render_dot_tree(tree)

    signal.signal(signal.SIGTERM, sig_handler)

    try:
        etrobo = initialize_etrobo(backend='raspike_art')
        etrobo.add_handler(TraverseBehaviourTree(tree))
        etrobo.dispatch(interval=EXEC_INTERVAL, logfile=args.logfile)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cleanup_thread()
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- exiting...")
