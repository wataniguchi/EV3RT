import argparse
import time
import math
import threading
import signal
from enum import Enum, IntEnum, auto
from etrobo_python import ETRobo, Hub, Motor, ColorSensor, TouchSensor, SonarSensor, GyroSensor
from simple_pid import PID
import py_trees.common
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
from py_etrobo_util import Video, TraceSide, Plotter

EXEC_INTERVAL: float = 0.04
VIDEO_INTERVAL: float = 0.02
ARM_SHIFT_PWM = 30
JUNCT_UPPER_THRESH = 50
JUNCT_LOWER_THRESH = 30

class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1

class JState(Enum):
    INITIAL = auto()
    JOINING = auto()
    JOINED = auto()
    FORKING = auto()
    FORKED = auto()

class Color(Enum):
    JETBLACK = auto()
    BLACK = auto()
    BLUE = auto()
    RED = auto()
    YELLOW = auto()
    GREEN = auto()
    WHITE = auto()

g_plotter: Plotter = None
g_hub: Hub = None
g_arm_motor: Motor = None
g_right_motor: Motor = None
g_left_motor: Motor = None
g_touch_sensor: TouchSensor = None
g_color_sensor: ColorSensor = None
g_sonar_sensor: SonarSensor = None
g_gyro_sensor: GyroSensor = None
g_video: Video = None
g_video_thread: threading.Thread = None
g_course: int = 0
g_dist: int = 1500
g_earned_dist: int = 0
g_distFlg: bool = False

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
        elif self.count > 3:
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
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
            self.count = 0
        else:
            cur_degree = g_arm_motor.get_count()
            if cur_degree == self.prev_degree:
                if self.count > 10:
                    g_arm_motor.set_power(0)
                    g_arm_motor.set_brake(True)
                    self.logger.info("%+06d %s.position set to %d" % (g_plotter.get_distance(), self.__class__.__name__, cur_degree))
                    return Status.SUCCESS
                else:
                    self.count += 1
            self.prev_degree = cur_degree
        g_arm_motor.set_power(ARM_SHIFT_PWM * self.direction)
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
        
        dist = 10 * g_sonar_sensor.get_distance()
        if (dist <= self.alert_dist and dist > 0):
            self.logger.info("%+06d %s.alerted at dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, dist))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsTouchOn(Behaviour):
    def __init__(self, name: str):
        super(IsTouchOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        if (g_touch_sensor.is_pressed() or
            g_hub.is_left_button_pressed() or
            g_hub.is_right_button_pressed()):
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


class RunAsInstructed(Behaviour):
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        super(RunAsInstucted, self).__init__(name)
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
                 trace_side: TraceSide) -> None:
        super(TraceLine, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))
        if self.trace_side == TraceSide.NORMAL:
            turn = (-1) * g_course * int(self.pid(g_color_sensor.get_brightness()))
        else: # TraceSide.OPPOSITE
            turn = g_course * int(self.pid(g_color_sensor.get_brightness()))
        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
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
        turn = (-1) * int(self.pid(g_video.get_theta()))
        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
        return Status.RUNNING

class TraceLineCamLR(Behaviour):
    def __init__(self, name: str, powerR: int, powerL: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide) -> None:
        super(TraceLineCam, self).__init__(name)
        self.powerR = powerR
        self.powerL = powerL
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-powerL, powerL))
        self.gs_min = gs_min
        self.gs_max = gs_max
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            g_video.set_thresholds(self.gs_min, self.gs_max)
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
        turn = (-1) * int(self.pid(g_video.get_theta()))
        g_right_motor.set_power(self.powerR - turn)
        g_left_motor.set_power(self.powerL + turn)
        return Status.RUNNING
    
class MoveStraight(Behaviour):
    def __init__(self, name: str, power: int, target_distance: int) -> None:
        super(MoveStraight, self).__init__(name)
        self.power = power
        self.target_distance = target_distance
        self.start_distance = None
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.start_distance = g_plotter.get_distance()
            g_right_motor.set_power(self.power)
            g_left_motor.set_power(self.power)
            self.logger.info("%+06d %s.開始、パワー=%d、目標距離=%d" % 
                            (self.start_distance, self.__class__.__name__, self.power, self.target_distance))
        
        current_distance = g_plotter.get_distance()
        traveled_distance = current_distance - self.start_distance
        
        if traveled_distance >= self.target_distance:
            g_right_motor.set_power(0)
            g_left_motor.set_power(0)
            self.logger.info("%+06d %s.目標距離に到達" % (current_distance, self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING

class MoveStraight_dbr(Behaviour):
    def __init__(self, name: str, power: int, target_distance: int) -> None:
        super(MoveStraight_dbr, self).__init__(name)
        self.power = power
        self.target_distance = target_distance
        self.start_distance = None
        self.running = False

    def update(self) -> Status:
        global g_distFlg
        if g_distFlg:
            return Status.SUCCESS

        if not self.running:
            self.running = True
            self.start_distance = g_plotter.get_distance()
            g_right_motor.set_power(self.power)
            g_left_motor.set_power(self.power)
            self.logger.info("%+06d %s.開始、パワー=%d、目標距離=%d" % 
                            (self.start_distance, self.__class__.__name__, self.power, self.target_distance))
        
        current_distance = g_plotter.get_distance()
        traveled_distance = current_distance - self.start_distance
        
        if traveled_distance >= self.target_distance:
            g_right_motor.set_power(0)
            g_left_motor.set_power(0)
            self.logger.info("%+06d %s.目標距離に到達" % (current_distance, self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING

class MoveStraightLR(Behaviour):
    def __init__(self, name: str, right_power: int, left_power: int, target_distance: int) -> None:
        super(MoveStraightLR, self).__init__(name)
        self.right_power = right_power
        self.left_power = left_power
        self.target_distance = target_distance
        self.start_distance = None
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.start_distance = g_plotter.get_distance()
            g_right_motor.set_power(self.right_power)
            g_left_motor.set_power(self.left_power)
            self.logger.info("%+06d %s.開始、右パワー=%d、左パワー=%d、目標距離=%d" % 
                             (self.start_distance, self.__class__.__name__, self.right_power, self.left_power, self.target_distance))
        
        current_distance = g_plotter.get_distance()
        traveled_distance = current_distance - self.start_distance
        
        if traveled_distance >= self.target_distance:
            g_right_motor.set_power(0)
            g_left_motor.set_power(0)
            self.logger.info("%+06d %s.目標距離に到達" % (current_distance, self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING

class MoveStraightLR_dbr(Behaviour):
    def __init__(self, name: str, right_power: int, left_power: int, target_distance: int) -> None:
        super(MoveStraightLR_dbr, self).__init__(name)
        self.right_power = right_power
        self.left_power = left_power
        self.target_distance = target_distance
        self.start_distance = None
        self.running = False

    def update(self) -> Status:
        global g_distFlg
        if g_distFlg:
            return Status.SUCCESS
        
        if not self.running:
            self.running = True
            self.start_distance = g_plotter.get_distance()
            g_right_motor.set_power(self.right_power)
            g_left_motor.set_power(self.left_power)
            self.logger.info("%+06d %s.開始、右パワー=%d、左パワー=%d、目標距離=%d" % 
                             (self.start_distance, self.__class__.__name__, self.right_power, self.left_power, self.target_distance))
        
        current_distance = g_plotter.get_distance()
        traveled_distance = current_distance - self.start_distance
        
        if traveled_distance >= self.target_distance:
            g_right_motor.set_power(0)
            g_left_motor.set_power(0)
            self.logger.info("%+06d %s.目標距離に到達" % (current_distance, self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING

class TraverseBehaviourTree(object):
    def __init__(self, tree: BehaviourTree) -> None:
        self.tree = tree
        self.running = False
    def __call__(
        self,
        **kwargs,
    ) -> None:
        global g_plotter
        if not self.running:
            if g_hub is None:
                print(" -- TraverseBehaviorTree waiting for ETrobo devices to be exposed...")
            else:
                self.running = True
                g_plotter = Plotter()
                print(" -- TraverseBehaviorTree initialization complete")
        else:
            self.tree.tick_once()
            g_plotter.plot(**kwargs)


class ExposeDevices(object):
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
        global g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor, g_gyro_sensor
        g_hub = hub
        g_arm_motor = arm_motor
        g_right_motor = right_motor
        g_left_motor = left_motor
        g_touch_sensor = touch_sensor
        g_color_sensor = color_sensor
        g_sonar_sensor = sonar_sensor
        g_gyro_sensor = gyro_sensor


class VideoThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            g_video.process(g_plotter, g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor, g_gyro_sensor)
            time.sleep(VIDEO_INTERVAL)

class IsRedColorDetected(Behaviour):
    def __init__(self, name: str, threshold: float):
        super(IsRedColorDetected, self).__init__(name)
        self.threshold = threshold
        self.running = False

    def update(self) -> Status:
        global g_dist
        global g_earned_dist
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.checking red color ratio with threshold=%f" % (g_plotter.get_distance(), self.__class__.__name__, self.threshold))

        red_percentage = g_video.get_red_ratio() * 100
        if red_percentage > self.threshold:
            self.logger.info("%+06d %s.red color ratio exceeds threshold: %f" % (g_plotter.get_distance(), self.__class__.__name__, red_percentage))
            g_dist = g_dist - g_earned_dist
            self.logger.info("グローバル変数更新 g_dist = g_dist - g_earned_dist")
            # print("g_earned_dist:"+ str(g_earned_dist))
            # print("g_dist:"+ str(g_dist))
            self.logger.info("赤判定")
            return Status.SUCCESS
        else:
            return Status.RUNNING
        
class IsBlueColorDetected(Behaviour):
    def __init__(self, name: str, threshold: float):
        super(IsBlueColorDetected, self).__init__(name)
        self.threshold = threshold
        self.running = False

    def update(self) -> Status:
        global g_dist
        global g_earned_dist
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.checking blue color ratio with threshold=%f" % (g_plotter.get_distance(), self.__class__.__name__, self.threshold))

        blue_percentage = g_video.get_blue_ratio() * 100
        if blue_percentage > self.threshold:
            self.logger.info("%+06d %s.blue color ratio exceeds threshold: %f" % (g_plotter.get_distance(), self.__class__.__name__, blue_percentage))
            g_dist = g_dist - g_earned_dist
            self.logger.info("グローバル変数更新 g_dist = g_dist - g_earned_dist")
            # print("g_earned_dist:"+ str(g_earned_dist))
            # print("g_dist:"+ str(g_dist))
            self.logger.info("青判定")
            return Status.SUCCESS
        else:
            return Status.RUNNING

class IsDistanceEarned_before(Behaviour):
    def __init__(self, name: str, delta_dist: int):
        super(IsDistanceEarned_before, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_dist = delta_dist
        self.running = False
        self.earned = False

    def update(self) -> Status:
        global g_earned_dist
        global g_dist
        if not self.running:
            self.running = True
            self.orig_dist = g_plotter.get_distance()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, self.delta_dist))
        cur_dist = g_plotter.get_distance()
        g_earned_dist = cur_dist - self.orig_dist

        if (g_earned_dist >= self.delta_dist or -g_earned_dist <= -self.delta_dist):
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta distance earned" % (cur_dist, self.__class__.__name__))
            global g_distFlg
            g_distFlg = True
            g_dist = 1000
            self.logger.info("グローバル変数更新 g_dist:980、g_earned_dist:0")
            return Status.SUCCESS
        else:
            return Status.RUNNING

class IsDistanceEarned_after(Behaviour):
    def __init__(self, name: str):
        super(IsDistanceEarned_after, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False
        self.earned = False

    def update(self) -> Status:
        global g_distFlg
        if g_distFlg:
            g_distFlg = False
            return Status.SUCCESS
        
        global g_dist
        global g_earned_dist
        if not self.running:
            self.running = True
            self.orig_dist = g_plotter.get_distance()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, g_dist))
        cur_dist = g_plotter.get_distance()
        g_earned_dist = cur_dist - self.orig_dist
        # print(g_dist)
        if (g_earned_dist >= g_dist or -g_earned_dist <= -g_dist):
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta distance earned" % (cur_dist, self.__class__.__name__))
            g_dist = 1000
            g_earned_dist = 0
            self.logger.info("グローバル変数更新 g_dist:980、g_earned_dist:0")
            return Status.SUCCESS
        else:
            return Status.RUNNING
        
def build_behaviour_tree() -> BehaviourTree:
    root = Sequence(name="competition", memory=True)
    calibration = Sequence(name="calibration", memory=True)
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne())
    loop_01 = Parallel(name="loop 01", policy=ParallelPolicy.SuccessOnOne())
    loop_02_1 = Parallel(name="loop 02_1", policy=ParallelPolicy.SuccessOnOne())
    loop_02 = Parallel(name="loop 02", policy=ParallelPolicy.SuccessOnOne())
    loop_03 = Parallel(name="loop 03", policy=ParallelPolicy.SuccessOnOne())
    loop_04 = Parallel(name="loop 04", policy=ParallelPolicy.SuccessOnOne())
    loop_05 = Parallel(name="loop 05", policy=ParallelPolicy.SuccessOnOne())
    loop_06 = Parallel(name="loop 06", policy=ParallelPolicy.SuccessOnOne())
    loop_07 = Parallel(name="loop 07", policy=ParallelPolicy.SuccessOnOne())
    loop_08 = Parallel(name="loop 08", policy=ParallelPolicy.SuccessOnOne())
    loop_09 = Parallel(name="loop 09", policy=ParallelPolicy.SuccessOnOne())
    loop_10 = Parallel(name="loop 10", policy=ParallelPolicy.SuccessOnOne())
    loop_11 = Parallel(name="loop 11", policy=ParallelPolicy.SuccessOnOne())
    loop_12 = Parallel(name="loop 12", policy=ParallelPolicy.SuccessOnOne())
    loop_13 = Parallel(name="loop 13", policy=ParallelPolicy.SuccessOnOne())
    loop_14 = Parallel(name="loop 14", policy=ParallelPolicy.SuccessOnOne())
    loop_14_1 = Parallel(name="loop 14_1", policy=ParallelPolicy.SuccessOnOne())
    loop_15 = Parallel(name="loop 15", policy=ParallelPolicy.SuccessOnOne())
    loop_16 = Parallel(name="loop 16", policy=ParallelPolicy.SuccessOnOne())
    loop_17 = Parallel(name="loop 17", policy=ParallelPolicy.SuccessOnOne())
    loop_18 = Parallel(name="loop 18", policy=ParallelPolicy.SuccessOnOne())
    loop_19 = Parallel(name="loop 19", policy=ParallelPolicy.SuccessOnOne())
    loop_20 = Parallel(name="loop 20", policy=ParallelPolicy.SuccessOnOne())
    loop_21 = Parallel(name="loop 21", policy=ParallelPolicy.SuccessOnOne())
    loop_22 = Parallel(name="loop 22", policy=ParallelPolicy.SuccessOnOne())
    loop_23 = Parallel(name="loop 23", policy=ParallelPolicy.SuccessOnOne())
    loop_24 = Parallel(name="loop 24", policy=ParallelPolicy.SuccessOnOne())
    loop_25 = Parallel(name="loop 25", policy=ParallelPolicy.SuccessOnOne())
    loop_26 = Parallel(name="loop 26", policy=ParallelPolicy.SuccessOnOne())
    loop_27 = Parallel(name="loop 27", policy=ParallelPolicy.SuccessOnOne())
    loop_28 = Parallel(name="loop 28", policy=ParallelPolicy.SuccessOnOne())
    loop_28_1 = Parallel(name="loop 28_1", policy=ParallelPolicy.SuccessOnOne())
    loop_29 = Parallel(name="loop 29", policy=ParallelPolicy.SuccessOnOne())
    loop_30 = Parallel(name="loop 30", policy=ParallelPolicy.SuccessOnOne())
    loop_31 = Parallel(name="loop 31", policy=ParallelPolicy.SuccessOnOne())
    loop_32 = Parallel(name="loop 32", policy=ParallelPolicy.SuccessOnOne())
    loop_33 = Parallel(name="loop 33", policy=ParallelPolicy.SuccessOnOne())
    loop_34 = Parallel(name="loop 34", policy=ParallelPolicy.SuccessOnOne())
    loop_35 = Parallel(name="loop 35", policy=ParallelPolicy.SuccessOnOne())
    loop_36 = Parallel(name="loop 36", policy=ParallelPolicy.SuccessOnOne())
    loop_37 = Parallel(name="loop 37", policy=ParallelPolicy.SuccessOnOne())
    loop_38 = Parallel(name="loop 37", policy=ParallelPolicy.SuccessOnOne())
    calibration.add_children(
        [
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            ResetDevice(name="device reset"),
        ]
    )
    start.add_children(
        [
            IsSonarOn(name="soner start", alert_dist=50),
            IsTouchOn(name="touch start"),
        ]
    )
# 1列目
    # 指定距離走行_before、赤青判定
    loop_01.add_children(
        [
        TraceLineCam(name="trace normal edge", power=43, pid_p=0.8, pid_i=0.0015, pid_d=0.1,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
        IsDistanceEarned_before(name="check distance", delta_dist = 1600),
        IsRedColorDetected(name="check red color", threshold=12.0), 
        IsBlueColorDetected(name="check blue color", threshold=12.0), 
        ]
    )
    loop_02_1.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=0, left_power=60, target_distance=40),
        ]
    )
    loop_02.add_children(
        [
            MoveStraight_dbr(name="move straight", power=40, target_distance=110),
        ]
    )
    loop_03.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=60, left_power=10, target_distance=240),
        ]
    )
    loop_04.add_children(
        [
            MoveStraight_dbr(name="back", power=-50, target_distance=20)
        ]
    )
    loop_05.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=-60, left_power=-10, target_distance=180),
        ]
    )
    loop_06.add_children(
        [
            MoveStraight_dbr(name="back", power=-50, target_distance=70)
        ]
    )
    # 指定距離走行_after
    loop_07.add_children(
        [
        TraceLineCam(name="trace normal edge", power=40, pid_p=1.0, pid_i=0.0015, pid_d=0.1,gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
        IsDistanceEarned_after(name="check distance"),
        ]
    )
    # 押し出し
    loop_08.add_children(
        [
            MoveStraight(name="move straight", power=40, target_distance=400),
        ]
    )
    # バック
    loop_09.add_children(
        [
            MoveStraight(name="back", power=-30, target_distance=150)
        ]
    )
    # 右に90度回転
    loop_10.add_children(
        [
            MoveStraightLR(name="move straight 4", right_power=0, left_power=60, target_distance=110),
        ]
    )
    # 指定距離走行_1列目から2列目移動
    loop_11.add_children(
        [
        TraceLineCam(name="trace normal edge", power=31, pid_p=1.0, pid_i=0.0015, pid_d=0.1,gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
        IsDistanceEarned(name="check distance", delta_dist=270),
        ]
    )
    # 右に90度回転
    loop_12.add_children(
        [
            MoveStraightLR(name="move straight 4", right_power=0, left_power=60, target_distance=130),
        ]
    )
# 2列目
    # 指定距離走行_before、赤青判定
    loop_13.add_children(
        [
        TraceLineCam(name="trace normal edge", power=31, pid_p=1.0, pid_i=0.0015, pid_d=0.1,
                         gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
        IsDistanceEarned_before(name="check distance", delta_dist = 1050),
        IsRedColorDetected(name="check red color", threshold=12.0), 
        IsBlueColorDetected(name="check blue color", threshold=12.0), 
        ]
    )
    loop_14_1.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=0, left_power=60, target_distance=40),
        ]
    )
    loop_14.add_children(
        [
            MoveStraight_dbr(name="move straight", power=40, target_distance=120),
        ]
    )
    loop_15.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=60, left_power=10, target_distance=240),
        ]
    )
    loop_16.add_children(
        [
            MoveStraight_dbr(name="back", power=-50, target_distance=20)
        ]
    )
    loop_17.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=-60, left_power=-10, target_distance=180),
        ]
    )
    loop_18.add_children(
        [
            MoveStraight_dbr(name="back", power=-50, target_distance=50)
        ]
    )
    # 指定距離走行_after
    loop_19.add_children(
        [
        TraceLineCam(name="trace normal edge", power=32, pid_p=1.0, pid_i=0.0015, pid_d=0.1,gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
        IsDistanceEarned_after(name="check distance"),
        ]
    )
    # 押し出し
    loop_20.add_children(
        [
            MoveStraight(name="move straight", power=40, target_distance=400),
        ]
    )
    # バック
    loop_21.add_children(
        [
            MoveStraight(name="back", power=-30, target_distance=160)
        ]
    )
    # 左に90度回転
    loop_22.add_children(
        [
            MoveStraightLR(name="move straight 4", right_power=60, left_power=0, target_distance=240),
        ]
    )
    # 指定距離走行_2列目から3列目移動
    loop_23.add_children(
        [
        TraceLineCam(name="trace normal edge", power=30, pid_p=0.8, pid_i=0.0015, pid_d=0.1,gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
        IsDistanceEarned(name="check distance", delta_dist=230),
        ]
    )
    # 押し出し
    loop_24.add_children(
        [
            MoveStraight(name="move straight", power=40, target_distance=270),
        ]
    )
    # バック
    loop_25.add_children(
        [
            MoveStraight(name="back", power=-30, target_distance=220)
        ]
    )
    # 左に90度回転
    loop_26.add_children(
        [
            MoveStraightLR(name="move straight 4", right_power=60, left_power=0, target_distance=230),
        ]
    )
# 3列目
    # 指定距離走行_before、赤青判定
    loop_27.add_children(
        [
        TraceLineCam(name="trace normal edge", power=32, pid_p=0.8, pid_i=0.0015, pid_d=0.1,
                         gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
        IsDistanceEarned_before(name="check distance", delta_dist = 1050),
        IsRedColorDetected(name="check red color", threshold=12.0), 
        IsBlueColorDetected(name="check blue color", threshold=12.0), 
        ]
    )
    loop_28_1.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=0, left_power=60, target_distance=40),
        ]
    )
    loop_28.add_children(
        [
            MoveStraight_dbr(name="move straight", power=40, target_distance=120),
        ]
    )
    loop_29.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=60, left_power=10, target_distance=250),
        ]
    )
    loop_30.add_children(
        [
            MoveStraight_dbr(name="back", power=-50, target_distance=20)
        ]
    )
    loop_31.add_children(
        [
            MoveStraightLR_dbr(name="move straight 4", right_power=-60, left_power=-10, target_distance=180),
        ]
    )
    loop_32.add_children(
        [
            MoveStraight_dbr(name="back", power=-50, target_distance=50)
        ]
    )
    # 指定距離走行_after
    loop_33.add_children(
        [
        TraceLineCam(name="trace normal edge", power=35, pid_p=1.0, pid_i=0.0015, pid_d=0.1,gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
        IsDistanceEarned_after(name="check distance"),
        ]
    )
    # 押し出し
    loop_34.add_children(
        [
            MoveStraight(name="move straight", power=40, target_distance=370),
        ]
    )
    # バック
    loop_35.add_children(
        [
            MoveStraight(name="back", power=-30, target_distance=120)
        ]
    )
    # 右に90度回転
    loop_36.add_children(
        [
            MoveStraightLR(name="move straight 4", right_power=0, left_power=60, target_distance=110),
        ]
    )
    # 指定距離走行_3列目から4列目移動
    loop_37.add_children(
        [
        TraceLineCam(name="trace normal edge", power=31, pid_p=1.0, pid_i=0.0015, pid_d=0.1,gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
        IsDistanceEarned(name="check distance", delta_dist=250),
        ]
    )
    # 右に90度回転
    loop_38.add_children(
        [
            MoveStraightLR(name="move straight 4", right_power=0, left_power=60, target_distance=110),
        ]
    )
    root.add_children(
        [
            calibration,
            start,
            loop_01,
            loop_02_1,
            loop_02,
            loop_03,
            loop_04,
            loop_05,
            loop_06,
            loop_07,
            loop_08,
            loop_09,
            loop_10,
            loop_11,
            loop_12,
            loop_13,
            loop_14,
            # loop_14_1,
            loop_15,
            loop_16,
            loop_17,
            loop_18,
            loop_19,
            loop_20,
            loop_21,
            loop_22,
            loop_23,
            loop_24,
            loop_25,
            loop_26,
            loop_27,
            loop_28,
            # loop_28_1,
            loop_29,
            loop_30,
            loop_31,
            loop_32,
            loop_33,
            loop_34,
            loop_35,
            loop_36,
            loop_37,
            loop_38,
            StopNow(name="stop"),
            TheEnd(name="end"),
        ]
    )
    return root

def initialize_etrobo(backend: str) -> ETRobo:
    return (ETRobo(backend=backend)
            .add_hub('hub')
            .add_device('arm_motor', device_type=Motor, port='A')
            .add_device('right_motor', device_type=Motor, port='B')
            .add_device('left_motor', device_type=Motor, port='C')
            .add_device('touch_sensor', device_type=TouchSensor, port='1')
            .add_device('color_sensor', device_type=ColorSensor, port='2')
            .add_device('sonar_sensor', device_type=SonarSensor, port='3')
            .add_device('gyro_sensor', device_type=GyroSensor, port='4'))

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
    parser.add_argument('--port', default='/dev/ttyAMA1', help='Serial port')
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()

    if args.course == 'right':
        g_course = -1
    else:
        g_course = 1

    setup_thread()

    #py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = build_behaviour_tree()
    display_tree.render_dot_tree(tree)

    signal.signal(signal.SIGTERM, sig_handler)

    try:
        etrobo = initialize_etrobo(backend='raspike')
        etrobo.add_handler(ExposeDevices())
        etrobo.add_handler(TraverseBehaviourTree(tree))
        etrobo.dispatch(interval=EXEC_INTERVAL, port=args.port, logfile=args.logfile)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cleanup_thread()
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- exiting...")
