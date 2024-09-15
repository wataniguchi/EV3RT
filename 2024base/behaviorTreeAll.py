import argparse
import time
import math
import threading
import signal
import colorsys
from enum import Enum, IntEnum, auto
from etrobo_python import ETRobo, Hub, Motor, ColorSensor, TouchSensor, SonarSensor, GyroSensor
from simple_pid import PID
from py_trees.common import ParallelPolicy
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import (
    display as display_tree,
    logging as log_tree
)
from py_etrobo_util import Video, Plotter

from const import (
    ArmDirection,
    TraceNum,
    DebriNum,
    TraceSide,
    JState,
    Wheel,
    Scene
)

EXEC_INTERVAL: float = 0.04
VIDEO_INTERVAL: float = 0.02
ARM_SHIFT_PWM = 30
JUNCT_UPPER_THRESH = 50
JUNCT_LOWER_THRESH = 40


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
g_earned_dist_debri: int = 0
g_debri_end: bool = False
g_debri_exist_bottle: bool = False

now_color = []


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


class IsDistanceEarnedTrace(Behaviour):
    def __init__(self, name: str, delta_dist: int):
        super(IsDistanceEarnedTrace, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_dist = delta_dist
        self.running = False
        self.earned = False

    def update(self) -> Status:
        global g_earned_dist_debri, g_debri_end
        if not self.running:
            g_debri_end = False
            self.running = True
            self.orig_dist = g_plotter.get_distance()
            self.g_earned_dist = g_earned_dist_debri
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, self.delta_dist))
        cur_dist = g_plotter.get_distance()
        earned_dist = cur_dist - self.orig_dist + self.g_earned_dist
        g_earned_dist_debri=earned_dist
        if (earned_dist >= self.delta_dist or -earned_dist <= -self.delta_dist):
            if not self.earned:
                self.earned = True
                g_earned_dist_debri = 0
                g_debri_end = True
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


class IsExistBottle(Behaviour):
    def __init__(self, name: str) -> None:
        global g_debri_exist_bottle
        super(IsExistBottle, self).__init__(name)

        self.running = False
        g_debri_exist_bottle = False

    def update(self) -> Status:
        global g_debri_exist_bottle

        if not self.running:
            self.running = True
        if(g_video.get_range_of_blue()>DebriNum.BOTTLE_RANGE_LOWER or 
           g_video.get_range_of_red()>DebriNum.BOTTLE_RANGE_LOWER):
            g_debri_exist_bottle = True
            return Status.SUCCESS
        return Status.RUNNING


class IsEndDebri(Behaviour):
    def __init__(self, name: str):
        super(IsEndDebri, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        if(g_debri_end):
            return Status.SUCCESS
        else:
            return Status.FAILURE


class IsRotated(Behaviour):
    def __init__(self, name: str, delta_dire: int):
        super(IsRotated, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False
        self.delta = delta_dire*Wheel.WHEEL_TREAD/Wheel.TIRE_DIAMETER*2
        self.rStart = 0
        self.lStart = 0

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.rStart = g_right_motor.get_count()
            self.lStart = g_left_motor.get_count()
        rDiff = abs(g_right_motor.get_count() - self.rStart)
        lDiff = abs(g_left_motor.get_count() - self.lStart)

        if((rDiff>self.delta) or (lDiff>self.delta)):
            self.running = False
            return Status.SUCCESS
        else:
            return Status.RUNNING

        
class CheckBrackColor(Behaviour):
    def __init__(self, name: str):
        super(CheckBrackColor, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False
    def update(self) -> Status:
        r, g, b = [x for x in g_color_sensor.get_raw_color()]
        # # RGBをHSVに変換
        # h, s, v = colorsys.rgb_to_hsv(r, g, b)
        # # Hueの値が青色の範囲（例: 180度〜250度程度）にあるかをチェック
        # # Hueは0.0〜1.0の範囲で返されるので、360度に換算する
        # h_degrees = h * 360

        # 青色の範囲をチェック
        # if 200 <= h_degrees <= 245 and s > 0.3 and v > 0.2:
        #     return Status.SUCCESS
        # else:
        #     return Status.RUNNING
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started" % (g_plotter.get_distance(), self.__class__.__name__))

        global now_color
        if not now_color:
            now_color = [r,g,b]
        else:
            self.logger.info('now_color:{}'.format(now_color))
            diff_r = now_color[0] - r
            diff_g = now_color[1] - g
            diff_b = now_color[2] - b

            self.logger.debug('diff_r:{}'.format(diff_r))
            if diff_r>=40 :
                return Status.SUCCESS

            now_color = [r,g,b]
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
        super(RunAsInstructed, self).__init__(name)
        if(g_course==-1):
            self.pwm_r = pwm_r
            self.pwm_l = pwm_l
        else:
            self.pwm_r = pwm_l
            self.pwm_l = pwm_r

        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))
        g_right_motor.set_power(self.pwm_r)
        if(g_course==-1):
            g_left_motor.set_power(self.pwm_l)
        else:
            g_left_motor.set_power(self.pwm_l-2)
        return Status.RUNNING


class ReturnSuccess(Behaviour):
    def __init__(self, name: str) -> None:
        super(ReturnSuccess, self).__init__(name)

    def update(self) -> Status:
        return Status.SUCCESS


class IsExecuteRemoveBottle(Behaviour):
    def __init__(self, name: str) -> None:
        super(IsExecuteRemoveBottle, self).__init__(name)

    def update(self) -> Status:
        global g_debri_exist_bottle

        if(g_debri_exist_bottle):
            return Status.SUCCESS
        else:
            return Status.FAILURE


class TraceLineCam(Behaviour):
    def __init__(self, name: str, power: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide, scene: Scene) -> None:
        super(TraceLineCam, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.gs_min = gs_min
        self.gs_max = gs_max
        self.trace_side = trace_side
        self.scene = scene
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
        if(self.scene == Scene.LOOP and g_course == -1):
            g_left_motor.set_power(self.power + turn-round(self.power/15))
        else:
            g_left_motor.set_power(self.power + turn)
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


def build_behaviour_tree() -> BehaviourTree:
    root = Sequence(name="competition", memory=True)

    calibration = Sequence(name="calibration", memory=True)
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne())

    loop = Sequence(name="loop", memory=True)
    loop_01 = Parallel(name="start loop", policy=ParallelPolicy.SuccessOnOne())
    loop_02 = Parallel(name="straight 01", policy=ParallelPolicy.SuccessOnOne())
    loop_03 = Parallel(name="curve 01", policy=ParallelPolicy.SuccessOnOne())
    loop_04 = Parallel(name="straight 02", policy=ParallelPolicy.SuccessOnOne())
    loop_05 = Parallel(name="curve 02", policy=ParallelPolicy.SuccessOnOne())
    loop_06 = Parallel(name="straight 03", policy=ParallelPolicy.SuccessOnOne())
    loop_07 = Parallel(name="junction 01", policy=ParallelPolicy.SuccessOnOne())
    loop_08 = Parallel(name="circle 01", policy=ParallelPolicy.SuccessOnOne())
    loop_09 = Parallel(name="junction 02", policy=ParallelPolicy.SuccessOnOne())
    loop_10 = Parallel(name="ellipse 01", policy=ParallelPolicy.SuccessOnOne())
    loop_11 = Parallel(name="junction 03", policy=ParallelPolicy.SuccessOnOne())
    loop_12 = Parallel(name="ellipse 02", policy=ParallelPolicy.SuccessOnOne())
    loop_13 = Parallel(name="junction 04", policy=ParallelPolicy.SuccessOnOne())
    loop_14 = Parallel(name="circle 02", policy=ParallelPolicy.SuccessOnOne())
    loop_15 = Parallel(name="junction 05", policy=ParallelPolicy.SuccessOnOne())
    loop_16 = Parallel(name="straight 04", policy=ParallelPolicy.SuccessOnOne())
    loop_17 = Parallel(name="end loop", policy=ParallelPolicy.SuccessOnOne())

    debri = Sequence(name="debri", memory=True)
    debri_01 = Parallel(name="vertical 01", policy=ParallelPolicy.SuccessOnOne())
    debri_02 = Selector(name="remove 01", memory=True)
    debri_02_01 = Sequence(name="remove", memory=True)
    debri_02_01_01 = Parallel(name="is exe remove", policy=ParallelPolicy.SuccessOnOne())
    debri_02_01_02 = Parallel(name="catch bottle", policy=ParallelPolicy.SuccessOnOne())
    debri_02_01_03 = Parallel(name="remove", policy=ParallelPolicy.SuccessOnOne())
    debri_02_01_04 = Parallel(name="go back 01", policy=ParallelPolicy.SuccessOnOne())
    debri_02_01_05 = Parallel(name="go back 02", policy=ParallelPolicy.SuccessOnOne())
    debri_02_01_06 = Parallel(name="vertical 01", policy=ParallelPolicy.SuccessOnOne())
    debri_03 = Parallel(name="debri 03", policy=ParallelPolicy.SuccessOnOne())
    debri_04 = Parallel(name="debri 04", policy=ParallelPolicy.SuccessOnOne())
    debri_05 = Parallel(name="debri 05", policy=ParallelPolicy.SuccessOnOne())
    debri_06 = Parallel(name="debri 06", policy=ParallelPolicy.SuccessOnOne())
    debri_07 = Parallel(name="debri 07", policy=ParallelPolicy.SuccessOnOne())
    debri_08 = Selector(name="debri 08", memory=True)
    debri_08_02 = Sequence(name="debri 08 02", memory=True)
    debri_08_02_01 = Parallel(name="debri 08 02 01", policy=ParallelPolicy.SuccessOnOne())
    debri_08_02_02 = Parallel(name="debri 08 02 02", policy=ParallelPolicy.SuccessOnOne())
    debri_08_02_03 = Parallel(name="debri 08 02 03", policy=ParallelPolicy.SuccessOnOne())
    debri_08_02_04 = Parallel(name="debri 08 02 04", policy=ParallelPolicy.SuccessOnOne())
    debri_08_02_05 = Parallel(name="debri 08 02 05", policy=ParallelPolicy.SuccessOnOne())

    carry = Sequence(name="carry", memory=True)
    carry_01 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_02 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_03 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_04 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_05 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_06 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_07 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_08 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_09 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    carry_10 = Parallel(name="carry 01", policy=ParallelPolicy.SuccessOnOne())
    
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

    loop.add_children([
        loop_01,
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
        loop_15,
        loop_16,
        loop_17,
    ])
    loop_01.add_children(
        [
            TraceLineCam(name="straight 01", power=TraceNum.POWER_JUNCTION,                    
                         pid_p=TraceNum.PID_P_SLOW, pid_i=TraceNum.PID_I_SLOW, pid_d=TraceNum.PID_D_SLOW,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 250),
        ]
    )
    loop_02.add_children(
        [
            TraceLineCam(name="straight 01", power=TraceNum.POWER_FAST,                    
                         pid_p=TraceNum.PID_P_FAST, pid_i=TraceNum.PID_I_FAST, pid_d=TraceNum.PID_D_FAST,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 3300),
        ]
    )
    loop_03.add_children(
        [
            TraceLineCam(name="curve 01", power=TraceNum.POWER_SLOW,                    
                         pid_p=TraceNum.PID_P_SLOW, pid_i=TraceNum.PID_I_SLOW, pid_d=TraceNum.PID_D_SLOW,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 1300),
        ]
    )
    loop_04.add_children(
        [
            TraceLineCam(name="straight 02", power=TraceNum.POWER_FAST,
                         pid_p=TraceNum.PID_P_FAST, pid_i=TraceNum.PID_I_FAST, pid_d=TraceNum.PID_D_FAST,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 2100),
        ]
    )
    loop_05.add_children(
        [
            TraceLineCam(name="curve 02", power=TraceNum.POWER_SLOW,                    
                         pid_p=TraceNum.PID_P_SLOW, pid_i=TraceNum.PID_I_SLOW, pid_d=TraceNum.PID_D_SLOW,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 1200),
        ]
    )
    loop_06.add_children(
        [
            TraceLineCam(name="straight 03", power=TraceNum.POWER_SLOW,                    
                         pid_p=TraceNum.PID_P_SLOW, pid_i=TraceNum.PID_I_SLOW, pid_d=TraceNum.PID_D_SLOW,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 450),
        ]
    )
    loop_07.add_children(
        [
            TraceLineCam(name="junction 01", power=TraceNum.POWER_JUNCTION,                    
                         pid_p=TraceNum.PID_P_SLOW, pid_i=TraceNum.PID_I_SLOW, pid_d=TraceNum.PID_D_SLOW,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsJunction(name="scan joined junction", target_state = JState.JOINED),
        ]
    )
    loop_08.add_children(
        [
            TraceLineCam(name="circle 01", power=TraceNum.POWER_CIRCLE,                    
                         pid_p=TraceNum.PID_P_CIRCLE, pid_i=TraceNum.PID_I_CIRCLE, pid_d=TraceNum.PID_D_CIRCLE,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
            IsDistanceEarned(name="check distance", delta_dist = 2400),
        ]
    )
    loop_09.add_children(
        [
            TraceLineCam(name="junction 02", power=TraceNum.POWER_JUNCTION,                    
                         pid_p=TraceNum.PID_P_CIRCLE, pid_i=TraceNum.PID_I_CIRCLE, pid_d=TraceNum.PID_D_CIRCLE,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
            IsJunction(name="scan joined junction", target_state = JState.JOINED),
        ]
    )
    loop_10.add_children(
        [
            TraceLineCam(name="ellipse 01", power=TraceNum.POWER_ELLIPSE,                    
                         pid_p=TraceNum.PID_P_ELLIPSE, pid_i=TraceNum.PID_I_ELLIPSE, pid_d=TraceNum.PID_D_ELLIPSE,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 1400),
        ]
    )
    loop_11.add_children(
        [
            TraceLineCam(name="junction 03", power=TraceNum.POWER_JUNCTION,                    
                         pid_p=TraceNum.PID_P_ELLIPSE, pid_i=TraceNum.PID_I_ELLIPSE, pid_d=TraceNum.PID_D_ELLIPSE,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 700),
        ]
    )
    loop_12.add_children(
        [
            TraceLineCam(name="ellipse 02", power=TraceNum.POWER_ELLIPSE,                    
                         pid_p=TraceNum.PID_P_ELLIPSE, pid_i=TraceNum.PID_I_ELLIPSE, pid_d=TraceNum.PID_D_ELLIPSE,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 1000),
        ]
    )
    loop_13.add_children(
        [
            TraceLineCam(name="junction 04", power=TraceNum.POWER_JUNCTION,                    
                         pid_p=TraceNum.PID_P_ELLIPSE, pid_i=TraceNum.PID_I_ELLIPSE, pid_d=TraceNum.PID_D_ELLIPSE,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsJunction(name="scan joined junction", target_state = JState.JOINED),
        ]
    )
    loop_14.add_children(
        [
            TraceLineCam(name="circle 02", power=TraceNum.POWER_CIRCLE,                    
                         pid_p=TraceNum.PID_P_CIRCLE, pid_i=TraceNum.PID_I_CIRCLE, pid_d=TraceNum.PID_D_CIRCLE,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
            IsDistanceEarned(name="check distance", delta_dist = 900),
        ]
    )
    loop_15.add_children(
        [
            TraceLineCam(name="junction 05", power=TraceNum.POWER_JUNCTION,                    
                         pid_p=TraceNum.PID_P_CIRCLE, pid_i=TraceNum.PID_I_CIRCLE, pid_d=TraceNum.PID_D_CIRCLE,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.OPPOSITE),
            IsJunction(name="scan joined junction", target_state = JState.JOINED),
        ]
    )
    loop_16.add_children(
        [
            TraceLineCam(name="straight 04", power=TraceNum.POWER_SLOW,                    
                         pid_p=TraceNum.PID_P_SLOW, pid_i=TraceNum.PID_I_SLOW, pid_d=TraceNum.PID_D_SLOW,
                         scene=Scene.LOOP,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 500),
        ]
    )
    loop_17.add_children(
        [
            TraceLineCam(name="straight 04", power=TraceNum.POWER_SLOW,                    
                         pid_p=TraceNum.PID_P_SLOW, pid_i=TraceNum.PID_I_SLOW, pid_d=TraceNum.PID_D_SLOW,
                         scene=Scene.DEBRI,
                         gs_min=0, gs_max=80, trace_side=TraceSide.CENTER),
            IsDistanceEarned(name="check distance", delta_dist = 700),
        ]
    )

    debri.add_children([
        debri_01,
        debri_02,
        debri_03,
        debri_04,
        debri_05,
        debri_06,
        debri_07,
        debri_08,
    ])
    debri_01.add_children(
        [
            TraceLineCam(name="run vertical", power=DebriNum.POWER_NORMAL, 
                         pid_p=DebriNum.PID_P, pid_i=DebriNum.PID_I, pid_d=DebriNum.PID_D,
                         gs_min=0, gs_max=80, scene=Scene.DEBRI, trace_side=TraceSide.CENTER),
            IsDistanceEarnedTrace(name="check distance01", delta_dist=DebriNum.DISTANCE_PASSED_THIRD),
            IsExistBottle(name="check bottle"),
        ]
    )

    debri_02.add_children(
        [
            debri_02_01,
            ReturnSuccess(name="seccess"),
        ]
    )
    debri_02_01.add_children(
        [
            debri_02_01_01,
            debri_02_01_02,
            debri_02_01_03,
            debri_02_01_04,
            debri_02_01_05,
            debri_02_01_06,
        ]
    )
    debri_02_01_01.add_children(
        [
            IsExecuteRemoveBottle(name="is execute remove"),
        ]
    )
    debri_02_01_02.add_children(
        [
            RunAsInstructed(name="run", pwm_r=40,pwm_l=40),
            IsDistanceEarned(name="check distance", delta_dist = DebriNum.DISTANCE_CATCH),
        ]
    )
    debri_02_01_03.add_children(
        [
            RunAsInstructed(name="remove", pwm_r=DebriNum.POWER_REMOVE_RIGHT,pwm_l=DebriNum.POWER_REMOVE_LEFT),
            IsDistanceEarned(name="check distance", delta_dist = DebriNum.DISTANCE_REMOVE),
        ]
    )
    debri_02_01_04.add_children(
        [
            RunAsInstructed(name="remove", pwm_r=-(DebriNum.POWER_REMOVE_RIGHT),pwm_l=-(DebriNum.POWER_REMOVE_LEFT)),
            IsDistanceEarned(name="check distance", delta_dist = DebriNum.DISTANCE_REMOVE-50),
        ]
    )
    debri_02_01_05.add_children(
        [
            RunAsInstructed(name="go back", pwm_r=-40,pwm_l=-40),
            IsDistanceEarned(name="check distance", delta_dist = DebriNum.DISTANCE_CATCH),
        ]
    )
    debri_02_01_06.add_children(
        [
            TraceLineCam(name="run vertical", power=DebriNum.POWER_NORMAL, 
                         pid_p=DebriNum.PID_P, pid_i=DebriNum.PID_I, pid_d=DebriNum.PID_D,
                         gs_min=0, gs_max=80, scene=Scene.DEBRI, trace_side=TraceSide.CENTER),
            IsDistanceEarnedTrace(name="check distance01", delta_dist=DebriNum.DISTANCE_PASSED_THIRD),
        ]
    )

    debri_03.add_children(
        [
            TraceLineCam(name="run vertical", power=DebriNum.POWER_NORMAL, 
                         pid_p=DebriNum.PID_P, pid_i=DebriNum.PID_I, pid_d=DebriNum.PID_D,
                         gs_min=0, gs_max=80, scene=Scene.DEBRI, trace_side=TraceSide.CENTER),
            IsDistanceEarned(name="check distance02", delta_dist=290),
        ]
    )
    debri_04.add_children(
        [
            RunAsInstructed(name="remove", pwm_r=40,pwm_l=40),
            IsDistanceEarned(name="check distance", delta_dist = 300),
        ]
    )
    debri_05.add_children(
        [
            RunAsInstructed(name="go back", pwm_r=-40,pwm_l=-40),
            IsDistanceEarned(name="check distance", delta_dist = 150),
        ]
    )

    debri_06.add_children(
        [
            RunAsInstructed(name="rotate", pwm_r=60,pwm_l=0),
            IsRotated(name="check rotated", delta_dire=90),
        ]
    )
    debri_07.add_children(
        [
            TraceLineCam(name="run side", power=DebriNum.POWER_SLOW, 
                         pid_p=DebriNum.PID_P, pid_i=DebriNum.PID_I, pid_d=DebriNum.PID_D,
                         gs_min=0, gs_max=80, scene=Scene.DEBRI, trace_side=TraceSide.CENTER),
            IsDistanceEarnedTrace(name="check distance04", delta_dist = DebriNum.DISTANCE_SIDE),
            IsExistBottle(name="check bottle"),
        ]
    )
    debri_08.add_children(
        [
            IsEndDebri(name="judge end"),
            debri_08_02,
        ]
    )
    debri_08_02.add_children(
        [
            debri_08_02_01,
            debri_08_02_02,
            debri_08_02_03,
            debri_08_02_04,
            debri_08_02_05,
        ]
    )
    debri_08_02_01.add_children(
        [
            RunAsInstructed(name="run", pwm_r=40,pwm_l=40),
            IsDistanceEarned(name="check distance", delta_dist = DebriNum.DISTANCE_CATCH),
        ]
    )
    debri_08_02_02.add_children(
        [
            RunAsInstructed(name="remove", pwm_r=DebriNum.POWER_REMOVE_RIGHT,pwm_l=DebriNum.POWER_REMOVE_LEFT),
            IsDistanceEarned(name="check distance", delta_dist = DebriNum.DISTANCE_REMOVE),
        ]
    )
    debri_08_02_03.add_children(
        [
            RunAsInstructed(name="remove", pwm_r=-(DebriNum.POWER_REMOVE_RIGHT),pwm_l=-(DebriNum.POWER_REMOVE_LEFT)),
            IsDistanceEarned(name="check distance", delta_dist = DebriNum.DISTANCE_REMOVE-50),
        ]
    )
    debri_08_02_04.add_children(
        [
            RunAsInstructed(name="go back", pwm_r=-40,pwm_l=-40),
            IsDistanceEarned(name="check distance", delta_dist = DebriNum.DISTANCE_CATCH),
        ]
    )
    debri_08_02_05.add_children(
        [
            TraceLineCam(name="run side", power=DebriNum.POWER_SLOW, 
                         pid_p=DebriNum.PID_P, pid_i=DebriNum.PID_I, pid_d=DebriNum.PID_D,
                         gs_min=0, gs_max=80, scene=Scene.DEBRI, trace_side=TraceSide.CENTER),
            IsDistanceEarnedTrace(name="check distance04", delta_dist = DebriNum.DISTANCE_SIDE),
        ]
    )
    
    carry.add_children([
        carry_01,
        carry_02,
        carry_03,
        carry_04,
        carry_05,
        carry_06,
        carry_07,
        carry_08,
        carry_09,
        carry_10,
    ])

    carry_01.add_children(
        [
            RunAsInstructed(name="go straight",pwm_l=38,pwm_r=38),
            IsDistanceEarned(name="check distance", delta_dist = 400),
        ]
    )
    
    carry_02.add_children(
        [
            TraceLineCam(name="trace normal edge", power=34, pid_p=1.9, pid_i=0.001, pid_d=0.1,scene=Scene.DEBRI, gs_min=0, gs_max=80, trace_side=TraceSide.CENTER),
            IsDistanceEarned(name="check distance", delta_dist = 500),
        ]
    )
    if(g_course==-1):
        carry_03.add_children(
                [
                    RunAsInstructed(name="rotate", pwm_r=60,pwm_l=20),
                    IsDistanceEarned(name="check distance", delta_dist = 550),
                ]
        )
    else:
        carry_03.add_children(
                [
                    RunAsInstructed(name="rotate", pwm_r=60,pwm_l=20),
                    IsDistanceEarned(name="check distance", delta_dist = 400),
                ]
        )
    carry_04.add_children(
        [
            RunAsInstructed(name="go straight",pwm_l=45,pwm_r=45),
            IsDistanceEarned(name="check distance", delta_dist = 1100),
        ]
    )
    carry_05.add_children(
        [
            RunAsInstructed(name="go straight",pwm_l=-40,pwm_r=-40),
            IsDistanceEarned(name="check distance", delta_dist = 250),
        ]
    )
    carry_06.add_children(
        [
            RunAsInstructed(name="rotate", pwm_r=0,pwm_l=60),
            IsRotated(name="check rotated", delta_dire=100),
        ]
    )
    carry_07.add_children(
        [
            RunAsInstructed(name="go straight",pwm_l=38,pwm_r=38),
            CheckBrackColor(name="checkBrackColor")
        ]
    )
    carry_08.add_children(
        [
            RunAsInstructed(name="rotate", pwm_r=0,pwm_l=60),
            IsRotated(name="check rotated", delta_dire=100),
        ]
    )
    carry_09.add_children(
        [
            RunAsInstructed(name="rotate", pwm_r=-35,pwm_l=-35),
            IsRotated(name="check rotated", delta_dire=55),
        ]
    )
    carry_10.add_children(
        [
            TraceLineCam(name="trace normal edge", power=36, pid_p=1.7, pid_i=0.0015, pid_d=0.1,scene=Scene.DEBRI, gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
        ]
    )

    root.add_children(
        [
            calibration,
            start,
            loop,
            debri,
            carry,
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
