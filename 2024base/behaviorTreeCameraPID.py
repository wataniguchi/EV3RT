import argparse
import time
import math
import threading
import signal
from etrobo_python import ColorSensor, ETRobo, Hub, Motor, TouchSensor
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
from py_etrobo_util import Video, TraceSide

INTERVAL: float = 0.04
TIRE_DIAMETER: float = 100.0
WHEEL_TREAD: float = 120.0

g_distance: float = 0.0

class Plotter(object):
    def __init__(self) -> None:
        self.running = False

    def __call__(
        self,
        hub: Hub,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
    ) -> None:
        global g_distance
        if not self.running:
            self.running = True
            right_motor.reset_count()
            left_motor.reset_count()
            self.prev_ang_r = right_motor.get_count()
            self.prev_ang_l = left_motor.get_count()
            g_distance = 0.0
            return

        cur_ang_r = right_motor.get_count()
        cur_ang_l = left_motor.get_count()
        delta_dist_r = math.pi * TIRE_DIAMETER * (cur_ang_r - self.prev_ang_r) / 360.0
        delta_dist_l = math.pi * TIRE_DIAMETER * (cur_ang_l - self.prev_ang_l) / 360.0
        delta_dist = (delta_dist_r + delta_dist_l / 2.0)
        if (delta_dist >= 0.0):
            g_distance += delta_dist
        else:
            g_distance -= delta_dist
        self.prev_ang_r = cur_ang_r
        self.prev_ang_l = cur_ang_l
        return


g_plotter: Plotter = None
g_hub: Hub = None
g_right_motor: Motor = None
g_left_motor: Motor = None
g_touch_sensor: TouchSensor = None
g_color_sensor: ColorSensor = None
g_video: Video = None
g_video_thread: threading.Thread = None
g_course: int = 0

class TheEnd(Behaviour):
    def __init__(self, name: str):
        super(TheEnd, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (int(g_distance), self.__class__.__name__))
        return Status.RUNNING


class IsDistanceEarned(Behaviour):
    def __init__(self, name: str, delta_dist: int):
        super(IsDistanceEarned, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_dist = delta_dist
        self.running = False
        self.earned = False

    def update(self) -> Status:
        global g_distance
        if not self.running:
            self.running = True
            self.orig_dist = g_distance
            self.logger.info("%+06d %s.distance accumulation started for delta=%d" % (int(g_distance), self.__class__.__name__, self.delta_dist))
        earned_dist = g_distance - self.orig_dist
        if (earned_dist >= self.delta_dist or -earned_dist <= -self.delta_dist):
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta distance earned" % (int(g_distance), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsTouchOn(Behaviour):
    def __init__(self, name: str):
        super(IsTouchOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        global g_hub, g_touch_sensor
        if (g_touch_sensor.is_pressed() or
            g_hub.is_left_button_pressed() or
            g_hub.is_right_button_pressed()):
            self.logger.info("%+06d %s.touch sensor pressed" % (int(g_distance), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.FAILURE


class StopNow(Behaviour):
    def __init__(self, name: str):
        super(StopNow, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        global g_right_motor, g_left_motor
        g_right_motor.set_power(0)
        g_left_motor.set_power(0)
        self.logger.info("%+06d %s.motors stopped" % (int(g_distance), self.__class__.__name__))
        return Status.SUCCESS


class TraceLine(Behaviour):
    def __init__(self, name: str, interval: float, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,
                 trace_side: TraceSide) -> None:
        super(TraceLine, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=interval, output_limits=(-power, power))
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        global g_color_sensor, g_right_motor, g_left_motor, g_course
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (int(g_distance), self.__class__.__name__, self.trace_side.name))
        if self.trace_side == TraceSide.NORMAL:
            turn = (-1) * g_course * int(self.pid(g_color_sensor.get_brightness()))
        else: # TraceSide.OPPOSITE
            turn = g_course * int(self.pid(g_color_sensor.get_brightness()))
        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
        return Status.RUNNING


class TraceLineCam(Behaviour):
    def __init__(self, name: str, interval: float, power: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide) -> None:
        super(TraceLineCam, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=interval, output_limits=(-power, power))
        self.gs_min = gs_min
        self.gs_max = gs_max
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        global g_video, g_right_motor, g_left_motor, g_course
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
            self.logger.info("%+06d %s.trace started with TS=%s" % (int(g_distance), self.__class__.__name__, self.trace_side.name))
        turn = (-1) * int(self.pid(g_video.get_theta()))
        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
        return Status.RUNNING


class TraverseBehaviourTree(object):
    def __init__(self, tree: BehaviourTree) -> None:
        self.tree = tree
    def __call__(
        self,
            **kwargs,
    ) -> None:
        self.tree.tick_once()
    

class ExposeDevices(object):
    def __call__(
        self,
        hub: Hub,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
    ) -> None:
        global g_hub, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor
        g_hub = hub
        g_right_motor = right_motor
        g_left_motor = left_motor
        g_touch_sensor = touch_sensor
        g_color_sensor = color_sensor


class VideoThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            g_video.process()
            time.sleep(0.02)


def build_behaviour_tree() -> BehaviourTree:
    root = Sequence(name="double loop neo", memory=True)
#    section1 = Parallel(name="section 1", policy=ParallelPolicy.SuccessOnOne())
    section2 = Parallel(name="section 2", policy=ParallelPolicy.SuccessOnOne())
#    section1.add_children(
#        [
#            TraceLine(name="run", interval=INTERVAL, target=60, power=35, pid_p=0.56, pid_i=0.005, pid_d=0.015,
#                      trace_side=TraceSide.NORMAL),
#            IsDistanceEarned(name="check distance", delta_dist = 2000),
#        ]
#    )
    section2.add_children(
        [
            TraceLineCam(name="run", interval=INTERVAL, power=33, pid_p=2.5, pid_i=0.0015, pid_d=0.1,
                         gs_min=0, gs_max=80, trace_side=TraceSide.NORMAL),
            IsDistanceEarned(name="check distance", delta_dist = 2000),
        ]
    )
    root.add_children(
        [
            IsTouchOn(name="start"),
#            section1,
            section2,
            StopNow(name="stop"),
            TheEnd(name="end"),
        ]
    )
    return root

def initialize_etrobo(backend: str) -> ETRobo:
    return (ETRobo(backend=backend)
     .add_hub('hub')
     .add_device('right_motor', device_type=Motor, port='B')
     .add_device('left_motor', device_type=Motor, port='C')
     .add_device('touch_sensor', device_type=TouchSensor, port='1')
     .add_device('color_sensor', device_type=ColorSensor, port='2'))

def setup():
    global g_video, g_video_thread
    g_video = Video()

    print(" -- starting VideoThread...")
    g_video_thread = VideoThread()
    g_video_thread.start()

def cleanup():
    print(" -- stopping VideoThread...")
    g_video_thread.stop()
    g_video_thread.join()

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

    setup()

    #py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = build_behaviour_tree()
    display_tree.render_dot_tree(tree)

    signal.signal(signal.SIGTERM, sig_handler)

    try:
        etrobo = initialize_etrobo(backend='raspike')
        etrobo.add_handler(ExposeDevices())
        etrobo.add_handler(Plotter())
        etrobo.add_handler(TraverseBehaviourTree(tree))
        etrobo.dispatch(interval=INTERVAL, port=args.port, logfile=args.logfile)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cleanup()
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- exiting...")
