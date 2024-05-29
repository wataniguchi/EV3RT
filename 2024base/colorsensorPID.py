import argparse
from etrobo_python import ColorSensor, ETRobo, Hub, Motor, TouchSensor
from simple_pid import PID

class LineTracer(object):
    def __init__(self, interval: float, target: int, power: int, pid_p: float, pid_i: float, pid_d: float) -> None:
        self.running = False
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=interval, output_limits=(-power, power))

    def __call__(
        self,
        hub: Hub,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
    ) -> None:
        if not self.running and (
                touch_sensor.is_pressed()
                or hub.is_left_button_pressed()
                or hub.is_right_button_pressed()):
            self.running = True

        if not self.running:
            return

        turn = int(self.pid(color_sensor.get_brightness()))

        right_motor.set_power(self.power - turn)
        left_motor.set_power(self.power + turn)


def run(backend: str, interval: float, target: int, power: int, pid_p: float, pid_i: float, pid_d: float, **kwargs) -> None:
    (ETRobo(backend=backend)
     .add_hub('hub')
     .add_device('right_motor', device_type=Motor, port='B')
     .add_device('left_motor', device_type=Motor, port='C')
     .add_device('touch_sensor', device_type=TouchSensor, port='1')
     .add_device('color_sensor', device_type=ColorSensor, port='2')
     .add_handler(LineTracer(interval, target, power, pid_p, pid_i, pid_d))
     .dispatch(interval, **kwargs))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyAMA1', help='Serial port.')
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()
    run(backend='raspike', port=args.port, interval=0.04, target=60, power=35, pid_p=0.56, pid_i=0.005, pid_d=0.015, logfile=args.logfile)
