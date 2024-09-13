from enum import Enum, IntEnum, auto


class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1


class JState(Enum):
    INITIAL = auto()
    JOINING = auto()
    JOINED = auto()
    FORKING = auto()
    FORKED = auto()


class Wheel:
    TIRE_DIAMETER: float = 100.0
    WHEEL_TREAD: float = 120.0


class TraceNum:
    PID_P_FAST: float = 1.1
    PID_I_FAST: float = 0.001
    PID_D_FAST: float = 0.0
    POWER_FAST: int = 65

    PID_P_SLOW: float = 1.4
    PID_I_SLOW: float = 0.0012
    PID_D_SLOW: float = 0.3
    POWER_SLOW: int = 45

    PID_P_CIRCLE: float = 1.45
    PID_I_CIRCLE: float = 0.0011
    PID_D_CIRCLE: float = 0.2
    POWER_CIRCLE: int = 45

    PID_P_ELLIPSE: float = 1.5
    PID_I_ELLIPSE: float = 0.0014
    PID_D_ELLIPSE: float = 0.3
    POWER_ELLIPSE: int = 45

    POWER_JUNCTION: int = 37


class DebriNum:
    BOTTLE_RANGE_LOWER = 8700
    DISTANCE_SIDE = 1100
    DISTANCE_VERTICAL = 1900
    DISTANCE_CATCH = 120
    DISTANCE_REMOVE = 300
    DISTANCE_PASSED_THIRD = 1500
    POWER_REMOVE_RIGHT = 60
    POWER_REMOVE_LEFT = 15

    PID_P: float = 1.7
    PID_I: float = 0.0015
    PID_D: float = 0.1
    POWER_NORMAL: int = 40
    POWER_SLOW: int = 33


class TraceSide(Enum):
    NORMAL = "Normal"
    OPPOSITE = "Opposite"
    RIGHT = "Right"
    LEFT = "Left"
    CENTER = "Center"

class Scene(Enum):
    LOOP = auto()
    DEBRI = auto()

