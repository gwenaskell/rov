from dataclasses import dataclass
from enum import Enum
from typing import TypedDict


@dataclass
class Commands:
    fx: float  # forward thrust
    fz: float  # vertical thrust
    cx: float  # roll
    cy: float  # pitch
    cz: float  # yaw
    tm_ms: int


@dataclass
class ThrusterVector:  # thrust vector of a front (left/right) thruster
    f_x: float  # thrust on x axis
    f_z: float  # thrust on z axis


@dataclass
class ThrusterState:  # state of a front (left/right) thruster
    tau: float  # engine thrust
    pos: int  # stepper position
    # usually 1 or 0. Used when thrust of all thrusters must be disabled or reduced during a rotation
    thrust_coef: float = 1.0


class Status(Enum):
    STOPPED = "stopped"
    PAUSED = "paused"
    RUNNING = "running"


class TargetsProxy(TypedDict):
    left_state: ThrusterState
    right_state: ThrusterState
    tail_thrust: float


class NamespaceProxy(TypedDict):
    status: Status
    targets: TargetsProxy


class Measurements:
    depth_m: float = 0

    v_batt: float = 0
    i_mots: float = 0

    flood_av: bool = False
    flood_ar: bool = False

    in_water: bool = False
    immerged: bool = False


class Feedbacks:
    measurements: Measurements = Measurements()

    iter_ms: int = 0

    bridled: bool = False

    status: Status = Status.STOPPED
