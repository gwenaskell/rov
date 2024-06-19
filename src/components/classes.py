from dataclasses import dataclass
from enum import Enum
from typing import TypedDict
from math import atan2, asin, pi
from time import time

@dataclass
class Commands:
    fx: float  # forward thrust
    fz: float  # vertical thrust
    cx: float  # roll
    cy: float  # pitch
    cz: float  # yaw
    tm_ms: int
    surface: bool = True


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


class Status(str, Enum):
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


@dataclass
class Measurements:
    depth_m: float = 0

    v_batt: float = 0
    i_mots: float = 0

    flood_av: bool = False
    flood_ar: bool = False

    in_water: bool = False
    immerged: bool = False

@dataclass
class Feedbacks:
    """feedbacks to the user"""
    measurements: Measurements = Measurements()

    iter_ms: int = 0

    bridled: bool = False

    status: Status = Status.STOPPED

@dataclass
class Euler:
    x: float
    y: float
    z: float
    
def safe_asin(value):
    if value <= -1.0:
        return pi / -2.0
    
    if value >= 1.0:
        return  pi / 2.0
    
    return asin(value)
    
@dataclass
class Quaternion:
    w: float = 1
    x: float = 0
    y: float = 0
    z: float = 0

    def mul(self, q2: "Quaternion") -> "Quaternion":
        """return the product of the two quaternions"""

        q1 = self
        return Quaternion(
            q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
            q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
            q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
            q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
        )

    def conj(self) -> "Quaternion":
        """return the conjugate of the quaternion"""
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def convert_coordinates(self, v: "Vec") -> "Vec":
        """convert the vector coordinates in the referential represented
        by the quaternion using the formula q' * v * q. It is the opposite of rotate"""
        # Pure quaternion from vector
        v_q = Quaternion(0, v.x, v.y, v.z)

        # compute q' * v * q
        v_q_prime = self.conj().mul(v_q).mul(self)

        # Extract rotated vector
        return Vec(v_q_prime.x, v_q_prime.y, v_q_prime.z)

    def rotate(self, v: "Vec") -> "Vec":
        """rotate the vector using the formula q * v * q'"""
        # Pure quaternion from vector
        v_q = Quaternion(0, v.x, v.y, v.z)

        # compute q * v * q'
        v_q_prime = self.mul(v_q).mul(self.conj())

        # Extract rotated vector
        return Vec(v_q_prime.x, v_q_prime.y, v_q_prime.z)

    def to_euler(self) -> Euler:
        """return the ZYX euler angles rotation corresponding to the quaternion"""
        
        half_min_y2 = 0.5 - self.y**2
        
        return Euler(
            x=atan2(self.w*self.x+self.y*self.z, half_min_y2-self.x**2),
            y=safe_asin(2.0*(self.w*self.y-self.z*self.x)),
            z=atan2(self.w*self.z+self.x*self.y, half_min_y2-self.z**2)
        )

    def to_matrix(self):
        """return the corresponding rotation matrix"""
        ww = self.w**2
        wx = self.w*self.x
        wy = self.w*self.y
        wz = self.w*self.z
        xy = self.x*self.y
        yz = self.y*self.z
        xz = self.x*self.z
        
        return [
            [
                2.0 * (ww-0.5*self.x**2),
                2.0 * (xy-wz),
                2.0 * (xz+wy),
            ],
            [
                2.0 * (xy+wz),
                2.0 * (ww-0.5*self.y**2),
                2.0 * (yz-wx),
            ],
            [
                2.0 * (xz-wy),
                2.0 * (yz+wx),
                2.0 * (ww-0.5*self.z**2),
            ]
        ]

@dataclass
class Vec:
    """vector"""
    x: float = 0
    y: float = 0
    z: float = 0
    
    def __add__(self, v: "Vec"):
        return Vec(
            x=self.x+v.x,
            y=self.y+v.y,
            z=self.z+v.z
        )
    
    def __sub__(self, v: "Vec"):
        return Vec(
            x=self.x-v.x,
            y=self.y-v.y,
            z=self.z-v.z
        )
    
    def __mul__(self, c: float):
        return Vec(
            x=self.x*c,
            y=self.y*c,
            z=self.z*c
        )

    def __rmul__(self, c: float):
        return self.__mul__(c)

    def __truediv__(self, c: float):
        return Vec(
            x=self.x/c,
            y=self.y/c,
            z=self.z/c
        )

    def __getitem__(self, i: int) -> float:
        if i == 0:
            return self.x
        if i == 1:
            return self.y
        if i == 2:
            return self.z
        raise IndexError(i)

    def __setitem__(self, i: int, val: float):
        if i == 0:
            self.x = val
        elif i == 1:
            self.y = val
        elif i == 2:
            self.z = val
        else:
            return IndexError(i)

@dataclass
class RovState:
    # speed
    # vx: float
    # vy: float
    # vz: float
    # acceleration
    ax: float = 0
    ay: float = 0
    az: float = 0
    # quaternion
    q: Quaternion = Quaternion()
    # angular speed
    wx: float = 0
    wy: float = 0 
    wz: float = 0
    time: float = 0
    
    def __post_init__(self):
        if self.time == 0:
            self.time = time()

