from dataclasses import dataclass


@dataclass
class GamePadSticks:
    leftX: int
    leftY: int
    rightX: int
    rightY: int
    padX: int
    padY: int


@dataclass
class GamePadButtons:
    A: bool
    B: bool
    X: bool
    Y: bool
    L: bool
    L2: bool
    R: bool
    R2: bool
    stickL: bool
    stickR: bool


@dataclass
class GamePad:
    connected: bool
    sticks: GamePadSticks
    buttons: GamePadButtons
    tm_ms: int


class MotorState:
    on: float
    angle: float
    thrust: float


class Feedbacks:
    surfaced: bool
    battery_charge: float
    ext_pressure: float
    left_mot: MotorState
    right_mot: MotorState
    tail_mot: MotorState
    flood_ar: bool
    flood_av: bool
    pressure_in: float
    current_mot: float
    current_electronics: float
    temp_in: bool
    bridled_mode: bool  # P mot plafonnée
