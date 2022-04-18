from dataclasses import dataclass


@dataclass
class GamePadSticks:
    leftX: int = 0
    leftY: int = 0
    rightX: int = 0
    rightY: int = 0
    padX: int = 0
    padY: int = 0


@dataclass
class GamePadButtons:
    A: bool = False
    B: bool = False
    X: bool = False
    Y: bool = False
    L: bool = False
    L2: bool = False
    R: bool = False
    R2: bool = False
    stickL: bool = False
    stickR: bool = False


@dataclass
class GamePad:
    connected: bool
    sticks: GamePadSticks = GamePadSticks()
    buttons: GamePadButtons = GamePadButtons()
    tm_ms: int = 0


class MotorState:
    on: float
    angle: float
    thrust: float
