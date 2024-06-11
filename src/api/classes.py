from dataclasses import dataclass


@dataclass
class GamePadSticks:
    leftX: int = 0   # cx or -fz if R is pressed
    leftY: int = 0   # -cz
    rightX: int = 0  # cy
    rightY: int = 0  # cx
    padX: int = 0
    padY: int = 0


@dataclass
class GamePadButtons:
    A: bool = False
    B: bool = False
    X: bool = False
    Y: bool = False
    L: bool = False   # surface mode
    L2: bool = False  # bridled mode
    R: bool = False   # switch cx to fz
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
