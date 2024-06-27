from pydantic import BaseModel, Field, ConfigDict, model_validator
from typing import Iterable, Callable, ClassVar, Self
from .utils.math import Deg


class BaseSettings(BaseModel):
    model_config = ConfigDict(strict=True)
    __listeners__: ClassVar[list[Callable[[Self], None]]] = []
    __singleton__: ClassVar[Self]

    @classmethod
    def subscribe_updates(cls, listener: Callable[[Self], None]) -> Self:
        cls.__listeners__.append(listener)
        return cls.__singleton__

    @classmethod
    def _get_listeners(cls):
        return cls.__listeners__

    @classmethod
    def _set_singleton(cls, instance: Self):
        cls.__singleton__ = instance

    def _update(self):
        self._set_singleton(self)
        for listener in self._get_listeners():
            listener(self)

    @classmethod
    def get(cls) -> Self:
        return cls.__singleton__


class PIDSettings(BaseSettings):
    on: bool
    Kp: float = Field(gt=0)
    Kd: float = Field(ge=0)
    Ki: float = Field(ge=0)
    # unit: s-1. Controls the speed at which the the integral fades over time when the error is zero.
    # a value of 0 means no fade.
    # a value of 1 means a decrease of 1/2 to 1/e=1/2.718 over one second. (faster when dt is smaller. 0.39 for dt = 0.1)
    integral_fade_rate: float = Field(ge=0)
    max_integral_value: float = Field(ge=1)
    # rad/s
    max_rot_x: float = Field(gt=0)
    max_rot_y: float = Field(gt=0)
    max_rot_z: float = Field(gt=0)


class PilotSettings(BaseSettings):
    # minimum vertical thrust to maintain the ROV underwater
    z_eps_thrust: float = Field(ge=0, lt=1)
    # efficiency of propellers when rotated backward
    reverse_efficiency: float = Field(gt=0, le=1)
    # coef applied to commands in bridled mode
    bridle_coef: float = Field(gt=0, lt=1)
    # ratio applied to cy for front engines
    cy_engines_ratio: float = Field(gt=0, le=1)
    # angle threshold to switch to forward thrust
    forward_threshold: Deg = Field(ge=90, lt=180)
    # angle threshold to switch to reverse thrust (hysteresis)
    reverse_threshold: Deg = Field(ge=90, lt=180)

    @model_validator(mode="after")
    def _validate_hysteresis(self) -> Self:
        if self.forward_threshold >= self.reverse_threshold:
            raise ValueError("forward threshold must be lower than reverse threshold")
        return self


class Settings(BaseSettings):
    pid: PIDSettings

    pilot: PilotSettings

    @property
    def _subsets(self) -> Iterable[BaseSettings]:
        return (self.pid, self.pilot)

    @classmethod
    def apply_settings(cls, new_settings: "Settings"):
        # new_settings = cls.model_validate_json(new_settings_json)

        new_settings.apply()

    def apply(self):
        for subset in self._subsets:
            subset._update()

        self._update()


# create initial value
Settings(
    pid=PIDSettings(
        on=True,
        Kp=1,
        Kd=0,
        Ki=0,
        integral_fade_rate=0.1,
        max_integral_value=10,
        max_rot_x=3,
        max_rot_y=2,
        max_rot_z=1,
    ),
    pilot=PilotSettings(
        z_eps_thrust=0.2,
        reverse_efficiency=0.9,
        bridle_coef=0.5,
        cy_engines_ratio=0.6,
        forward_threshold=Deg(130),
        reverse_threshold=Deg(140),
    ),
).apply()
