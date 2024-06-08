from _typeshed import Incomplete

ALIGNMENT_NXNYPZ: int
ALIGNMENT_NXNZNY: int
ALIGNMENT_NXPYNZ: int
ALIGNMENT_NXPZPY: int
ALIGNMENT_NYNXNZ: int
ALIGNMENT_NYNZPX: int
ALIGNMENT_NYPXPZ: int
ALIGNMENT_NYPZNX: int
ALIGNMENT_NZNXPY: int
ALIGNMENT_NZNYNX: int
ALIGNMENT_NZPXNY: int
ALIGNMENT_NZPYPX: int
ALIGNMENT_PXNYNZ: int
ALIGNMENT_PXNZPY: int
ALIGNMENT_PXPYPZ: int
ALIGNMENT_PXPZNY: int
ALIGNMENT_PYNXPZ: int
ALIGNMENT_PYNZNX: int
ALIGNMENT_PYPXNZ: int
ALIGNMENT_PYPZPX: int
ALIGNMENT_PZNXNY: int
ALIGNMENT_PZNYPX: int
ALIGNMENT_PZPXPY: int
ALIGNMENT_PZPYNX: int
CONVENTION_ENU: int
CONVENTION_NED: int
CONVENTION_NWU: int

class Ahrs:
    earth_acceleration: Incomplete
    flags: Incomplete
    heading: Incomplete
    internal_states: Incomplete
    linear_acceleration: Incomplete
    quaternion: Incomplete
    settings: Incomplete
    @classmethod
    def __init__(cls, *args, **kwargs) -> None: ...
    def reset(self, *args, **kwargs): ...
    def update(self, *args, **kwargs): ...
    def update_external_heading(self, *args, **kwargs): ...
    def update_no_magnetometer(self, *args, **kwargs): ...

class Flags:
    acceleration_recovery: Incomplete
    angular_rate_recovery: Incomplete
    initialising: Incomplete
    magnetic_recovery: Incomplete

class InternalStates:
    acceleration_error: Incomplete
    acceleration_recovery_trigger: Incomplete
    accelerometer_ignored: Incomplete
    magnetic_error: Incomplete
    magnetic_recovery_trigger: Incomplete
    magnetometer_ignored: Incomplete

class Offset:
    @classmethod
    def __init__(cls, *args, **kwargs) -> None: ...
    def update(self, *args, **kwargs): ...

class Quaternion:
    w: Incomplete
    wxyz: Incomplete
    x: Incomplete
    y: Incomplete
    z: Incomplete
    @classmethod
    def __init__(cls, *args, **kwargs) -> None: ...
    def to_euler(self, *args, **kwargs): ...
    def to_matrix(self, *args, **kwargs): ...

class Settings:
    acceleration_rejection: Incomplete
    convention: Incomplete
    gain: Incomplete
    gyroscope_range: Incomplete
    magnetic_rejection: Incomplete
    recovery_trigger_period: Incomplete
    @classmethod
    def __init__(cls, *args, **kwargs) -> None: ...

def axes_swap(*args, **kwargs): ...
def compass_calculate_heading(*args, **kwargs): ...
