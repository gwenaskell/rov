from src.components.classes import Measurements
from .accessors import AnalogChans, AnalogReader
from .accessors import Arduino

PERC_THRESHOLD = 10

_perc_to_amps = 35/100


class Sensors:
    def __init__(self) -> None:
        self.analog = AnalogReader()
        self.measurements = Measurements()
        # arduino

    def start(self):
        pass

    async def read_sensors(self) -> Measurements:
        self.measurements.in_water = self.analog.get_perc_val(
            AnalogChans.SURFACE_LOW) > PERC_THRESHOLD
        self.measurements.immerged = self.analog.get_perc_val(
            AnalogChans.SURFACE_UP) > PERC_THRESHOLD

        self.measurements.v_batt = self.analog.get_perc_val(
            AnalogChans.VBATT_0)

        self.measurements.i_mots = self.analog.get_perc_val(
            AnalogChans.CURRENT_MOT) * _perc_to_amps

        # formula: v_out/v_cc = (2/3*P_mpa + 0.1)
        self.measurements.depth_m = (self.analog.get_perc_val(
            AnalogChans.EXT_PRESSURE)-10)*0.15*10

        self.measurements.flood_ar = self.analog.get_perc_val(
            AnalogChans.WATER_AR) > PERC_THRESHOLD
        self.measurements.flood_av = self.analog.get_perc_val(
            AnalogChans.WATER_AV) > PERC_THRESHOLD

        return self.measurements
