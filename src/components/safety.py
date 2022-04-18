from src.components.classes import Measurements


class Safety:
    VBATT_THRESHOLD = 19
    IMOT_THRESHOLD = 30

    @classmethod
    def must_bridle(cls, ms: Measurements) -> bool:
        return ms.v_batt < cls.VBATT_THRESHOLD or ms.i_mots > cls.IMOT_THRESHOLD
