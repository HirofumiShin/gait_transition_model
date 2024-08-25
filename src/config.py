import numpy as np
from enum import Enum, auto

class PHASE(Enum):
    SINGLE_SUPPORT = auto()
    DOUBLE_SUPPORT = auto()
    JUMPING = auto()
    SUPPORT = auto()

class EVENT(Enum):
    LEAVE = auto()
    CONTACT = auto()

class LEG(Enum):
    FORE = auto()
    REAR = auto()
    COM = auto()
    RIGHT = auto()
    LEFT = auto()
    V_SWING = auto()
    V_STANCE = auto()
    PRO_SWING = auto()
    PRO_STANCE = auto()
    PRO_FORE = auto()

l0 = 1.0

simulation_scenario = {
    "does_animation_plot": False,
    "is_one_target_data": False,
    "is_transition_movie": False,
}

target_value = {
    "begin_dx": 0.5,
    "end_dx": 3.0,
    "target_height": 0.99,
    "duration_time_for_one_target": 2.0,
    "change_target_dx_every": 0.01
}

target_dx = target_value["begin_dx"]
# initial_params = 
gains = {
    "p_kick": 0.0001,
    "i_kick": 0.0000001,
    "p_swing": 0.1,
}

params = {
    "leg_len_limit": 1.0,
    "natural_len_fore": l0,
    "natural_len_rear": l0,
    "spring_const": 50000, 
    "damper_const": 1000,
    "mass": 80.0,
    "alpha0": 76 * (np.pi / 180),
    "gravity": 9.81,
    "dt": 0.0001

}

count = {
    "every_for_animation": 100,
    "every_for_csv_save": 25,
    "end_time_for_one_target_data": 2
}
