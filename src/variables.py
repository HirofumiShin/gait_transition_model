from .config import *
import numpy as np

class StateDim:
    def __init__(self, x=0.0, y=0.0, 
                 dx=0.0, dy=0.0, ddx=0.0, ddy=0.0, 
                 ddx_damper=0.0, ddy_damper=0.0):
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.ddx = ddx
        self.ddy = ddy
        # other
        self.ddx_damper = ddx_damper
        self.ddy_damper = ddy_damper

    def clone(self):
        # value copy
        return StateDim(self.x, self.y, self.dx, self.dy, self.ddx, self.ddy)

class Position:
    def __init__(self):
        a = 90.0
        x = np.cos(np.pi / 180 * a)
        y = np.sin(np.pi / 180 * a) + 0.02
        dx = target_dx
        dy = 0.0
        ddx = 0
        ddy = 0
        leg_len = 0.0

        self.positions = {
            LEG.FORE: StateDim(y=0.1),  # fore leg
            LEG.REAR: StateDim(y=0.1),  # rear leg
            LEG.COM: StateDim(x=x, y=y,
                              dx=dx, dy=dy,
                              ddx=ddx, ddy=y), # center of mass 
            LEG.RIGHT: StateDim(), # right leg
            LEG.LEFT: StateDim(), # left leg
            LEG.V_SWING: StateDim(x=0, y=0), # virtual swing leg
            LEG.V_STANCE: StateDim(), # virtual stance leg
            LEG.PRO_FORE: StateDim(x=0, y=0) # provisional fore leg
        }

    def __getitem__(self, leg):
        return self.positions.get(leg)

    def __setitem__(self, leg, position):
        if leg in self.positions:
            self.positions[leg] = position
        else:
            raise ValueError("Invalid leg position")


class StateTransition:
    def __init__(self):
        self.TsVSL, self.TsVSLbuff = 0.1, 0.0
        self.pre_event = EVENT.CONTACT
        self.cur_event =  EVENT.LEAVE


class DataLog:
    def __init__(self):
        self.supplied_energy = 0.0
        self.cnt_step_for_energy_analysis = 1
        self.supplied_energy_integral = 0
        self.total_energy_during_one_target_dx = 0


class TimingFlag:
    def __init__(self):
        self.does_get_leave_or_contact_event = False
        self.is_change_speed = False
        self.is_transition_true = True


