import numpy as np
from .config import *
from .variables import *

k, c, l0 = params["spring_const"], params["damper_const"], params["natural_len_rear"]
m, g, dt = params["mass"], params["gravity"], params["dt"]

class EquationOfMotion:
    def __init__(self):
        self.is_xl, self.is_dxl = 0, 1
        self.phaseFLAG = PHASE.JUMPING
        self.spring_cross = 0.00
        self.dump_cross = 0.05
        self.l_mass_limit = params["leg_len_limit"]
        self.rear_spring_force = 0

    def velocity_verlet(self, pre_xl, pre_dxl, pre_ddxl, ddxl, xl_or_dxl):
        if xl_or_dxl == self.is_xl:
            return pre_xl + pre_dxl * dt + 1.0 / 2.0 * pre_ddxl * (dt**2)
        elif xl_or_dxl == self.is_dxl:
            return pre_dxl + 1.0 / 2.0 * (ddxl + pre_ddxl) * dt

    def computeActuatedSpringValueInAxis(self, xl, yl, x_rear_FP, x_fore_FP, motor_pos_rear, motor_pos_fore):
        if np.sqrt(pow(xl - x_rear_FP, 2) + pow(yl, 2)) > self.l_mass_limit-self.spring_cross:
            kr = 0
        else:
            kr = k
        if np.sqrt(pow(xl - x_fore_FP, 2) + pow(yl, 2)) > self.l_mass_limit-self.spring_cross:
            kf = 0
        else:
            kf = k
        
        KRl = kr * ( (self.l_mass_limit+motor_pos_rear) / ((xl - x_rear_FP) **
                                   2 + yl**2)**(1.0 / 2.0) - 1.0)
        KFl = kf * ( (self.l_mass_limit+motor_pos_fore) / ((xl - x_fore_FP)
                                   ** 2 + yl**2)**(1.0 / 2.0) - 1.0)
        return max(KRl, 0), max(KFl, 0)

    def computeDumperValueInAxis(self, xl, yl, x_rear_FP, x_fore_FP, dxl, dyl):
        CRl = c * ((xl - x_rear_FP) * dxl + yl * dyl) / \
            np.sqrt(pow(xl - x_rear_FP, 2) + pow(yl, 2))
        CFl = c * ((xl - x_fore_FP) * dxl + yl * dyl) / \
            np.sqrt(pow(xl - x_fore_FP, 2) + pow(yl, 2))
        return CRl, CFl
    
    def computeSpringEnergy(self, pos, pos_rear, pos_fore, motor_pos_rear):
        state = self.phaseFLAG
        xl, yl, x_rear_FP, x_fore_FP, dxl, dyl = pos.x, pos.y, pos_rear.x, pos_fore.x, pos.dx, pos.dy
        if np.sqrt(pow(xl - x_rear_FP, 2) + pow(yl, 2)) > self.l_mass_limit-self.spring_cross:
            kr = 0
        else:
            kr = k
        if np.sqrt(pow(xl - x_fore_FP, 2) + pow(yl, 2)) > self.l_mass_limit-self.spring_cross:
            kf = 0
        else:
            kf = k
        kr = kr if state == PHASE.SINGLE_SUPPORT else kr if state == PHASE.DOUBLE_SUPPORT else 0
        kf = 0 if state == PHASE.SINGLE_SUPPORT else kf if state == PHASE.DOUBLE_SUPPORT else 0

        KREl = 1.0 / 2.0 * kr * \
            pow(self.l_mass_limit-self.spring_cross+motor_pos_rear - np.sqrt(pow(xl - x_rear_FP, 2) + pow(yl, 2)), 2)
        KFEl = 1.0 / 2.0 * kf * \
            pow(self.l_mass_limit-self.spring_cross - np.sqrt(pow(xl - x_fore_FP, 2) + pow(yl, 2)), 2)
        return max(KREl, 0), max(KFEl, 0)

    def EOM_SLIP(self, xl, yl, x_rear_FP, x_fore_FP, dxl, dyl, state, motor_pos_rear, motor_pos_fore):  # return ddx and ddy
        KRl, KFl = self.computeActuatedSpringValueInAxis(
            xl, yl, x_rear_FP, x_fore_FP, motor_pos_rear, motor_pos_fore)
        CRl, CFl = self.computeDumperValueInAxis(
            xl, yl, x_rear_FP, x_fore_FP, dxl, dyl)
        if np.sqrt(pow(xl - x_rear_FP, 2) + pow(yl, 2)) < self.l_mass_limit - self.dump_cross:
            CRl = 0
        elif np.sqrt(pow(xl - x_rear_FP, 2) + pow(yl, 2)) > 1.0:
            CRl = 0
        if np.sqrt(pow(xl - x_fore_FP, 2) + pow(yl, 2)) < self.l_mass_limit - self.dump_cross:
            CFl = 0
        elif np.sqrt(pow(xl - x_fore_FP, 2) + pow(yl, 2)) > 1.0:
            CFl = 0

        if state == PHASE.SINGLE_SUPPORT:
            KRl = KRl
            KFl = 0
            CRl = CRl
            CFl = 0
        elif state == PHASE.DOUBLE_SUPPORT:
            KRl = KRl
            KFl = KFl
            CRl = CRl
            CFl = CFl
        else:
            KRl = 0
            KFl = 0
            CRl = 0
            CFl = 0

        self.rear_spring_force = KRl

        legLength_rear = np.sqrt(pow(xl - x_rear_FP, 2) + pow(yl, 2))
        legLength_support = np.sqrt(pow(xl - x_fore_FP, 2) + pow(yl, 2))

        ddx_spring = (+KRl * (xl - x_rear_FP) / m
                      + KFl * (xl - x_fore_FP) / m)
        ddx_damp = (-CRl * (xl - x_rear_FP)/legLength_rear / m
                    - CFl * (xl - x_fore_FP)/legLength_support / m)
        
        ddy_spring = (+KRl * yl / m
                      + KFl * yl / m)
        ddy_damp = (-CRl * (yl) /legLength_rear/ m
                    - CFl * (yl) /legLength_support/ m)

        ret_ddx = ddx_spring + ddx_damp 
        ret_ddy = ddy_spring + ddy_damp  - g

        return ret_ddx, ret_ddy,CRl, CFl, ddx_damp, ddy_damp

    def simulate(self, pos, pos_rear, pos_fore, motor_pos_rear, motor_pos_fore):
        self.update_phase_for_eom(pos_rear, pos_fore)
        phase_state = self.phaseFLAG
        state = phase_state, motor_pos_rear, motor_pos_fore
        x_rear_FP, x_fore_FP = pos_rear.x, pos_fore.x
        pre_xl, pre_dxl, pre_ddxl = pos.x, pos.dx, pos.ddx
        pre_yl, pre_dyl, pre_ddyl = pos.y, pos.dy, pos.ddy
        # calculation EOM
        ddxl, ddyl, CRl, CFl,ddx_damp, ddy_damp = self.EOM_SLIP(
            pre_xl, pre_yl, x_rear_FP, x_fore_FP, pre_dxl, pre_dyl, phase_state, motor_pos_rear, motor_pos_fore)
        dxl = pre_dxl + ddxl * dt
        dyl = pre_dyl + ddyl * dt
        xl = pre_xl + dxl * dt
        yl = pre_yl + dyl * dt
        return (StateDim(x=xl, dx=dxl, ddx=ddxl, y=yl, dy=dyl, ddy=ddyl ,ddx_damper=ddx_damp, ddy_damper=ddy_damp))

    def update_phase_for_eom(self, pos_rear, pos_fore):
        if (pos_rear.y <= 0.0) and (pos_fore.y <= 0.0):
            self.phaseFLAG = PHASE.DOUBLE_SUPPORT
        elif (pos_rear.y > 0.0) and (pos_fore.y > 0.0):
            self.phaseFLAG = PHASE.JUMPING
        else:
            self.phaseFLAG =  PHASE.SINGLE_SUPPORT 
