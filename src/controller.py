from .utils import *
from .config import *
from .variables import *

class Controller:
    def __init__(self, target_dx):
        # foot placement control
        self.i_value_fp = 0

        # energy values status
        self.mechanical_energy = 0
        self.energy_error_integral = 0
        # kick control
        self.motor_pos = {
            LEG.FORE: 0.0,
            LEG.REAR: 0.0,
        }
        self.pre_motor_pos_rear = 0
        self.actuator_len_vel = 0
        self.tar_mechanical_energy = 0
        
        self.tar_len_vl = 0.96
        self.dx_on_peak_pos = target_dx
        self.virtual_leg_phase = PHASE.JUMPING
        self.peak_height = 0.0
        self.leg_ang = 0 # up-light is 90 deg

    def leg_len_limitation(self, pos_ref, _buff_, ret_pos):
        ## leg length limitation
        if (pos_ref[LEG.COM].x - _buff_.x)**2 + (pos_ref[LEG.COM].y - _buff_.y)**2 > 1.0:
            # set to leg len is 1.0
            ret_pos = StateDim(
                x=pos_ref[LEG.COM].x + (pos_ref[LEG.V_SWING].x-pos_ref[LEG.COM].x)/self.tar_len_vl,
                y=pos_ref[LEG.COM].y + (pos_ref[LEG.V_SWING].y-pos_ref[LEG.COM].y)/self.tar_len_vl)
        return ret_pos

    def forward_speed_controller(self, pos_ref, target_dx, virtual_leg_phase, TsVSL, dx_on_peak_pos):
        neutral_point_x = pos_ref[LEG.COM].x + ((dx_on_peak_pos) * TsVSL / 2.0)
        pos_ref[LEG.V_SWING].x = neutral_point_x - gains["p_swing"] * (target_dx - dx_on_peak_pos)
        ## limitation
        if pos_ref[LEG.V_SWING].x <= pos_ref[LEG.COM].x - 0.05:
            pos_ref[LEG.V_SWING].x = pos_ref[LEG.COM].x - 0.05
        ##
        pos_ref[LEG.V_SWING] = calc_y_pos_from_len(pos_ref[LEG.V_SWING].x, pos_ref[LEG.COM], self.tar_len_vl)
        _buff_ = calc_foot_pos_from_mid_point(
            pos_ref[LEG.V_SWING], pos_ref[LEG.REAR].x, pos_ref[LEG.REAR].y)
        if virtual_leg_phase == PHASE.SUPPORT:
            pos_ref[LEG.PRO_FORE] = _buff_
            if not ((pos_ref[LEG.FORE].y <= 0) and (pos_ref[LEG.REAR].y <= 0)):
                pos_ref[LEG.FORE] = _buff_
        elif virtual_leg_phase == PHASE.JUMPING:
            pos_ref[LEG.FORE] = _buff_
            if pos_ref[LEG.REAR].y > 0.0:
                pos_ref[LEG.REAR] = pos_ref[LEG.COM].clone()
                pos_ref[LEG.FORE] = self.leg_len_limitation(
                    pos_ref, _buff_, pos_ref[LEG.FORE])
            if (pos_ref[LEG.FORE].y < -0.00) and (pos_ref[LEG.COM].dy <= 0) and (pos_ref[LEG.COM].ddy <= 0):
                pos_ref[LEG.FORE].y = 0.0
        pos_ref[LEG.REAR].len_len = np.sqrt(
            pow(pos_ref[LEG.COM].x - pos_ref[LEG.REAR].x, 2) + pow(pos_ref[LEG.COM].y-pos_ref[LEG.REAR].y, 2))        
        pos_ref[LEG.V_SWING].y = (0.0 + pos_ref[LEG.FORE].y)/2.0 
        return pos_ref[LEG.FORE], pos_ref[LEG.REAR], pos_ref[LEG.PRO_FORE], pos_ref[LEG.V_SWING]
    
    def jump_height_controller(self, motor_pos_rear, leg_ang,  
                               tar_mechanical_energy, mechanical_energy):
        error = tar_mechanical_energy  - mechanical_energy
        self.energy_error_integral = 0
        if (leg_ang > 90.0) : 
            self.energy_error_integral += error if error > 0 else 0
            motor_pos_rear = gains["p_kick"] * \
                (error) + gains["i_kick"] * \
                self.energy_error_integral/params["dt"]
            if motor_pos_rear <= 0:
                motor_pos_rear = 0
            #####
            self.actuator_len_vel = (
                motor_pos_rear-self.pre_motor_pos_rear)/params["dt"]
            self.pre_motor_pos_rear = motor_pos_rear
            if self.actuator_len_vel < 0:
                motor_pos_rear = self.pre_motor_pos_rear
                self.actuator_len_vel = 0  
                
        return motor_pos_rear

    def calc_support_leg_ang(self, pos_com, pos_sup):
        return np.arctan2(
            pos_com.y, pos_sup.x - pos_com.x) * 180 / np.pi
