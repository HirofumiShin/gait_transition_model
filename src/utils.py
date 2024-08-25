from .config import *
import sys
import argparse
from .equation_of_motion import PHASE
from .variables import *
# function def ----------------------------------------------
#-------------------------------------------------------------------
sign = 1
returned_time = 0
def queue(src, a):
    dst = np.roll(src, -1)
    dst[-1] = a
    return dst

def calc_mechanical_energy(y, d_pos, spring_energy_rear_leg = 0, spring_energy_fore_leg = 0):
    return params["mass"] * params["gravity"] * y + 0.5 * params["mass"] * (d_pos.dx * d_pos.dx + d_pos.dy * d_pos.dy) + spring_energy_rear_leg + spring_energy_fore_leg

def calc_foot_pos_from_mid_point(mid_pos, rear_x, rear_y=0.0):
    fore_x = (mid_pos.x - rear_x) * 2 + rear_x
    fore_y = (mid_pos.y - rear_y) * 2 + rear_y
    return StateDim(x=fore_x, y=fore_y) 

def calc_y_pos_from_len(tar_x, pos, len):
    tar_y = pos.y - (np.sqrt(pow(len, 2) - pow(pos.x - tar_x, 2)))
    return StateDim(x=tar_x, y=tar_y)

def calc_x_pos_from_len(tar_y, pos, len):
    tar_x = pos.x + (np.sqrt(pow(len, 2) - pow(pos.y - tar_y, 2)))
    return StateDim(x=tar_x, y=tar_y)

def write_to_csv(simulation_scenario_ref, target_value_ref, target_dx):
    if simulation_scenario_ref["is_transition_movie"] == True:
        file_name = './data/gait_transition_for_movie.csv'
        open(file_name, 'w+')
        print('gait transition')
    elif simulation_scenario_ref["is_one_target_data"] == True:
        file_name = './data/target_dx_'+str(target_dx)+'.csv'
        open(file_name, 'w+')
        print('getting data which target vale is ' +str(target_dx) )
    elif target_value_ref["begin_dx"] > target_value_ref["end_dx"]:
        file_name = './data/gait_transition_from_high_to_low.csv'
        open(file_name, 'w+')
        print('gait transition start from ' +
              str(target_value_ref["begin_dx"]) + ' to ' + str(target_value_ref["end_dx"]))
    elif target_value_ref["begin_dx"] < target_value_ref["end_dx"]:
        file_name = './data/gait_transition_from_low_to_high.csv'
        open(file_name, 'w+')
        print('gait transition start from ' +
              str(target_value_ref["begin_dx"]) + ' to ' + str(target_value_ref["end_dx"]))
    return file_name


def arrange_data(pos_ref, fig_ref, log_ref, con_ref, eom,simulation_time, 
                 target_dx, dx_on_peak_pos_one_value_for_one_phase, animation_phase):
    pos_com = pos_ref[LEG.COM]
    pos_left = fig_ref.pos_left
    pos_right = fig_ref.pos_right
    pos_v_swing = pos_ref[LEG.V_SWING]
    motor_pos_rear = con_ref.motor_pos[LEG.REAR]
    mechanical_energy = con_ref.mechanical_energy 
    energy_error_integral = con_ref.energy_error_integral
    actuator_len_vel =  con_ref.actuator_len_vel
    i_value_fp = con_ref.i_value_fp
    total_energy_during_one_target_dx = log_ref.total_energy_during_one_target_dx
    supplied_energy = log_ref.supplied_energy
    supplied_energy_integral = log_ref.supplied_energy_integral 
    stance_leg = fig_ref.StanceLEG
    virtual_leg_phase = con_ref.virtual_leg_phase
    phaseFLAG = eom.phaseFLAG

    pos_rear = pos_ref[LEG.REAR]
    pos_fore = pos_ref[LEG.FORE]
    pos_v_stance = pos_ref[LEG.V_STANCE]

    return[
            simulation_time,  # 0
            pos_com.dx, 
            pos_com.dy, 
            target_dx,  
            supplied_energy, 
            pos_rear.y, #5
            pos_fore.y, 
            pos_v_stance.y,
            0.0,
            motor_pos_rear, 
            mechanical_energy, #10
            0,  
            0.0, 
            energy_error_integral,
            actuator_len_vel,
            supplied_energy_integral,#15
            i_value_fp,
            pos_com.ddx_damper,
            pos_com.ddy_damper,
            total_energy_during_one_target_dx,#19
            pos_com.x, #20
            pos_com.y, 
            pos_left.x, 
            pos_left.y,
            pos_right.x, 
            pos_right.y, #25
            pos_v_swing.x, 
            pos_v_swing.y,
            phaseFLAG,
            stance_leg,
            virtual_leg_phase,#30
            animation_phase,
            dx_on_peak_pos_one_value_for_one_phase     
    ] 

def change_target_dx_depending_on_simulation_mode(log_ref, tim_flag_ref, 
                                                  simulation_time, target_dx, virtual_leg_phase, simulation_scenario_ref, target_value_ref):
    # cnt_step, supplied_energy_integral
    log_ref.supplied_energy_integral
    global sign, returned_time
    begin_dx = target_value_ref["begin_dx"]
    end_dx = target_value_ref["end_dx"]

    if (simulation_scenario_ref["is_one_target_data"] == False):
        sign = 1 if begin_dx < end_dx else -1
        round_for_time = ((simulation_time-returned_time) //
                          target_value_ref["duration_time_for_one_target"])
        pre_target_dx = target_dx
        changed_target_dx = begin_dx + sign * target_value_ref["change_target_dx_every"] * round_for_time
        if (changed_target_dx != pre_target_dx) and (tim_flag_ref.is_change_speed == False):
            tim_flag_ref.is_change_speed = True
            print("sim time: "+str(np.round(simulation_time,2)), "target_dx = "+str(np.round(changed_target_dx,3)))
            log_ref.total_energy_during_one_target_dx = log_ref.supplied_energy_integral/log_ref.cnt_step_for_energy_analysis
            log_ref.cnt_step_for_energy_analysis = 1
            log_ref.supplied_energy_integral = 0
        if target_dx*sign > end_dx*sign:
            # print(Tflag.is_transition_true)
            if (simulation_scenario_ref["is_transition_movie"] == True) and (tim_flag_ref.is_transition_true == True):
                __buff_dx__ = begin_dx
                begin_dx = end_dx
                end_dx = __buff_dx__
                returned_time = simulation_time
                tim_flag_ref.is_transition_true = False
                sign = 1 if begin_dx < end_dx else -1
                target_dx = begin_dx
                print(begin_dx, end_dx)
            else:
                print("end for limit target")
                sys.exit()   
    elif ((simulation_time >= count["end_time_for_one_target_data"]) 
          and (simulation_scenario_ref["is_one_target_data"] == True)):
        sys.exit()
    ## for gait transition mode
    if virtual_leg_phase == PHASE.JUMPING:
        if (tim_flag_ref.is_change_speed == True):
            target_dx = changed_target_dx
            tim_flag_ref.is_change_speed = False

    return target_dx, begin_dx, end_dx

def set_argument_parse(pos_ref, simulation_scenario_ref, target_value_ref):
    parser = argparse.ArgumentParser(description="Argparse for key=value input.")
    parser.add_argument('--mode', type=str, help='Simulation mode selection: low-to-high, high-to-low, low-to-low, one-target')
    parser.add_argument('--target', type=str, help='Target speed selection: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0..')
    parser.add_argument('--animation', action='store_true', help='Enable animation: True, False')
    args = parser.parse_args()


    mode = args.mode
    target_speed = args.target
    animation_arg = args.animation

    if animation_arg==True:
        simulation_scenario_ref["does_animation_plot"] = True
    else:
        simulation_scenario_ref["does_animation_plot"] = False

    if mode=="low-to-high":
        print(f"Simulation in mode: {mode}")
        simulation_scenario_ref["is_transition_movie"] = False
        simulation_scenario_ref["is_one_target_data"] = False
    elif mode=="high-to-low":
        __buff__ = target_value_ref["begin_dx"]
        print(f"Simulation in mode: {mode}")
        target_value_ref["begin_dx"]  = target_value_ref["end_dx"]
        target_value_ref["end_dx"] = __buff__
        simulation_scenario_ref["is_transition_movie"] = False
        simulation_scenario_ref["is_one_target_data"] = False
    elif mode=="low-to-low":
        print(f"Simulation in mode: {mode}")
        target_value_ref["duration_time_for_one_target"] = 1.0
        target_value_ref["change_target_dx_every"] = 0.1
        simulation_scenario_ref["is_transition_movie"] = True
    elif mode=="one-target":
        print(f"Simulation in mode: {mode}")
        simulation_scenario_ref["is_one_target_data"] = True
    elif mode==None:
        print(f"Simulation in mode: {mode}")
        pass
    else:
        print("Sorry: non this mode")

    target_dx = target_value["begin_dx"]
    if target_speed!=None:
        target_value_ref["begin_dx"] = target_dx = float(target_speed)
        pos_ref[LEG.COM].dx = target_value_ref["begin_dx"]
    else:
        pos_ref[LEG.COM].dx = target_value_ref["begin_dx"]

    return target_dx
