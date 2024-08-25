import numpy as np
import time
import argparse
import csv
import sys
from src.config import *
# import src.plot_for_animation

from src.plot_for_animation import *
from src.utils import *
from src.equation_of_motion import *
from src.controller import *
from src.variables import *

Pos = Position()   
target_dx = set_argument_parse(Pos, simulation_scenario, target_value)
# target_dx = 0.4
# Constructor
Fig = SpringLeg(fig_type=simulation_scenario["does_animation_plot"])
Con = Controller(target_dx)
StateTran = StateTransition()
Log = DataLog()

Eom = EquationOfMotion()
TimFlag = TimingFlag()
#-------------------------------------------------------------------
#-------------------------------------------------------------------
file_name = write_to_csv(simulation_scenario, target_value, target_dx)
cnt_for_csv, cnt_for_ani = 0, 0
simulation_time = 0
animation_phase = "Running"


previous_pahse_peak_dx = 0
current_pahse_peak_dx = 0
delta_pre_and_cur = 0
flag_for_poincare_section = True
add_exp_frag_fore_leg__is_contact = True
add_exp_frag_rear_leg__is_contact = True 
# main loop --------------------------------------------------------
#-------------------------------------------------------------------
while 1:
    ################################################################
    # start here  
    ################################################################
    #---------------------------------------------------------------
    # Dynamics calculation 
    #---------------------------------------------------------------
    # calc param for animation and qom
    Pos[LEG.COM] = Eom.simulate(Pos[LEG.COM], Pos[LEG.REAR], Pos[LEG.FORE], 
                                Con.motor_pos[LEG.REAR], Con.motor_pos[LEG.FORE])
    #--------------------------------------------------------------- 
    ## general state initialization ans set
    #--------------------------------------------------------------- 
    if Con.virtual_leg_phase == PHASE.SUPPORT:
        Fig.vslfoot.set_alpha(0.6)
        Con.motor_pos[LEG.FORE], Con.motor_pos[LEG.REAR] = 0, 0
        Con.peak_height = 0
        if flag_for_poincare_section == True:
           flag_for_poincare_section = False
           previous_pahse_peak_dx = current_pahse_peak_dx
           current_pahse_peak_dx = dx_on_peak_pos
           delta_pre_and_cur = current_pahse_peak_dx-previous_pahse_peak_dx
    elif Con.virtual_leg_phase == PHASE.JUMPING:
        Fig.vslfoot.set_alpha(0)
        Con.motor_pos[LEG.FORE], Con.motor_pos[LEG.REAR] = 0, 0
        Con.energy_error_integral, Log.supplied_energy = 0, 0
        ## gat state for controller at peak position (midstance)
        if Con.peak_height<Pos[LEG.COM].y: 
            Con.peak_height = Pos[LEG.COM].y
            dx_on_peak_pos = Pos[LEG.COM].dx
        flag_for_poincare_section = True

    # #--------------------------------------------------------------- 
    ## forward speed controller including calc virtual legs
    #--------------------------------------------------------------- 
    Pos[LEG.FORE], Pos[LEG.REAR], Pos[LEG.PRO_FORE], Pos[LEG.V_SWING] = Con.forward_speed_controller(Pos, target_dx, Con.virtual_leg_phase, 
                                                                                   StateTran.TsVSL, dx_on_peak_pos)
    if Pos[LEG.REAR].y <= 0.0:
        buff = calc_y_pos_from_len(Pos[LEG.REAR].x, Pos[LEG.COM], params["leg_len_limit"]).y
        tmp_rear_leg_y = buff if (buff > 0) else 0
        Pos[LEG.V_STANCE].y = tmp_rear_leg_y
        if (Pos[LEG.FORE].y <= 0):
            Pos[LEG.V_STANCE].y = (Pos[LEG.FORE].y +  tmp_rear_leg_y)/2
    if (Pos[LEG.REAR].y > 0.0) and (Pos[LEG.FORE].y > 0.0):
        Pos[LEG.V_STANCE].y = 0.0
    #--------------------------------------------------------------- 
    ## jump height controller
    #--------------------------------------------------------------- 
    if Con.virtual_leg_phase == PHASE.SUPPORT:
        Con.leg_ang               = Con.calc_support_leg_ang(Pos[LEG.COM], Pos[LEG.V_STANCE])
        Con.tar_mechanical_energy = calc_mechanical_energy(target_value["target_height"], StateDim(dx=target_dx))
        Con.mechanical_energy     = calc_mechanical_energy(Pos[LEG.COM].y, Pos[LEG.COM])
        Con.motor_pos[LEG.REAR]   = Con.jump_height_controller(Con.motor_pos[LEG.REAR], Con.leg_ang, 
                                                               Con.tar_mechanical_energy, Con.mechanical_energy)
    #--------------------------------------------------------------- 
    ## State Transition (EVENT)
    #--------------------------------------------------------------- 
    # print(Pos[LEG.FORE].y, Pos[LEG.V_SWING].y)
    if ((Pos[LEG.COM].dy <= 0) and (Pos[LEG.COM].ddy <= 0) and (Pos[LEG.V_SWING].y <= 0) 
        and not ((StateTran.pre_event == EVENT.CONTACT) and (StateTran.cur_event == EVENT.CONTACT))):
        TimFlag.does_get_leave_or_contact_event = True
        StateTran.pre_event, StateTran.cur_event = StateTran.cur_event, EVENT.CONTACT
        StateTran.TsVSL, StateTran.TsVSLbuff = 0, simulation_time
    # elif ((Pos[LEG.REAR].len_len > params["leg_len_limit"]) ## means "rear leg leave" which = "v_stance leave"
    elif ((Pos[LEG.V_STANCE].y > 0.0) 
          and not ((StateTran.pre_event == EVENT.LEAVE) and (StateTran.cur_event == EVENT.LEAVE))): 
        TimFlag.does_get_leave_or_contact_event = True
        StateTran.pre_event, StateTran.cur_event = StateTran.cur_event, EVENT.LEAVE
        StateTran.TsVSL = simulation_time - StateTran.TsVSLbuff
        Log.cnt_step_for_energy_analysis += 1
        Log.supplied_energy_integral += Log.supplied_energy
        Log.supplied_energy = 0
    #--------------------------------------------------------------- 
    ## State Transition (RULE)
    #--------------------------------------------------------------- 
    # print(Pos[LEG.V_STANCE].x)
    if TimFlag.does_get_leave_or_contact_event == True:
        if (Con.virtual_leg_phase == PHASE.SUPPORT) and (StateTran.cur_event == EVENT.CONTACT):
            # print(np.round(simulation_time,5), ">>>>>>>>>>>>>>>>>>>>> SUPPORT: CONTACT")
            Con.virtual_leg_phase = PHASE.SUPPORT
            Pos[LEG.FORE] = StateDim(Pos[LEG.PRO_FORE].x, 0)
            Pos[LEG.V_STANCE] = StateDim(Pos[LEG.V_SWING].x, 0)
            Pos[LEG.V_SWING] = StateDim(Pos[LEG.V_SWING].x, 0.01)
            Con.i_value_fp = 0
        elif (Con.virtual_leg_phase == PHASE.SUPPORT) and (StateTran.cur_event == EVENT.LEAVE):
            # print(np.round(simulation_time,5), ">>>>>>>>>>>>>>>>>>>>> SUPPORT: LEAVE")
            Con.virtual_leg_phase = PHASE.JUMPING
            Pos[LEG.REAR] = Pos[LEG.FORE].clone()
            Con.i_value_fp = Con.i_value_fp + target_dx - dx_on_peak_pos
            Con.i_value_fp = Con.i_value_fp+1 if target_dx < dx_on_peak_pos else Con.i_value_fp-1
            Fig.StanceLEG = LEG.LEFT if (Fig.StanceLEG == LEG.RIGHT) else LEG.RIGHT            
        elif (Con.virtual_leg_phase == PHASE.JUMPING) and (StateTran.cur_event == EVENT.CONTACT):
            # print(np.round(simulation_time,5), ">>>>>>>>>>>>>>>>>>>>> JUMP: CONTACT")
            Con.virtual_leg_phase = PHASE.SUPPORT
            if (Pos[LEG.REAR].y > 0.0):
                Pos[LEG.REAR] = Pos[LEG.FORE].clone()
                Pos[LEG.FORE].y = 0.001
            Pos[LEG.V_STANCE] = StateDim(Pos[LEG.V_SWING].x, 0)
        elif (Con.virtual_leg_phase == PHASE.JUMPING) and (StateTran.cur_event == EVENT.LEAVE):
            # print(np.round(simulation_time,5), ">>>>>>>>>>>>>>>>>>>>> JUMP: LEAVE")
            Con.virtual_leg_phase =  PHASE.JUMPING
            Pos[LEG.REAR].y = 0.01
        TimFlag.does_get_leave_or_contact_event = False
    ################################################################
    # ene here
    # following code is animation and data analysis and simulation mode change       
    ################################################################
    # gat energy for data analysis
    if Con.virtual_leg_phase == PHASE.SUPPORT:
        if (Con.leg_ang > 90.0) : 
            Log.supplied_energy += Eom.rear_spring_force * Con.actuator_len_vel * params["dt"]
    # change target dx depending on simulation mode
    target_dx, target_value["begin_dx"], target_value["end_dx"]= change_target_dx_depending_on_simulation_mode(
        Log, TimFlag, simulation_time, target_dx, Con.virtual_leg_phase, simulation_scenario, target_value)
    # for animation mode
    if(StateTran.pre_event == StateTran.cur_event) and (StateTran.pre_event == EVENT.CONTACT):
        animation_phase = "Walking"
    if(StateTran.pre_event == StateTran.cur_event) and (StateTran.pre_event == EVENT.LEAVE):
        animation_phase = "Running"

    Fig.leg_assign_for_animation(Pos[LEG.COM], Pos[LEG.V_SWING], 
                                 Pos[LEG.V_STANCE], Pos[LEG.REAR], Pos[LEG.FORE],
                                 Eom.phaseFLAG, l0, Fig.StanceLEG)
    cnt_for_ani += 1
    if (simulation_scenario["does_animation_plot"] == True) and (cnt_for_ani == count["every_for_animation"]):
        cnt_for_ani = 0
        Fig.animation(Eom.phaseFLAG, Fig.StanceLEG,  
                        Pos[LEG.COM], Pos[LEG.FORE], Pos[LEG.REAR], 
                        Pos[LEG.V_SWING], Pos[LEG.V_STANCE])
        Fig.ax.set_title(
            f'Time (sec) = {simulation_time:.2f}   Locomotion speed = {Pos[LEG.COM].dx:.2f}   Gait type = {animation_phase}', y=1.03)
    # print(Eom.phaseFLAG, Con.virtual_leg_phase)
    ## data save
    cnt_for_csv += 1
    if cnt_for_csv == count["every_for_csv_save"]:
        cnt_for_csv = 0
        with open(file_name, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(arrange_data(Pos, Fig, Log, Con, Eom, simulation_time, target_dx, delta_pre_and_cur, animation_phase))
    # time.sleep(0.0001)
    simulation_time = simulation_time + params["dt"]
# -------------------------------------------------------------------
# -------------------------------------------------------------------
