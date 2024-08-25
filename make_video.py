from src.config import PHASE, EVENT, LEG  # config.pyからenumをインポート
import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation

from src.plot_for_animation import *
from src.utils import *
from src.equation_of_motion import *
from src.controller import *
from src.variables import *
import os

Con = Controller(target_dx)

SpringLegClass = SpringLeg(None)
SlipClass = EquationOfMotion()
# setting initial parameters
k, c, l0, m, g, dt, alpha0 = params["spring_const"], params["damper_const"], params["natural_len_fore"], params["mass"], params["gravity"], params["dt"], params["alpha0"]

Pos = Position()
StateTran = StateTransition()
target_height = target_value["target_height"]
# setting variable parameters
x, dx, ddx = Pos[LEG.COM].x, Pos[LEG.COM].dx, Pos[LEG.COM].ddx
y, dy, ddy = Pos[LEG.COM].y, Pos[LEG.COM].dy, Pos[LEG.COM].ddy
footstate = [x_rear_FP, x_fore_FP, x_leftFP, y_leftFP,
             x_rightFP, y_rightFP, l0_rear, l0_fore] = [0] * 8
state = [phaseFLAG, step_count, top_speed, alpha0,
         jump_count, state_flag] = [0, 0, 0, alpha0, 0, 0]
figcnt, count["every_for_animation"] = 0, count["every_for_animation"]
Uh, K = m * g * y, 1.0 / 2.0 * m * dx**2 + 1.0 / 2.0 * m * dy**2

Con.tar_len_VL, target_dx = Con.tar_len_VL, target_value["begin_dx"]
target_dx = 1.0
dx = target_dx
# setting for figure------------------------------------------
#-------------------------------------------------------------
fig, ax = plt.subplots(figsize=(5, 4))
ax.axis('equal')
ax.grid()
ax.set_ylim([-0.115, 1.2])
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title(
    f'Time (sec) = {0:.2f} \n  Locomotion speed = {0:.2f}   Gait type = running',
    loc='left', 
    y=1.01)
ground, = ax.plot([], [], color="black")
line2, = ax.plot([], [], '-', color="blue", linewidth=5)
foot1, = ax.plot([], [], 'o', markersize=15, color="blue")
line3, = ax.plot([], [], '-', color="red", linewidth=5)
foot2, = ax.plot([], [], 'o', markersize=15, color="red")
vslfoot, = ax.plot([], [], 'o', markersize=15, color="gray", alpha=0.6)
line4, = ax.plot([], [], '-', color="gray", linewidth=5)
line1, = ax.plot([], [], 'o', markersize=30, color="black")
line4.set_alpha(0.6)



#-------------------------------------------------------------------
#-------------------------------------------------------------------


# Setting of initial value -----------------------------------------
#-------------------------------------------------------------------
Con.motor_pos[LEG.REAR], Con.motor_pos[LEG.FORE] = 0, 0
time = 0
dx_on_peak_pos = target_dx
plt_x, plt_y, plt_y2, plt_y3, plt_y4 = [], [], [], [], []

phaseFLAG, prePhaseFLAG, StanceLEG = PHASE.JUMPING, PHASE.SINGLE_SUPPORT, LEG.LEFT
virtual_leg_phase, preVslPhase = PHASE.JUMPING, PHASE.SUPPORT

x_vslFoot = x + ((dx_on_peak_pos) * StateTran.TsVSL / 2.0)
y_vslFoot = y - (np.sqrt(pow(Con.tar_len_VL, 2) - pow(x - x_vslFoot, 2)))
x_fore_FP = (x_vslFoot - x_rear_FP) * 2 + x_rear_FP

file_path = os.path.join(
    os.getcwd(), "data/gait_transition_for_movie.csv")

data = np.genfromtxt(file_path, delimiter=',', dtype=str, encoding=None)
numeric_cols = []
non_numeric_cols = []
for i in range(data.shape[1]):  
    if i in [data.shape[1] + -1, data.shape[1] + -2, data.shape[1] + -3,  data.shape[1] + -4]:
        non_numeric_cols.append(data[:, i])
    else:
        numeric_cols.append(data[:, i].astype(float))
data = np.array(numeric_cols, dtype=float)  
non_numeric_data = np.array(non_numeric_cols)  

#-------------------------------------------------------------------
#-------------------------------------------------------------------


# function definisions ----------------------------------------------
def plot_animation(figcnt):
    figcnt = figcnt * 2 #2 is real time
    global ax, line1, line2, line3, line4, foot1, foot2, vslfoot, phaseFLAG, data, StanceLEG
    time = data[0][figcnt]
    dx = data[1][figcnt]
    target_dx = data[3][figcnt]
    Con.tar_len_VL = 0.94
    x = data[20][figcnt]
    y = data[21][figcnt] 
    x_leftFP = data[22][figcnt]
    y_leftFP = data[23][figcnt]
    x_rightFP = data[24][figcnt]
    y_rightFP = data[25][figcnt]
    x_vslFoot = data[26][figcnt]
    y_vslFoot = data[27][figcnt]
    phaseFLAG = non_numeric_data[-4][figcnt]
    StanceLEG = non_numeric_data[-3][figcnt]
    virtual_leg_phase = non_numeric_data[-2][figcnt]
    animation_phase = non_numeric_data[-1][figcnt]
    

    if virtual_leg_phase == "PHASE.SUPPORT":
        vslfoot.set_alpha(0.3)
    elif virtual_leg_phase == "PHASE.JUMPING":
        vslfoot.set_alpha(0)
    vslfoot.set_alpha(0)
    line4.set_alpha(0.3)

    if phaseFLAG == "PHASE.SINGLE_SUPPORT":
        vslfoot.set_alpha(0.0)
        line2.set_alpha(0.2 if StanceLEG == "LEG.RIGHT" else 1)
        foot1.set_alpha(0.0 if StanceLEG == "LEG.RIGHT" else 1)
        line3.set_alpha(0.2 if StanceLEG == "LEG.LEFT" else 1)
        foot2.set_alpha(0.0 if StanceLEG == "LEG.LEFT" else 1)

# Leg State: Double Support phase (Actual Leg)
#---------------------------------------------------------------    
    elif phaseFLAG == "PHASE.DOUBLE_SUPPORT":
        line2.set_alpha(1)
        foot1.set_alpha(1)
        line3.set_alpha(1)
        foot2.set_alpha(1)
        # Leg State: Jump phase (Actual Leg)
#---------------------------------------------------------------    
    elif phaseFLAG == "PHASE.JUMPING":
        line2.set_alpha(0.2 if StanceLEG == "LEG.RIGHT" else 1)
        foot1.set_alpha(0.0)
        line3.set_alpha(0.2 if StanceLEG == "LEG.LEFT" else 1)
        foot2.set_alpha(0.0)
    
    ax.set_ylim([-0.115, 1.2])
    data_fig = [x, y, x_leftFP, y_leftFP,
                x_rightFP, y_rightFP, x_vslFoot, y_vslFoot]
    line1 = SpringLegClass.update_fig_of_link_VSL(
        data_fig, ax, line1, line2, line3, line4, foot1, foot2, vslfoot)
    
    ground.set_xdata(np.array([x-10, x+10]))
    ground.set_ydata(np.array([0.0]))
    ax.set_title(
    f'Time: {time:.2f} (sec)                  Gait type: {animation_phase} \nLocomotion speed: {dx:.2f} (m/s)',
    loc='left', 
    y=1.01)


ani = animation.FuncAnimation(fig, plot_animation, interval = 1)
ani.save("./video/transition_video.mp4", writer = 'ffmpeg')
# plt.show()


