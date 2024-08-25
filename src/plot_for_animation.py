import numpy as np
import matplotlib.pyplot as plt
from .config import *
from .utils import *
from .equation_of_motion import *
from .variables import *

def queue(src, a):
    dst = np.roll(src, -1)
    dst[-1] = a
    return dst
class SpringLeg:
    def __init__(self, fig_type):
        if fig_type != None:
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(1, 1, 1)
            self.ax.axis('equal')
            self.ax.grid()
            self.ax.set_ylim([-0.115, 1.2])
            self.ax.set_xlabel('x [m]')
            self.ax.set_ylabel('y [m]')
            self.ax.set_title('VLS Length = ' + str(0.0) +
                                "   " + "dx = " + str(0.0))
            self.line4, = self.ax.plot([], [], '-', color="gray", linewidth=5)
            self.vslfoot, = self.ax.plot(
                [], [], 'o', markersize=15, color="gray", alpha=0.6)
            self.testpoint, = self.ax.plot(
                [], [], 'x', markersize=5, color="black", alpha=1.0)
            self.testpoint2, = self.ax.plot(
                [], [], 'x', markersize=5, color="black", alpha=1.0)
            self.byffpoint, = self.ax.plot(
                [], [], 'x', markersize=5, color="black", alpha=1.0)
            self.line2, = self.ax.plot([], [], '-', color="blue", linewidth=5)
            self.foot1, = self.ax.plot(
                [], [], 'o', markersize=15, color="blue")
            self.line3, = self.ax.plot([], [], '-', color="red", linewidth=5)
            self.foot2, = self.ax.plot([], [], 'o', markersize=15, color="red")
            self.line1, = self.ax.plot(
                [], [], 'o', markersize=30, color="black")
            self.testprint, = self.ax.plot(
                [], [], 'o', markersize=15, color="blue")
            self.pos_right = StateDim()
            self.pos_left = StateDim()

            self.ground, = self.ax.plot([], [], color="black")
            self.StanceLEG = LEG.LEFT


    def animation(self, phaseFLAG, StanceLEG, 
                  pos_com, pos_fore, pos_rear, pos_v_swing, pos_v_stance):
        dx, dy = pos_com.dx, pos_com.dy
        x_rightFP, y_rightFP = self.pos_right.x, self.pos_right.y
        x_leftFP, y_leftFP = self.pos_left.x, self.pos_left.y
        x, y = pos_com.x, pos_com.y
        if phaseFLAG == PHASE.DOUBLE_SUPPORT:
            x_virtual_swing, y_virtual_swing = pos_v_stance.x, pos_v_stance.y
            self.byffpoint.set_xdata(np.array([pos_v_stance.x]))
            self.byffpoint.set_ydata(np.array([pos_v_stance.y]))

        else:
            x_virtual_swing, y_virtual_swing = pos_v_swing.x, pos_v_swing.y
            self.byffpoint.set_xdata(np.array([pos_v_swing.x]))
            self.byffpoint.set_ydata(np.array([pos_v_swing.y]))
        data_fig = x, y, x_leftFP, y_leftFP, x_rightFP, y_rightFP, x_virtual_swing, y_virtual_swing
        self.update_fig_of_link_VSL(
                data_fig, self.ax, self.line1, self.line2, self.line3, self.line4, self.foot1, self.foot2, self.vslfoot)
        self.line4.set_alpha(0.6)
        self.testpoint2.set_xdata(np.array([pos_fore.x]))
        self.testpoint2.set_ydata(np.array([pos_fore.y]))
        self.testpoint.set_xdata(np.array([pos_rear.x]))
        self.testpoint.set_ydata(np.array([pos_rear.y]))
        self.ground.set_xdata(np.array([pos_fore.x-10, pos_fore.x+10]))
        self.ground.set_ydata(np.array([0.0]))
        self.pos_right.x, self.pos_right.y = x_rightFP, y_rightFP
        self.pos_left.x, self.pos_left.y = x_leftFP, y_leftFP

        plt.pause(0.01)

    def leg_assign_for_animation(self,  pos_com, pos_v_swing, pos_v_stance, pos_rear, pos_fore, phaseFLAG, l0, StanceLEG):
        ################################################################
    ################################################################
    # Leg State: Single Support phase (Actual Leg)
    #---------------------------------------------------------------
        if phaseFLAG == PHASE.SINGLE_SUPPORT:
            if (pos_com.x-pos_rear.x)**2+(pos_com.y-pos_rear.y)**2 > 0.1:  # tmp to fix plot error
                self.vslfoot.set_alpha(0.0)
                self.line2.set_alpha(0.2 if StanceLEG == LEG.RIGHT else 1)
                self.foot1.set_alpha(0.0 if StanceLEG == LEG.RIGHT else 1)
                self.line3.set_alpha(0.2 if StanceLEG == LEG.LEFT else 1)
                self.foot2.set_alpha(0.0 if StanceLEG == LEG.LEFT else 1)
                self.pos_right = pos_rear.clone() if StanceLEG == LEG.RIGHT else pos_fore.clone()
                self.pos_left  = pos_rear.clone() if StanceLEG == LEG.LEFT else pos_fore.clone()
        # Leg State: Double Support phase (Actual Leg)
        #---------------------------------------------------------------    
        elif phaseFLAG == PHASE.DOUBLE_SUPPORT:
            self.line2.set_alpha(1)
            self.foot1.set_alpha(1)
            self.line3.set_alpha(1)
            self.foot2.set_alpha(1)
            self.pos_right = pos_rear.clone() if StanceLEG == LEG.RIGHT else pos_fore.clone()
            self.pos_left  = pos_rear.clone() if StanceLEG == LEG.LEFT else pos_fore.clone()
        # Leg State: Jump phase (Actual Leg)
        #---------------------------------------------------------------    
        elif phaseFLAG == PHASE.JUMPING:
            self.line2.set_alpha(0.2 if StanceLEG == LEG.RIGHT else 1)
            self.foot1.set_alpha(0.0)
            self.line3.set_alpha(0.2 if StanceLEG == LEG.LEFT else 1)
            self.foot2.set_alpha(0.0)
            # __buff__ = calc_y_pos_from_len(pos_v_swing.x, pos_com, l0)
            if StanceLEG == LEG.RIGHT:
                self.pos_right  =  pos_fore.clone()
                self.pos_left = StateDim(x=pos_com.x-(pos_fore.x-pos_com.x), y=0.1)  
            else:
                self.pos_right = StateDim(
                    x=pos_com.x-(pos_fore.x-pos_com.x), y=0.1)  # pos_rear.clone()
                self.pos_left =  pos_fore.clone()
    
    def spring(self, xm, ym, xf, yf, lines, foot):
        legEC, iniWidth = 1.0, 0.05
        diffs, diff, x, y = 0.25, [0] * 10, [0] * 10, [0] * 10
        legEC = np.sqrt(np.power((xm - xf), 2) +
                        np.power((ym - yf), 2)) - diffs * 2.0
        diffe = diffs + 16.0 / 16 * legEC
        for i in range(0, 10):
            diff[i] = diffs + (2.0 * i + 1.0) / 16.0 * legEC
        radian = np.arctan2(ym - yf, xm - xf)
        for i in range(1, 9):
            x[i] = xm - diff[i - 1] * \
                np.cos(radian) + ((-1)**(i + 1)) * iniWidth * np.sin(radian)
            y[i] = ym - diff[i - 1] * \
                np.sin(radian) + ((-1)**(i)) * iniWidth * np.cos(radian)
        x[0], y[0] = xm - diffs * np.cos(radian), ym - diffs * np.sin(radian)
        x[9], y[9] = xm - diffe * np.cos(radian), ym - diffe * np.sin(radian)
        lines.set_xdata(np.array([xm, x[0], x[1], x[2], x[3], x[
                        4], x[5], x[6], x[7], x[8], x[9], xf]))
        lines.set_ydata(np.array([ym, y[0], y[1], y[2], y[3], y[
                        4], y[5], y[6], y[7], y[8], y[9], yf]))
        foot.set_xdata(np.array([xf]))
        foot.set_ydata(np.array([yf]))

    def update_fig_of_link(self, data1, ax, line1, line2, line3, foot1, foot2, phase_state):
        xs, ys = np.reshape(data1, (-1, 2)).T
        self.ax.set_xlim([xs[0] - 10, xs[0] + 10])
        self.spring(xs[0], ys[0], xs[1], ys[1], line2, foot1)
        self.spring(xs[0], ys[0], xs[2], ys[2], line3, foot2)
        line1.set_xdata(np.array([xs[0]]))
        line1.set_ydata(np.array([ys[0]]))

    def update_fig_of_link_VSL(self, data1, ax, line1, line2, line3, line4, foot1, foot2, foot3):
        xs, ys = np.reshape(data1, (-1, 2)).T
        ax.set_xlim([xs[0] - 10, xs[0] + 10])
        self.spring(xs[0], ys[0], xs[1], ys[1], line2, foot1)
        self.spring(xs[0], ys[0], xs[2], ys[2], line3, foot2)
        self.spring(xs[0], ys[0], xs[3], ys[3], line4, foot3)
        line1.set_xdata(np.array([xs[0]]))
        line1.set_ydata(np.array([ys[0]]))
        return line1
