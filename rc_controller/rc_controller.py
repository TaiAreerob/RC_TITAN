import numpy as np
import matplotlib.pyplot as plt
import sys
import csv
import math
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D 
from collections import deque 
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference


L = 1.1  # [m] Wheel base of vehicle
max_steer = np.radians(25.0)  # [rad] max steering angle

show_animation = True
ref_x = [] 
ref_y = [] 
ref_yaw = []
global pid,errorIsum ,errorD_last,error_debug
pid = [1.0,0.0,0.0]
errorIsum =0
errorD_last =0
error_debug =[0.0]
cte_debug = [0.0]
cte_debug_rms = [0.0]
yaw_debug=[0.0]
class State(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        delta = np.clip(delta, -max_steer, max_steer)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        # self.v += acceleration * dt
        self.v = 2.24


def pid_control(target, current):
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)
    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx
    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx

def pid_steering_control(error_front_axle):
    global pid,errorIsum,errorD_last
    
    # if last_target_idx >= current_target_idx:
    #     current_target_idx = last_target_idx
    errorP = error_front_axle
    errorI = errorIsum
    errorD = errorD_last - error_front_axle
    error = (pid[0] * errorP) + (pid[1] * errorI) + (pid[2] * errorD)
    errorIsum = errorIsum+error
    errorD_last = error
    
    return error,error_front_axle


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def calc_target_index2(state, cx, cy):
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = fx - cx
    dy = fy - cy 

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx, dy], front_axle_vec)

    return error_front_axle

def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
def main_ref():
    global ref_x,ref_y,ref_yaw
    with open('data.csv', 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            ref_x.append(float(row[0]))
            ref_y.append(float(row[1]))
            _,_,yaw=euler_from_quaternion(float(row[4]),float(row[5]),float(row[6]),float(row[3]))
            ref_yaw.append(yaw)
def rmse(predictions, targets):
    return np.sqrt(((predictions - targets) ** 2).mean())

def main():
    global error_debug,yaw_debug

    cyaw = ref_yaw
    target_speed = 2.0 # [m/s]

    max_simulation_time =1000
    Nloop = 15
    RC_loop = 1771 * Nloop
    ref_x_copy =ref_x
    ref_y_copy =ref_y
    ref_x_copy[len(ref_x_copy):] = np.zeros(RC_loop)
    ref_y_copy[len(ref_y_copy):] = np.zeros(RC_loop)
    cx = ref_x_copy
    cy = ref_y_copy
    # Initial state
    state = State(x=1.1710157490773279, y=0.14681135715130766, yaw=np.radians(3.14), v=2.0)
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    #target_idx, _ = calc_target_index(state, cx, cy)
    count_loop = 0
    plotloop =[0.0]
    loop_count=[0]
    deque_create = np.zeros(4100)
    error_control_input_x = np.zeros(RC_loop+10000)
    error_control_input_y = np.zeros(RC_loop+10000)
    error_rc = deque(deque_create,maxlen=4100)
    i=0
    tick_loop =0
    KRC =0.5
    error_avg_count=[0.0]
    #while max_simulation_time >= time and last_idx > target_idx and count_loop < 3:
    # while  count_loop <= 5:
    for n in range(Nloop):
        cte_debug_rms = []
        for i in range(1771):
            ai = pid_control(target_speed, state.v)
            X_control = cx[i+((n)*1771)] + (KRC*error_control_input_x[i+((n)*1771)+1])
            Y_control = cy[i+((n)*1771)] + (KRC*error_control_input_y[i+((n)*1771)+1])
            #print("input = "+str(i+((n)*1771)),"error = "+str(i+(n*1771)+1),"input = "+str(i+((n+1)*1771)))
            # if(i+((n)*1771)%1771==1):
            #     print(X_control,Y_control)
            ctenow = calc_target_index2(state,X_control, Y_control)
            di,cte = pid_steering_control(ctenow)
            cx[i+((n+1)*1771)] = X_control
            cy[i+((n+1)*1771)]= Y_control
            state.update(ai, di)
            error_control_input_x[i+((n+1)*1771)] = X_control - state.x
            error_control_input_y[i+((n+1)*1771)] = Y_control - state.y
            x.append(state.x)
            y.append(state.y)
            time += dt
            error_debug.append(di/3.14*180)
            cte_debug.append(cte)
            cte_debug_rms.append(cte)
            yaw_debug.append(state.yaw/3.14*180)
            t.append(time)
        plotloop.append(time)
        rmse_val = rmse(np.array(cte_debug_rms),np.zeros(len(cte_debug_rms)))
        print("rms error is: " + str(rmse_val),"=>diff=",rmse_val-error_avg_count[len(error_avg_count)-1])
        error_avg_count.append(rmse_val)   
    print(len(ref_x))
    if show_animation:  # pragma: no cover
        plt.plot(ref_x,ref_y, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        # plt.subplots(1)
        # plt.plot(t, [iv * 3.6 for iv in v], "-r")
        # plt.xlabel("Time[s]")
        # plt.ylabel("Speed[km/h]")
        # plt.grid(True)
        fig, (ax1, ax2) = plt.subplots(2)
        fig.suptitle('Tracking PID')
        ax1.set(xlabel='time(s)', ylabel='cte error (m)')
        ax1.plot(t,cte_debug, "-r")
        ###################################
        # line1 = [(0,0), (0,50)]
        # (line1_xs, line1_ys) = zip(*line1)
        # ax2.add_line(Line2D(line1_xs, line1_ys, linewidth=2, color='blue'))
        ###################################
        ax1.grid(True)
        ax2.set(xlabel='time(s)', ylabel='steering angle (degree)')
        ax2.plot(t,error_debug, "-r")
        ax2.grid(True)
        #ax1.xaxis.set_major_locator(MultipleLocator(5))
        ax1.yaxis.set_major_locator(MultipleLocator(0.1))
        #ax1.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax1.yaxis.set_minor_locator(AutoMinorLocator(5))
        ax1.grid(which='major', color='#CCCCCC', linestyle='--')
        #ax1.xaxis.set_major_locator(MultipleLocator(5))
        ax2.yaxis.set_major_locator(MultipleLocator(20))
        #ax1.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax2.yaxis.set_minor_locator(AutoMinorLocator(5))
        ax2.grid(which='major', color='#CCCCCC', linestyle='--')
        fig1, (ax3, ax4,ax5) = plt.subplots(3)
        fig1.suptitle('Tracking PID')
        ax3.set(xlabel='time(s)', ylabel='X (m)')
        ax3.plot(t,x, "-r")
        ax3.grid(True)
        ax4.set(xlabel='time(s)', ylabel='Y (m)')
        ax4.plot(t,y, "-r")
        ax4.grid(True)
        ax5.set(xlabel='time(s)', ylabel='heading angle (degree)')
        ax5.plot(t,yaw_debug, "-r")
        ax5.grid(True)
        ax3.xaxis.set_major_locator(MultipleLocator(20))
        ax3.yaxis.set_major_locator(MultipleLocator(10))
        ax3.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax3.yaxis.set_minor_locator(AutoMinorLocator(5))
        ax3.grid(which='major', color='#CCCCCC', linestyle='--')
        ax4.xaxis.set_major_locator(MultipleLocator(20))
        ax4.yaxis.set_major_locator(MultipleLocator(10))
        ax4.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax4.yaxis.set_minor_locator(AutoMinorLocator(5))
        ax4.grid(which='major', color='#CCCCCC', linestyle='--')
        ax5.xaxis.set_major_locator(MultipleLocator(20))
        ax5.yaxis.set_major_locator(MultipleLocator(50))
        ax5.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax5.yaxis.set_minor_locator(AutoMinorLocator(5))
        ax5.grid(which='major', color='#CCCCCC', linestyle='--')
        fig2, (ax6) = plt.subplots(1)
        fig2.suptitle('Tracking PID')
        
        y_pos = np.arange(len(error_avg_count))
        ax6.yaxis.set_major_locator(MultipleLocator(0.01))
        ax6.grid(which='major', color='#CCCCCC', linestyle='--')
        ax6.bar(y_pos, error_avg_count, align='center', alpha=0.5)
        for indexplot in range(len(plotloop)):
           if(indexplot%2==0):
               ax1.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#A7EEFF')
               ax2.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#A7EEFF')
            #    ax3.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#A7EEFF')
            #    ax4.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#A7EEFF')
            #    ax5.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#A7EEFF')
               
           elif(indexplot%2==1):
               ax1.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#CBFF75')
               ax2.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#CBFF75')
            #    ax3.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#CBFF75')
            #    ax4.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#CBFF75')
            #    ax5.axvspan(plotloop[indexplot-1],plotloop[indexplot], facecolor='#CBFF75')
        plt.show()
    # print(range(len(ref_x)))
    # for loop in range(3):
    #     for t in range(len(ref_x)):
    #         ai = pid_control(target_speed, state.v)
    #         x_target = 
    #         y_target = 
    #         cte = calc_target_index2(state, cx[n], cy[n])
    #         di,cte = pid_steering_control(cte)
    #         state.update(ai, di)
    #         time += dt
    #         error_debug.append(di/3.14*180)
    #         cte_debug.append(cte)
    #         # print(state.x,state.y)
    #         x.append(state.x)
    #         y.append(state.y)
    #         yaw.append(state.yaw)
    #         yaw_debug.append(state.yaw/3.14*180)
    #         v.append(state.v)
    #         t.append(time)
    #         error_rc.append(di)
        
    # for n in range(ref_x):
    #     #print(last_idx,target_idx)
    #     ai = pid_control(target_speed, state.v)
    #     current_target_idx, unit = calc_target_index(state, cx, cy)
    #     error_new = unit + (KRC*error_rc[n%4000])
    #     if(n<3999):
    #         error_new =unit
    #     di,cte = pid_steering_control(error_new)
    #     error_rc[n%4000] = di
    #     control_input[n%4000] = unit
    #     state.update(ai, di)
    #     time += dt
    #     error_debug.append(di/3.14*180)
    #     cte_debug.append(cte)
    #     # print(state.x,state.y)
    #     x.append(state.x)
    #     y.append(state.y)
    #     yaw.append(state.yaw)
    #     yaw_debug.append(state.yaw/3.14*180)
    #     v.append(state.v)
    #     t.append(time)
    #     error_rc.append(di)
    #     # if(-0.2 < state.x < -0.1 and 0.2 <state.y< 1.0):
    #     if(n%4000 ==3999):
    #         #print("tai")
    #         count_loop=count_loop+1
    #         plotloop.append(time)
    #         loop_count.append(len(x))
    #         rmse_val = rmse(np.array(cte_debug),np.zeros(len(cte_debug)))
    #         error_avg_count.append(rmse_val)
    #         print("rms error is: " + str(rmse_val))
    #         print(count_loop)
    #         i = 0
    #         #print(time,state.x,state.y)
    #     # if show_animation:  # pragma: no cover
    #     #     plt.cla()
    #     #     # for stopping simulation with the esc key.
    #     #     plt.gcf().canvas.mpl_connect('key_release_event',
    #     #             lambda event: [exit(0) if event.key == 'escape' else None])
    #     #     plt.plot(cx, cy, ".r", label="course")
    #     #     plt.plot(x, y, "-b", label="trajectory")
    #     #     plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
    #     #     plt.axis("equal")
    #     #     plt.grid(True)
    #     #     plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
    #     #     plt.pause(0.00001)
    #     i = i+1
    # numnote  = len(cte_debug)
    # Test
    # assert last_idx >= target_idx, "Cannot reach goal"
    # for indexplot in range(1,len(loop_count)):
    #     print(loop_count[indexplot]-loop_count[indexplot-1])
    # if show_animation:  # pragma: no cover
    #     plt.plot(cx, cy, ".r", label="course")
    #     plt.plot(x, y, "-b", label="trajectory")
    #     plt.legend()
    #     plt.xlabel("x[m]")
    #     plt.ylabel("y[m]")
    #     plt.axis("equal")
    #     plt.grid(True)
    #     # plt.subplots(1)
    #     # plt.plot(t, [iv * 3.6 for iv in v], "-r")
    #     # plt.xlabel("Time[s]")
    #     # plt.ylabel("Speed[km/h]")
    #     # plt.grid(True)
    #     fig, (ax1, ax2) = plt.subplots(2)
    #     fig.suptitle('Tracking PID')
    #     ax1.set(xlabel='time(s)', ylabel='cte error (m)')
    #     ax1.plot(t,cte_debug, "-r")
    #     ###################################
    #     # line1 = [(0,0), (0,50)]
    #     # (line1_xs, line1_ys) = zip(*line1)
    #     # ax2.add_line(Line2D(line1_xs, line1_ys, linewidth=2, color='blue'))
       
    #     ###################################
    #     ax1.grid(True)
    #     ax2.set(xlabel='time(s)', ylabel='steering angle (degree)')
    #     ax2.plot(t,error_debug, "-r")
    #     ax2.grid(True)
    #     #ax1.xaxis.set_major_locator(MultipleLocator(5))
    #     ax1.yaxis.set_major_locator(MultipleLocator(0.1))
    #     #ax1.xaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax1.yaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax1.grid(which='major', color='#CCCCCC', linestyle='--')
    #     #ax1.xaxis.set_major_locator(MultipleLocator(5))
    #     ax2.yaxis.set_major_locator(MultipleLocator(20))
    #     #ax1.xaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax2.yaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax2.grid(which='major', color='#CCCCCC', linestyle='--')
        
    #     fig1, (ax3, ax4,ax5) = plt.subplots(3)
    #     fig1.suptitle('Tracking PID')
    #     ax3.set(xlabel='time(s)', ylabel='X (m)')
    #     ax3.plot(t,x, "-r")
    #     ax3.grid(True)
    #     ax4.set(xlabel='time(s)', ylabel='Y (m)')
    #     ax4.plot(t,y, "-r")
    #     ax4.grid(True)
    #     ax5.set(xlabel='time(s)', ylabel='heading angle (degree)')
    #     ax5.plot(t,yaw_debug, "-r")
    #     ax5.grid(True)
    #     ax3.xaxis.set_major_locator(MultipleLocator(20))
    #     ax3.yaxis.set_major_locator(MultipleLocator(10))
    #     ax3.xaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax3.yaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax3.grid(which='major', color='#CCCCCC', linestyle='--')
    #     ax4.xaxis.set_major_locator(MultipleLocator(20))
    #     ax4.yaxis.set_major_locator(MultipleLocator(10))
    #     ax4.xaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax4.yaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax4.grid(which='major', color='#CCCCCC', linestyle='--')
    #     ax5.xaxis.set_major_locator(MultipleLocator(20))
    #     ax5.yaxis.set_major_locator(MultipleLocator(50))
    #     ax5.xaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax5.yaxis.set_minor_locator(AutoMinorLocator(5))
    #     ax5.grid(which='major', color='#CCCCCC', linestyle='--')
    #     fig2, (ax6) = plt.subplots(1)
    #     fig2.suptitle('Tracking PID')
        
    #     y_pos = np.arange(len(error_avg_count))
    #     ax6.yaxis.set_major_locator(MultipleLocator(0.01))
    #     ax6.grid(which='major', color='#CCCCCC', linestyle='--')
    #     ax6.bar(y_pos, error_avg_count, align='center', alpha=0.5)
       
    #     plt.show()

if __name__ == '__main__':
    main_ref()
    main()
