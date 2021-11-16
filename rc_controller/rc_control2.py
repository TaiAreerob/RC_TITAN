import numpy as np
import matplotlib.pyplot as plt
import sys
import csv
import math
import random

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D 
from collections import deque 
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
#Set all parameters
pid_steering = [0.5,0.0,0.0]  # control gain
pid_velocity = [0.5,0.0,0.0]  # control gain
dt = 0.1  # [s] time difference

loopRC = 10
K_RC = 0

v = 1; #Velocity of 1 m/s
L = 1.1  # [m] Wheel base of vehicle
max_steer = np.radians(25.0)  # [rad] max steering angle
######################################################################]
ref_x = [] 
ref_y = []
ref_yaw = []

xp=0
yp=0
x_next=0
y_next=0
ref_x_sim= []
ref_y_sim= []
ref_yaw_sim= []
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
        #self.v += acceleration * dt
        self.v = 1.5

def fineWayPoint(W,P,current_index):
    #####find dis waypoint point
    dx = [P[0]- icx for icx in W[0]]
    dy = [P[1] - icy for icy in W[1]]
    d1 = np.hypot(dx, dy)
    d2 = d1
    target_idx1 = np.argmin(d1)
    target_remove =target_idx1
    for dis2 in d2:
        d2=np.delete(d2,target_remove, 0)
        target_idx2 = np.argmin(d2)
        if(target_idx1<target_idx2):
            break
        target_remove = target_idx2
    # #####Create Unit vector
    distance_v1 = [W[0][target_idx1] - P[0], W[1][target_idx1] - P[1]]
    direction_v1 = [distance_v1[0] / np.linalg.norm(distance_v1), distance_v1[1] /np.linalg.norm(distance_v1)]
    distance_v2 = [W[0][target_idx2] - W[0][target_idx1], W[1][ target_idx2] - W[1][target_idx1]]
    direction_v2 = [distance_v2[0] / np.linalg.norm(distance_v2), distance_v2[1] /np.linalg.norm(distance_v2)]
    theta = np.rad2deg(math.acos(np.cross(direction_v1, direction_v2)))
    print(theta)
    if abs(theta) > 90:
        next = W[:,target_idx2]
        Ppass = W[:,target_idx1]
        next_index = target_idx2
        pass_index = target_idx1
    elif abs(theta) < 90:
        next = W[:,target_idx1]
        Ppass = W[:,target_idx1-1]
        next_index = target_idx1
        pass_index = target_idx1-1
    elif abs(theta) == 90:
        next = W[:,target_idx1]
        Ppass = W[:,target_idx1-1]
        next_index = target_idx1
        pass_index = target_idx1-1
    if current_index == 1:
        next =  W[:,target_idx1]
        Ppass = [0,0]
        next_index = current_index+1
        pass_index = 1
    return next,Ppass,next_index,pass_index

def crossTrackError(state,way_point):
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)
    #check if w1 is passed
    dx = fx - way_point[0]
    dy = fy - way_point[1]
    #find angle of the two vectory
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx, dy], front_axle_vec)
    return error_front_axle
    
def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

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
    # index_min_1 = np.argmin(d)
    # index_min_2 = np.argmin(d)+1
    
    # #####Create Unit vector
    # distance_v1 = [W[0][index_min_1] - P[0], W[1][index_min_1] - P[1]]
    # direction_v1 = [distance_v1[0] / np.linalg.norm(distance_v1), distance_v1[1] /np.linalg.norm(distance_v1)]
    # distance_v2 = [W[0][index_min_1] - P[0], W[1][index_min_1] - P[1]]
    # direction_v2 = [distance_v2[0] / np.linalg.norm(distance_v2), distance_v2[1] /np.linalg.norm(distance_v2)]
    # theta = np.rad2deg(math.acos(np.cross(direction_v1, direction_v2)))
    # if abs(theta) > 90:
    #     next = W[index_min_1]
    #     Ppass = W[index_min_2]
    # elif abs(theta) < 90:
    #     next = W[index_min_1]
    #     Ppass = W[index_min_1-1]
    # elif abs(theta) == 90:
    #     next = W[index_min_1]
    #     Ppass = W[index_min_1-1]
    # if index == 1:
    #     next = W[index_min_1]
    #     Ppass = [0,0]
    #     next_index = index+1
    #     pass_index = [1]
        
    # return next,Ppass,next_index,pass_index)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z # in radians
    
def readCsv():
    global ref_x,ref_y,ref_yaw
    global ref_x_sim,ref_y_sim,ref_yaw_sim
    with open('data.csv', 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            ref_x.append(float(row[0]))
            ref_y.append(float(row[1]))
            _,_,yaw=euler_from_quaternion(float(row[4]),float(row[5]),float(row[6]),float(row[3]))
            ref_yaw.append(yaw)
            ########Gen sim #############################
            ref_x_sim.append(float(row[0])*random.uniform(0.991, 1.01))
            ref_y_sim.append(float(row[1])*random.uniform(0.991, 1.01))
            print(random.uniform(0.9, 0.99))
            ref_yaw_sim.append(yaw)

def main():
    global ref_x,ref_y,ref_yaw
    global x_next,y_next
    global xp,yp
    global ref_x_sim,ref_y_sim,ref_yaw_sim
    state = State(x=0.0, y=0.0, yaw=np.radians(0), v=0.0)
    i=0
    for i in range(len(ref_x)):
        point_init = [ref_x[i]/0.7,ref_y[i]/0.7]
        xp=point_init[0]
        yp=point_init[1]
        waypoint=np.array([ref_x,ref_y])
        next,Ppass,next_index,pass_index = fineWayPoint(waypoint,point_init,1)
        x_next =next[0]
        y_next =next[1]
        cte = crossTrackError(state,next)
        print("CTE = "+str(cte))
        show_animation =1
        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(ref_x,ref_y, ".r", label="course")
            plt.plot(ref_x_sim,ref_y_sim, ".b", label="Sim")
            plt.plot(x_next,y_next, "b", label="next",marker=".", markersize=20)
            plt.plot(xp,yp, "g", label="P",marker=".", markersize=20)
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

            # while i < loopRC:
        #     print(i)
        # state.update(0,steering_angle)
        # if(last_index == last_index):
        #     i+=1

def plotresult():
    global ref_x,ref_y,ref_yaw
    show_animation =1
    global x_next,y_next
    global xp,yp
    print(x_next,y_next)
    print(xp,yp)
    global ref_x_sim,ref_y_sim,ref_yaw_sim
    if show_animation:  # pragma: no cover
        plt.plot(ref_x,ref_y, ".r", label="course")
        plt.plot(ref_x_sim,ref_y_sim, ".b", label="Sim")
        plt.plot(x_next,y_next, "b", label="next",marker=".", markersize=10)
        plt.plot(xp,yp, "g", label="P",marker=".", markersize=10)
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.show()
        
if __name__ == '__main__':
    readCsv()
    main()
    plotresult()