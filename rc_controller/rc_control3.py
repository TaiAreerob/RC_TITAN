"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Punyapat Areerob (@punyapat)

"""
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import csv

from numpy.core.records import array
sys.path.append("CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise
#parameters
ref_x = [] 
ref_y = []
NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 0  # horizon length
MAX_TIME = 1000.0  # max simulation time
DT = 0.001  # [s] time tick
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 5.0 / 3.6  # stop speed
# iterative paramter
pid = [0.3,0,0.0001]
errorIsum = 0.0
errorD_last = 0.0

TARGET_SPEED = 5 / 3.6  # [m/s] target speed
N_IND_SEARCH = 20  # Search index number
# Vehicle parameters
LENGTH = 1.1  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 1.1  # [m]
findPidAuto =False

MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
MAX_SPEED = 5 / 3.6  # maximum speed [m/s]
MIN_SPEED = -5 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = False
class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
        
def get_path(dl):
    global ref_x,ref_y
    with open('data.csv', 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            ref_x.append(float(row[0]))
            ref_y.append(float(row[1]))
    ax = ref_x
    ay = ref_y
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    return cx, cy, cyaw, ck

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi
    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi
    return angle

def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile
def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    if(pind == 0): ind = 1
    return ind, mind
def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]
        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]
        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw
def rmse(predictions, targets):
    return np.sqrt(((predictions - targets) ** 2).mean())
def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)
    
    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref
def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)
    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    #if isgoal and isstop:
    if isgoal:
        return True

    return False
def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")
def crossTrackError(state,way_point):
    fx = state[0] + LENGTH  * np.cos(state[3])
    fy = state[1] + LENGTH  * np.sin(state[3])
    #check if w1 is passed
    dx = fx - way_point[0]
    dy = fy - way_point[1]
    #find angle of the two vectory
    front_axle_vec = [-np.cos(state[3] + np.pi / 2),
                      -np.sin(state[3] + np.pi / 2)]
    error_front_axle = np.dot([dx, dy], front_axle_vec)
    return error_front_axle
def pid_control(xref,x0):
    global pid
    xref = xref[:,0]
    cte = crossTrackError(x0,xref)
    global pid,errorIsum,errorD_last
    errorP = cte
    errorI = errorIsum
    errorD = errorD_last - cte
    error = (pid[0] * errorP) + (pid[1] * errorI) + (pid[2] * errorD)
    errorIsum = errorIsum+error
    errorD_last = error
    # print(error)
    return error,cte
def pid_control_velocity(target, current):
    return 1 * (target - current)
def update_state(state, a, delta):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state. v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state. v < MIN_SPEED:
        state.v = MIN_SPEED

    return state
def do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state):
    
    goal = [cx[-1], cy[-1]]
    state = initial_state
    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]
    error = [0.0]
    cyaw = smooth_yaw(cyaw)
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)
    
    loop = 0
    error_ILC = []
    Nloop = 2
    RC_loop = len(cx) * Nloop
    ref_x_cx = []
    ref_y_cy = []
    for i in range(Nloop+1):
        ref_x_cx  = np.concatenate((ref_x_cx, np.copy(np.array(cx))), axis=None)
        ref_y_cy  = np.concatenate((ref_y_cy, np.copy(np.array(cy))), axis=None)
    error_control_input_x = np.zeros(len(cx)+RC_loop)
    error_control_input_y = np.zeros(len(cx)+RC_loop)
    KRC = 0.1
        # ref_x_new = cx[i+((n)*1771)] + (KRC*error_control_input_x[i+((n)*1771)+1])
        # ref_y_new = cy[i+((n)*1771)] + (KRC*error_control_input_y[i+((n)*1771)+1])
        # irst_half = ref_x_cx[(last*loop):last ]
        #        

    while loop < Nloop:
        last =target_ind
        sup = len(cx)
        if(loop == 0):
            ref_x_x = ref_x_cx[(sup*loop):(sup*loop)+sup]
            ref_y_y =  ref_y_cy[(sup*loop):(sup*loop)+sup]
        else:
            ref_x_x = ref_x_cx[(sup*loop):(sup*loop)+sup]
            ref_y_y =  ref_y_cy[(sup*loop):(sup*loop)+sup]
        xref, target_ind, dref = calc_ref_trajectory(
            state, ref_x_x ,ref_y_y , cyaw, ck, sp, dl, target_ind)
        x0 = [state.x, state.y, state.v, state.yaw]  # current state
        ai = pid_control_velocity(TARGET_SPEED,state.v)
        di,cte = pid_control(xref, x0)
        state = update_state(state, ai, di)
        time = time + DT
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di/3.14*180)
        a.append(ai)
        error.append(cte)
        
        if(last != target_ind):
            # print(len(cx),(sup*(loop+1))+target_ind-1)
            ref_x_cx[(sup*(loop+1))+target_ind-1]  = ref_x_x[target_ind-1] + (KRC*cte)
            ref_y_cy[(sup*(loop+1))+target_ind-1]  = ref_y_y[target_ind-1] + (KRC*cte)
            error_ILC.append(error)
            # print(len(cx),last,(sup*(loop+1))+last)
        if check_goal(state, goal, target_ind, len(cx)):
            # plt.close("all")
            # plt.plot(ref_x_x,ref_y_y, "-r", label="spline")
            # print(ref_x_x[len(ref_x_x)-1],loop)
            #plt.pause(0.01)
            loop += 1
            target_ind = 0
            print("Goal")
        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            # if ox is not None:
            #     plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw, steer=0)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            plt.pause(0.0001)
    
    rmse_val = rmse(np.array(error),np.zeros(len(error)))
    # cyaw = smooth_yaw(cyaw)
    return t, x, y, yaw, v, d, a,rmse_val,error_ILC, 
def main():
    global pid
    print(__file__ + " start!!")
    dl = 0.1  # course tick
    cx, cy, cyaw, ck = get_path(dl)
    
    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)
    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
    t, x, y, yaw, v, d, a , best_err,error = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state)
   
    dp = [0.01,0.0001,0.00001]
    it = 0
    tol = 0.002
    # if(findPidAuto):
    #     while sum(dp) > tol:
    #         print("Iteration {}, best error = {} , pid => {} Dp=> {}".format(it, best_err,pid,dp))
    #         for i in range(len(pid)):
    #             initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
    #             t, x, y, yaw, v, d, a , e,error = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state)
    #             if e < best_err:
    #                 best_err = e
    #                 dp[i] *= 1.1
    #             else:
    #                 pid[i] -= 2 * dp[i]
    #                 initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
    #                 t, x, y, yaw, v, d, a , e,error = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state)
    #                 if e < best_err:
    #                     best_err = e
    #                     dp[i] *= 1.01
    #                 else:
    #                     pid[i] += dp[i]
    #                     dp[i] *= 0.99
    #         it += 1
    #         plt.close("all")
    #         plt.plot(cx, cy, "-r", label="spline")
    #         plt.plot(x, y, "-g", label="tracking")
    #         plt.grid(True)
    #         plt.axis("equal")
    #         plt.xlabel("x[m]")
    #         plt.ylabel("y[m]")
    #         plt.pause(0.1)
    # print(pid)  # pragma: no cover
    plt.close("all")
    plt.plot(cx, cy, "-r", label="spline")
    plt.plot(x, y, "-g", label="tracking")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.pause(1)
    fig, (ax1, ax2) = plt.subplots(2)
    fig.suptitle('Tracking PID')
    ax1.set(xlabel='time(s)', ylabel='cte error (m)')
    ax1.plot(t,error, "-r")
    ax1.plot(t,d, "-b")
    ax1.grid(True)
    ax2.set(xlabel='time(s)', ylabel='steering angle (degree)')
    # ax2.plot(t,error_debug, "-r")
    # ax2.grid(True)
    #ax1.xaxis.set_major_locator(MultipleLocator(5))
    # ax1.yaxis.set_major_locator(MultipleLocator(0.1))
    # #ax1.xaxis.set_minor_locator(AutoMinorLocator(5))
    # ax1.yaxis.set_minor_locator(AutoMinorLocator(5))
    # ax1.grid(which='major', color='#CCCCCC', linestyle='--')
    # #ax1.xaxis.set_major_locator(MultipleLocator(5))
    # ax2.yaxis.set_major_locator(MultipleLocator(20))
    # #ax1.xaxis.set_minor_locator(AutoMinorLocator(5))
    # ax2.yaxis.set_minor_locator(AutoMinorLocator(5))
    # ax2.grid(which='major', color='#CCCCCC', linestyle='--')
    # plt.subplots()
    # plt.plot(t, error, "-r", label="course")
    # plt.grid(True)
    # plt.axis("equal")
    # plt.xlabel("x[m]")
    # plt.ylabel("y[m]")
    # plt.legend()
    
    # plt.subplots()
    # plt.plot(t, v, "-r", label="speed")
    # plt.grid(True)
    # plt.xlabel("Time [s]")
    # plt.ylabel("Speed [kmh]")
    plt.show()
        
if __name__ == '__main__':
    main()