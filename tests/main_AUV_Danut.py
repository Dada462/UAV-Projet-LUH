from cmath import atan
from numpy import cos, real, sin, tanh, arctan, arctan2, pi
import numpy as np
import matplotlib.pyplot as plt
from tools import rungeKutta2, sawtooth, R, path_info_update, draw_crab, show_info,mat_reading,mat_reading1,R3



def state(X, controller):
    x,y,theta,s= X
    Vt=controller[:2]
    dtheta=controller[2]
    return np.hstack((R(theta)@Vt,dtheta,u[3]))


def controller_Non_Col(x):
    #For the case when at least two motors are not colinear
    X = x[0:2]
    theta_m = x[2]
    s = x[3]
    F = path_info_update(path_to_follow, s)
    theta_c = F.psi
    theta = sawtooth(theta_m-theta_c)
    s1, y1 = R(theta_c).T@(X-F.X)
    Kdy1=1
    psi_a=pi/2
    delta = -psi_a*tanh(Kdy1*y1)
    beta=delta+theta_c-theta_m
    psi = sawtooth(theta+beta)
    nu=1
    ks = 1
    ds = cos(psi)*nu + ks*s1
    theta_m_d=pi/4
    dtheta_m_d=0
    k2=1
    dtheta_m=dtheta_m_d+k2*(theta_m_d-theta_m)
    u,v=nu*cos(beta),nu*sin(beta)
    return np.array([u,v,dtheta_m,ds])


alpha2=pi/2
alpha3=-pi/2
L=1
r=0.25 #Size of the robot on the drawings, purely for drawing purposes

# Drawing and window info
dt, w_size, w_shift = 0.01, 15, 0
fig, ax = plt.subplots(figsize=(8, 7))

# Lists to stock the information. For viewing resulsts after the simulation
# T is the list temporal values, state_info[i] is the state x(t_i) where t_i=T[i]
T, state_info = [], []
t_break = 0  # it will be the time at which the simulation stops

# Initial conditions
px0, py0 = -4, 7 #Initial position
u0, v0 = 1,0 #Initial speed
theta0, dtheta0 = 0, 0 # Initial orientation and angular velocity
s0 = 100

# mat_reading(lambda t : 5*np.array([cos(t),sin(0.9*t)])) # The path the robot has to follow
# path_to_follow = mat_reading(lambda t : 5*np.array([cos(t),sin(0.9*t)])) # The path the robot has to follow  # The path the robot has to follow
path_to_follow=mat_reading(lambda t : 2.5*(2+sin(10*t))*np.array([cos(t),sin(t)])) # The path the robot has to follow
# path_to_follow = mat_reading(lambda a,b : 5+14*np.array([cos(a),sin(0.9*b)]))  # The path the robot has to follow
# path_to_follow = mat_reading(lambda a,b : 5+14*np.array([cos(0.9*a),0.5*sin(0.9*b)]))  # The path the robot has to follow
# path_to_follow = mat_reading(lambda a,b : (a**2+b**2)**.5*np.array([0.5*a,sin(2*b)]))  # The path the robot has to follow
# path_to_follow = mat_reading(lambda x,y : (x+17,2*x))  # The path the robot has to follow
x0 = np.array([px0, py0,theta0,s0 ]) # (x,y,vu,vv,s,theta_m,dtheta_m)
path = [x0[0:2]]  # Red dots on the screen, path that the robot follows
x = x0
draw, end_of_path = 1, 0

for t in np.arange(0, 1000, dt):
    if end_of_path:
        print("End of simulation")
        break
    # Drawing and screen update
    if draw and (t/dt) % 15 == 0 and t!=0:
        X = x[0:2]
        theta_m= x[2]
        s = x[3]
        if (t/dt) % 45 == 0:
                path.append(X)
        ax.clear()
        ax.set_xlim(-w_size-w_shift, w_size-w_shift)
        ax.set_ylim(-w_size, w_size)
        show_info(ax, path_to_follow, X, np.zeros(2), theta_m, u, [0,alpha2,alpha3, 0.5, 0.25], [
                s, t], path, [w_size, w_shift], forces=0, speed=0)
        draw_crab(X, theta_m, ax, 0.5, 0.25)
        plt.pause(10**-6)
    # Update of the state of the simulation
    t_break = t
    T.append(t)
    u=controller_Non_Col(x)
    # u=np.zeros(4)
    # u[0]=1
    # u[2]=1
    state_info.append(x)
    #To control the robot using a game controller
    x = rungeKutta2(x, u, dt, state)
    x[3] = max(0, x[3])
    x[2] = sawtooth(x[2])
