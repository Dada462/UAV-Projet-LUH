import scipy.io
import numpy as np
import matplotlib.pyplot as plt
from numpy import cos, sin, pi,arctan2
import keyboard


def rungeKutta2(x, u, h, f):
    x = x + h*(0.5*f(x, u) + 0.5*f(x + h*f(x, u), u))
    return x

def rect(x,a):
    if abs(x)<=a:
        return x
    else:
        return a


def sawtooth(x):
    return (x+pi) % (2*pi)-pi   # or equivalently   2*arctan(tan(x/2))


class path_data:
    def __init__(self, X, psi, s, C_c,dC_c):
        self.s = s
        self.psi = psi
        self.C_c = C_c
        self.dC_c = dC_c
        self.X = X
    def __str__(self):
        return str(self.s) + ' ' + str(self.psi) + ' ' + str(self.C_c) + ' ' + str(self.X)


def mat_reading1(path='PATH.mat'):  # Return the path for follow info: s,X,psi,C_c
    mat = scipy.io.loadmat(path)

    s = []
    for i in range(len(mat['Chemin']['s'][0, :])):
        s.append(mat['Chemin']['s'][0, :][i][0][0])
    s = np.array(s)

    X = np.array([[], []])
    for i in range(len(mat['Chemin']['X'][0, :])):
        X = np.hstack((X, mat['Chemin']['X'][0, :][i]))
    X = X.T

    psi = []
    for i in range(len(mat['Chemin']['psi'][0, :])):
        psi.append(mat['Chemin']['psi'][0, :][i][0][0])
    psi = np.array(psi)

    C_c = []
    for i in range(len(mat['Chemin']['C_c'][0, :])):
        C_c.append(mat['Chemin']['C_c'][0, :][i][0][0])
    C_c = np.array(C_c)
    path_to_follow = path_data(X, psi, s, C_c,0)
    return path_to_follow

def default_path(X,Y):
    return 5+9*np.array([cos(X),sin(0.9*Y)])

def mat_reading(f=default_path,r=[-10,10]):
    def sum(X):
        s = 0
        if len(X)!=0 and len(X)!=1:
            for x in X:
                s += x
        return s
    n=4000
    n=n+3

    T=np.linspace(*r,n)
    points=f(T)
    X,Y=points[0],points[1]
    ds=np.array([((X[i+1]-X[i])**2+(Y[i+1]-Y[i])**2)**0.5 for i in range(0,n-1)])
    S=np.array([sum(ds[0:i]) for i in range(n)])

    dX=np.array([X[i+1]-X[i] for i in range(0,n-1)])
    dY=np.array([Y[i+1]-Y[i] for i in range(0,n-1)])
    
    psi=arctan2(dY,dX)
    # L=[]
    # for i in range(len(psi)-1):
    #     if abs(psi[i+1]-psi[i])>=6.2:
    #         L.append(i)
    # for k in range(len(L)-1):
    #     psi[L[k]+1:L[k+1]+1]=psi[L[k]+1:L[k+1]+1]+(k+1)*2*pi
    # psi[L[-1]+1:len(psi)]=psi[L[-1]+1:len(psi)]+(len(L)+1)*2*pi
    dpsi=np.array([sawtooth(psi[i+1]-psi[i]) for i in range(0,n-2)])
    psi=np.array([sum(dpsi[0:i]) for i in range(n-2)])
    psi=np.hstack((psi,arctan2(dY[-1],dX[-1])))
    psi=psi+arctan2(dY[0],dX[0])

    ds=list(ds)
    ds.pop()
    ds=np.array(ds)
    C=dpsi/ds
    ds=list(ds)
    ds.pop()
    ds=np.array(ds)
    dC=np.array([C[i+1]-C[i] for i in range(0,n-3)])
    dC=dC/ds

    X=list(X)
    X.pop()
    X.pop()
    X.pop()
    X=np.array(X)
    
    Y=list(Y)
    Y.pop()
    Y.pop()
    Y.pop()
    Y=np.array(Y)


    psi=list(psi)
    psi.pop()
    psi.pop()
    psi=np.array(psi)

    S=list(S)
    S.pop()
    S.pop()
    S.pop()
    S=np.array(S)

    C=list(C)
    C.pop()
    C=np.array(C)

    XY=np.array([X,Y]).T
    # print(np.shape(XY),np.shape(psi),np.shape(S),np.shape(C),np.shape(dC))
    path_to_follow = path_data(XY, psi, S, C,dC)
    return path_to_follow


def find(elements, s):
    inds = [i for (i, val) in enumerate(elements) if val > s]
    if inds != []:
        return inds[0]
    else:
        return -1


def path_info_update(path_info_last, s):
    I = find((path_info_last.s), s)
    path_info = path_data(0, 0, 0, 0,0)

    Delta_S_Lievre = (path_info_last.s)[I]-(path_info_last.s)[I-1]
    ratio_S_Lievre = (s-path_info_last.s[I-1])/Delta_S_Lievre
    path_info.s = s
    path_info.psi = path_info_last.psi[I-1] * \
        (1-ratio_S_Lievre) + path_info_last.psi[I]*(ratio_S_Lievre)
    path_info.C_c = path_info_last.C_c[I-1] * \
        (1-ratio_S_Lievre) + path_info_last.C_c[I]*(ratio_S_Lievre)
    path_info.X = path_info_last.X[I-1] * \
        (1-ratio_S_Lievre) + path_info_last.X[I]*(ratio_S_Lievre)
    path_info.dC_c=path_info_last.dC_c[I-1] * \
        (1-ratio_S_Lievre) + path_info_last.dC_c[I]*(ratio_S_Lievre)
    return path_info


def R(theta):
    return np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
def R3(theta):
     return np.array([[cos(theta), -sin(theta),0], [sin(theta), cos(theta),0],[0,0,1]])

def draw_crab(P, theta, ax, L, r, wheel_size=0.5):
    theta_deg = 180*theta/pi-90
    body = plt.Circle(P, L, edgecolor='#1F6EE1', facecolor='none')
    wheel1 = plt.Rectangle(P+R(theta-pi/2)@(L*np.array([0, 1])-np.array(
        [r/2, 0])), r, wheel_size, angle=theta_deg, linewidth=1, edgecolor='g', facecolor='none')
    wheel2 = plt.Rectangle(P+R(theta-pi/2+2*pi/3)@(L*np.array([0, 1])-np.array(
        [r/2, 0])), r, wheel_size, linewidth=1, angle=theta_deg+120, edgecolor='r', facecolor='none')
    wheel3 = plt.Rectangle(P+R(theta-pi/2-2*pi/3)@(L*np.array([0, 1])-np.array(
        [r/2, 0])), r, wheel_size, linewidth=1, angle=theta_deg-120, edgecolor='r', facecolor='none')

    ax.add_patch(body)
    ax.add_patch(wheel1)
    ax.add_patch(wheel2)
    ax.add_patch(wheel3)


def show_info(ax, path_to_follow, P, Vt, theta, u, robot_parameters, sim_parameters, path, window_parameters, forces=True, speed=True):
    _, alpha2, alpha3, L, r = robot_parameters
    w_size, w_shift = window_parameters
    s, t = sim_parameters
    ax.plot(*path_to_follow.X.T, c='#3486F4')

    #Drawing of the forces

    if forces:
        ax.quiver(*(P+R(theta)@R(0)@np.array([L, 0])), *R(
            theta-pi/2)@np.array([u[0], 0]), color='red', scale=5000)
        ax.quiver(*(P+R(theta)@R(alpha2)@np.array([L, 0])), *R(
            theta-pi/2+alpha2)@np.array([u[1], 0]), color='red', scale=5000)
        ax.quiver(*(P+R(theta)@R(alpha3)@np.array([L, 0])), *R(
            theta-pi/2+alpha3)@np.array([u[2], 0]), color='red', scale=5000)
    #Drawing the speed
    if speed:
        ax.quiver(*P, *R(theta)@Vt, color='red', scale=10)
    #Drawing point Q
    ax.scatter(*path_info_update(path_to_follow, s).X, c='#34F44C')
    ax.plot(*np.array(path).T,'--',c='red')
    info = 'u1 : '+str(round(u[0], 2))+'\n'+'u2 : ' + \
        str(round(u[1], 2)) + '\n'+'u3 : '+str(round(u[2], 2))
    # ax.text(w_size/2, w_size+0.5, info, style='italic',
    #         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
    # ax.text(0, w_size+1, 'time :' + str(round(t, 2)) + ' s',
    #         style='italic', bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 10})
    # if t>t0:
    #     ax.text(w_size, w_size+0.5,'MOTOR DEAD', style='italic',bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})


def key_press():
    global end_of_path
    alpha = 1
    key = np.array([0, 0, 0])
    if keyboard.is_pressed("up"):
        key[0] = alpha
    elif keyboard.is_pressed("down"):
        key[0] = -alpha
    else:
        key[0] = 0
    if keyboard.is_pressed("a"):
        key[1] = alpha
    elif keyboard.is_pressed("e"):
        key[1] = -alpha
    else:
        key[1] = 0

    if keyboard.is_pressed("left"):
        key[2] = alpha
    elif keyboard.is_pressed("right"):
        key[2] = -alpha
    else:
        key[2] = 0
    if keyboard.is_pressed("space"):
        end_of_path = 1
    return key