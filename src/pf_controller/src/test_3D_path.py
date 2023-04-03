from tools_3D import R,Path_3D
import numpy as np
from numpy import pi,cos,sin
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
import pyqtgraph.opengl as gl
from scipy.linalg import expm,logm




import pyqtgraph.examples
pyqtgraph.examples.run()

def state(X,u):
    return u




pg.setConfigOptions(antialias=True)
app = pg.mkQApp("GLLinePlotItem Example")
w = gl.GLViewWidget()
w.show()
w.setWindowTitle('pyqtgraph example: GLLinePlotItem')
w.setCameraPosition(distance=40)
gx = gl.GLGridItem()
gx.rotate(90, 0, 1, 0)
gx.translate(-10, 0, 0)
w.addItem(gx)
gy = gl.GLGridItem()
gy.rotate(90, 1, 0, 0)
gy.translate(0, -10, 0)
w.addItem(gy)
gz = gl.GLGridItem()
gz.translate(0, 0, -10)
w.addItem(gz)

def f(t):
    x = 5*np.cos(t)
    y = 5*np.sin(0.9*t)
    z=15+t
    return np.array([x,y,z])

p=Path_3D(f,[-10,10],type='parametric')
F=p.local_info(p.s)
pts=F.X.T
plt = gl.GLLinePlotItem(pos=pts, color='red', width=3, antialias=True)


pos=np.zeros(3)
X=np.zeros(4)
sp1 = gl.GLScatterPlotItem(pos=pos,size=0.5,color=(0.2,0.96,0.25,1), pxMode=False)
robot = gl.GLScatterPlotItem(pos=X[:3],size=0.5,color=(0.5,0.96,0.25,1), pxMode=False)

w.addItem(plt)
w.addItem(sp1)
w.addItem(robot)


# create the arrow item
s1_arrow = gl.GLLinePlotItem(width=3, color=(0, 0, 1, 0.8))
y1_arrow = gl.GLLinePlotItem(width=3, color=(0, 1, 0, 0.8))
w1_arrow = gl.GLLinePlotItem(width=3, color=(1, 0.5, 0.5, 0.8))

w.addItem(s1_arrow)
w.addItem(y1_arrow)
w.addItem(w1_arrow)


t=0

dt=0.075


def adj(w):
    return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def adj_inv(A):
    return np.array([A[2,1],A[0,2],A[1,0]])

def controller(state):
    X = state[:3]
    s=state[-1]

    F=p.local_info(s)
    Rpath=np.vstack((F.s1,F.y1,F.w1)).T
    s1, y1, w1 = Rpath.T@(X-F.X)
    
    
    psi_a=pi/2
    e=np.array([s1,y1,w1])
    cross_e=np.cross(np.array([1,0,0]),e)
    delta=-psi_a*np.tanh(cross_e)
    Rpsi=expm(adj(delta))
    Rs=Rpath@Rpsi
    nu=2
    Vr=Rs@np.array([nu,0,0])

    ks=3
    ds = (np.array([nu,0,0])@Rpsi)[0]+ks*s1

    dX=np.array([*Vr,ds])
    
    return dX


from time import time
t0=time()
def update_plot_data():
    global pts,t,pos,s1_arrow,y1_arrow,F,X,t0
    F=p.local_info(X[-1])
    pos=F.X.T
    s1=np.vstack((pos,pos+2*F.s1))
    y1=np.vstack((pos,pos+2*F.y1))
    w1=np.vstack((pos,pos+2*F.w1))
    u=controller(X)
    X[-1]=max(0,X[-1])
    
    sp1.setData(pos=pos)
    s1_arrow.setData(pos=s1)
    y1_arrow.setData(pos=y1)
    w1_arrow.setData(pos=w1)
    robot.setData(pos=X[:3])
    X=X+dt*state(X,u)

timer = QtCore.QTimer()
timer.setInterval(75)
timer.timeout.connect(update_plot_data)
timer.start()
pg.exec()