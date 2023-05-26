import pyqtgraph as pg
# import pyqtgraph.examples as e
# e.run()
import numpy as np
from numpy import pi
# from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
import matplotlib.pyplot as plt
from numpy.linalg import norm
from Map import Map

print(1/np.inf)

class plotter():
    def __init__(self,X0,f,u):
        pg.setConfigOptions(antialias=True)
        self.p = pg.plot(pen={'color': '#0e70ec', 'width': 2}, background='w')
        self.p.resize(1200, 850)
        self.p.move(300, 115)
        self.p.showGrid(x=True,y=True)
        self.p.setAspectLocked(lock=True, ratio=1)
        
        self.s1 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 120))
        self.p.addItem(self.s1)

        self.counter=0
        self.T=np.linspace(0,2*pi,1000)
        self.updating_plot=self.p.plot(pen={'color': '#0e70ec', 'width': 2})
        self.timer = QtCore.QTimer()
        self.dt=0.075
        self.X=X0
        self.f=f
        self.u=u
        
        
        self.timer.setInterval(int(1000*self.dt))
        self.timer.timeout.connect(self.update_plot_data)
        # self.timer.start()
        
    
    def plot(self,X,Y,color='#0e70ec',width=2):
        self.p.plot(x=X, y=Y, pen={'color': color, 'width': width})
    
    def scatter(self):
        n = 300
        pos = np.random.normal(size=(2,n), scale=1)
        spots = [{'pos': pos[:,i], 'data': 1} for i in range(n)] + [{'pos': [0,0], 'data': 1}]
        self.s1.addPoints(spots)
    def update_plot_data(self):
        self.updating_plot.setData(np.sin(self.counter/10)*np.cos(self.T),np.sin(self.counter/10)*np.sin(self.T))
        self.X=self.X+self.dt*self.f(self.X,self.u(self.X))
        self.counter+=1

# def f(X,u):
#     return 0

# def u(X):
#     return

X0=0

plt.rcParams["figure.figsize"] = (8,8)
dt=0.1
Q=np.array([[1.15,-1.15],
            [1.15,-1.15]])

# T=np.linspace(pi/2,3*pi/2,1)
T=np.linspace(pi/2,3*pi/2,1)


nb=0.1
# Q=np.array([(0.25+T)/6*np.cos(T),(0.25+T)/6*np.sign(np.sin(T))*(1-np.cos(T)**nb)**(1/nb)])
# Q=np.array([np.cos(T),np.sign(np.sin(T))*(1-np.cos(T)**nb)**(1/nb)])
theta=T
# r=1/(np.abs(np.cos(theta)+np.sin(theta))+np.abs(np.cos(theta)-np.sin(theta)))
r=0.5
# t1=np.linspace(0,2*pi,100)
# plt.plot(1.5*np.cos(t1),1.5*np.sin(t1),color='orangered',linewidth=3)
Q=np.array([r*np.cos(theta)-0.5,r*np.sin(theta)])

def rep(X,q):
    # return (X-q).T/(0.01+norm(X-q,axis=1)**3)
    U=1
    a=1.5
    x,y=(X-q).T
    return U*(1-a**2*(x**2-y**2)/((x**2+y**2)**2)),-8*U*a**2*x*y/((x**2+y**2)**2)

x = np.linspace(-3.1,3.1,100)
y = np.linspace(-3.1,3.1,100)


plt.grid()
X, Y = np.meshgrid(x, y)


points=np.vstack((X.reshape(-1),Y.reshape(-1)))
# plt.scatter(*points,c='red')
# points=m.X_to_Map(points)
# r=m.X_to_Map(np.array([-0.7,1.5]))
# plt.scatter(*points,c='blue')
# plt.show()
# print(points.shape)
u,v=np.zeros_like(points)
for i in range(len(Q.T)):
    u1,v1=rep(points.T,Q[:,i])
    u,v=u1+u,v1+v

# from scipy.spatial.transform import Rotation
# r=Rotation.from_rotvec(np.array([0,0,pi]))

# V=np.vstack((u,v,np.zeros_like(u))).T
# V=r.apply(V)
# u,v,_=V.T
u=u.reshape(X.shape)
v=v.reshape(Y.shape)
# def f(x,y):
#     U=2
#     a=0.5
#     return U*1-a**2*(x**2-y**2)/((x**2+y**2)**2),-(U*a)**2*x*y/((x**2+y**2)**2)
# Z=X+1j*Y
# def f(Z):
#     U=1
#     a=0.5
#     return U*Z+U*a**2/Z

# Z=f(Z)
# u=np.real(Z)
# v=np.imag(Z)
# u = u / np.sqrt(u**2 + v**2)
# v = v / np.sqrt(u**2 + v**2)


plt.scatter(*Q,s=5,color='orangered')
# a=0
# T=np.linspace(0,2*pi,250)
# circle=np.array([np.cos(T),np.sin(T)]).T
# for q in Q.T:
    # a=a+1/norm(points.T-q,axis=1)
    # #plt.plot(*(circle+q).T)
# plt.imshow(a.reshape((50,50)))
color_array = np.sqrt((u)**2 + (v)**2)


plt.streamplot(X, Y, u, v,color='cornflowerblue')
# plt.quiver(X, Y, u, v,color_array)
plt.colorbar()
plt.show()

# for t in np.arange(0,10,dt):
#     plt.cla()
#     plt.xlim(-2,2)
#     plt.ylim(-2,2)
#     # rep()

#     plt.pause(1e-6)



# import sympy
# from sympy.abc import x, y

# def cylinder_stream_function(U=1, R=1):
#     r = sympy.sqrt(x**2 + y**2)
#     theta = sympy.atan2(y, x)
#     return U * (r - R**2 / r) * sympy.sin(theta)

# def velocity_field(psi):
#     u = sympy.lambdify((x, y), psi.diff(y), 'numpy')
#     v = sympy.lambdify((x, y), -psi.diff(x), 'numpy')
#     return u, v

# import numpy as np


# def f(x,y):
#     U=1
#     a=1
#     c=np.array([U*1-a**2*(x**2-y**2)/((x**2+y**2)**2),-(U*a)**2*x*y/((x**2+y**2)**2)])
#     x=x-1
#     y=y-1
#     a=0.5
#     c=c+np.array([U*1-a**2*(x**2-y**2)/((x**2+y**2)**2),-(U*a)**2*x*y/((x**2+y**2)**2)])
#     return c



# def plot_streamlines(ax, u, v, xlim=(-1, 1), ylim=(-1, 1)):
#     x0, x1 = xlim
#     y0, y1 = ylim
#     Y, X =  np.ogrid[y0:y1:500j, x0:x1:500j]
#     # ax.streamplot(X, Y, u(X, Y), v(X, Y), color='cornflowerblue')
#     print(f(X,Y).shape)
#     ax.streamplot(X, Y, *f(X,Y), color='cornflowerblue')


# def format_axes(ax):
#     ax.set_aspect('equal')
#     ax.figure.subplots_adjust(bottom=0, top=1, left=0, right=1)

#     ax.xaxis.set_ticks([])
#     ax.yaxis.set_ticks([])
#     ax.axis('off')

# import matplotlib.pyplot as plt

# psi = cylinder_stream_function()
# u, v = velocity_field(psi)

# xlim = ylim = (-3, 3)
# fig, ax = plt.subplots(figsize=(4, 4))
# plot_streamlines(ax, u, v, xlim, ylim)

# c = plt.Circle((0, 0), radius=1, facecolor='none')
# ax.add_patch(c)

# format_axes(ax)
# plt.show()


