import pyqtgraph as pg
# import pyqtgraph.examples as e
# e.run()
import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
from numpy.linalg import norm


plt.rcParams["figure.figsize"] = (8,8)
T=np.linspace(-pi,pi,50)

nb=0.1
# Q=np.array([(0.25+T)/6*np.cos(T),(0.25+T)/6*np.sign(np.sin(T))*(1-np.cos(T)**nb)**(1/nb)])
# Q=np.array([np.cos(T),np.sign(np.sin(T))*(1-np.cos(T)**nb)**(1/nb)])
theta=T
# r=1/(np.abs(np.cos(theta)+np.sin(theta))+np.abs(np.cos(theta)-np.sin(theta)))
r=1
# t1=np.linspace(pi/2,3*pi/2,2)
# plt.plot(r*np.cos(t1),r*np.sin(t1),color='orangered',linewidth=3)
Q=np.array([r*np.cos(theta),r*np.sin(theta)])
# Q=np.array([[1,1]]).T
# def rep(X,q):
#     U=1
#     a=2
#     x,y=(X-q).T
#     b=(x**2+y**2)>=a**2
#     # b=1
#     Z=np.array([U*(1-a**2*(x**2-y**2)/((x**2+y**2)**2)),-2*U*a**2*x*y/((x**2+y**2)**2)])
#     Z=b*Z+(1-b)*(X-q).T
#     return Z

def rep(X,q):
    distances=np.linalg.norm(X-q,axis=1)
    rep=(X-q).T/distances**2
    # rep=np.sum(rep,axis=1)
    return rep


x = np.linspace(-5.1,5,50)
y = np.linspace(-5.1,5,50)


plt.grid()
X, Y = np.meshgrid(x, y)


points=np.vstack((X.reshape(-1),Y.reshape(-1)))
u,v=np.zeros_like(points)
for i in range(len(Q.T)):
    u1,v1=rep(points.T,Q[:,i])
    u,v=u1+u,v1+v

u=u.reshape(X.shape)
v=v.reshape(Y.shape)



# plt.scatter(*Q,s=5,color='orangered')
color_array = np.sqrt((u)**2 + (v)**2)

# plt.streamplot(X, Y, u, v,color='cornflowerblue')
# plt.quiver(X, Y, u, v,color_array)
from controller_tools.tools import R
closest_point=np.zeros(3)
r1=np.array([1,0,0])
th0=pi/4
# waypoints=[list(closest_point+R(theta,'z')@r1) for theta in np.linspace(0,th0,40)]
theta=np.linspace(0,th0,40)
# theta=list(theta)
from scipy.spatial.transform import Rotation
r=Rotation.from_rotvec(np.array([np.zeros_like(theta),np.zeros_like(theta),theta]).T)
r2=r.apply(r1)
print(r2)
# waypoints=np.array(waypoints).T
waypoints=r2.T
print(waypoints.shape)
plt.plot(waypoints[0],waypoints[1])
plt.plot([0,0],[0,-24])
plt.xlim(-1,2)
plt.ylim(-1,2)
a=np.zeros((5,11,19))
b=np.zeros((19,11))
# print((a@b).shape)
# plt.quiver(*(closest_point[:2]),*(r1[:2]),color='blue')
# for r in r2:
#     plt.quiver(*(closest_point[:2]),*(r[:2]),color='red')
# plt.quiver(*(closest_point[:2]),*(r2[:2]),color='red')
# t=np.linspace(0,pi,200)
# plt.plot(1.5*np.cos(t)-3,1.5*np.sin(t))
# plt.colorbar()
# from controller_tools.tools import R,sawtooth
# n=15
# r=np.linspace(0,2,n)
# r0=1
# b=pi/2*(np.tanh((r-r0))+1)
# print(b)
# X=np.array([np.linspace(0,2,n),np.zeros(n)])
# for theta in np.linspace(-pi,pi,100):
#     x,y=R(theta)@X
#     s=-np.sign(np.sin(theta))
#     u,v=R(theta)@(R(b*s)[:,0,:])
#     # if np.abs(theta)>=pi/3:
#     # else:
#     #     u,v=R(theta)@(R(b*s)[:,0,:])
#     #     u[r>=r0]=-u[r>=r0]
#     # u=-s*u
#     plt.quiver(x,y,u,v,color='cornflowerblue')
plt.show()


