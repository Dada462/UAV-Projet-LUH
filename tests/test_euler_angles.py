import numpy as np
from scipy.spatial.transform import Rotation
from numpy import tan,cos,sin,pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import expm
from tools import sawtooth

def state(X,u):
    phi,theta,psi=X
    S=np.array([[1,tan(theta)*sin(phi),tan(theta)*cos(phi)],
                [0,cos(phi),-sin(phi)],
                [0,sin(psi)/cos(theta),cos(phi)/cos(theta)]])

    return S@u



def adj(w):
    return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def adj_inv(A):
    return np.array([A[2,1],A[0,2],A[1,0]]) # A is 3x3
# w=np.array([1,1,0])
# w=w/np.linalg.norm(w)
# w=adj(w)
# color=['r','g','b']
# ax.set_xlim([-2, 2])
# ax.set_ylim([-2, 2])
# ax.set_zlim([-1, 1])
# # plt.pause(1)
# for j in range(3):
#     Y=np.eye(3)[:,j]
#     ax.quiver(0,0,0,*Y,color='black')

# for i in np.linspace(-45,45,25):
    
#     # R=Rotation.from_euler('XYZ',angles=angles,degrees=True).as_matrix()
#     # ang=-120
#     # R=expm(w*ang/180*pi)
#     # r=Rotation.from_matrix(R).as_euler('ZYX')
#     # print(r*180/pi)
#     for k in np.linspace(-45,45,25):
#         angles=[0,i,k]
#         R=Rotation.from_euler('ZYX',angles,degrees=True).as_matrix()
#         ax.clear()
#         ax.set_xlim([-2, 2])
#         ax.set_ylim([-2, 2])
#         ax.set_zlim([-1, 1])
#         for j in range(3):
#             Y=R[:,j]
#             ax.quiver(0,0,0,*Y,color=color[j])    
#         plt.pause(0.001)
# plt.show()

w=np.array([1,1,0])
w=w/np.linalg.norm(w)
############################## Lab ##############################
w0=2*pi
dt=0.005
T=np.arange(0,10,dt)
phi=np.zeros((len(T),3))
w_adj=np.array([adj(w*w0*t) for t in T])
R=expm(w_adj)
r1=Rotation.from_matrix(R)
phi[0]=r1.as_euler('xyz')[0]
r=r1.as_euler('xyz')

############################## Lab ##############################
#################################################################
#################################################################
#################################################################
############################## Th ##############################
def R1(phi,theta,psi):
    return np.array([[cos(theta)*cos(psi),-sin(psi),0],
                     [cos(theta)*sin(psi),cos(psi),0],
                     [-sin(theta),0,1]])

for (i,t) in enumerate(T):
    try:
        R4=np.linalg.inv(R1(*phi[i]))
        phi[i+1]=phi[i]+w0*dt*R4@w
    except:
        print(i)
phi=sawtooth(phi)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlim([-2, 2])
# ax.set_ylim([-2, 2])
# ax.set_zlim([-1, 1])
# # print(r.shape)
# r1=Rotation.from_euler('xyz',phi,degrees=False)
# r2=r1.as_matrix()[[5*i for i in range(len(phi)//5)],:,:]
# ax.quiver(*np.zeros_like(r2).T,*r2[:,:,0].T,color='black')
# ax.set_xlabel('X', fontsize=20, rotation=60)
# ax.set_ylabel('Y',fontsize=20, rotation=60)
# ax.set_zlabel('Z', fontsize=20, rotation=60)
# plt.show()
############################## Th ##############################
#################################################################
#################################################################
#################################################################
############################## Results ##############################
fig1 = plt.figure()
ax1 = fig1.add_subplot()
ax1.plot(T,180/pi*r[:,0])
ax1.plot(T,180/pi*phi[:,0])
plt.legend(['X','Y','Z','Xth','Yth','Zth'])
plt.grid()
plt.show()
