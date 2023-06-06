from typing import Any
import numpy as np
from scipy.spatial.transform import Rotation
from controller_tools.tools import Path_3D,R


def OA(state,vel):
    Rm=Rotation.from_euler('XYZ',angles=state[6:9],degrees=False)
    distances=np.linalg.norm(vel,axis=1)
    # t=(distances>0.4)*(distances<3)*(vel[:,2]>-0.4)
    t=(distances>0.4)*(vel[:,2]>-0.35)
    # vel_front=ptf.local_info(s).R.T@Rm.as_matrix()@(vel.T)
    # vel_front=(vel_front[0]>=-0.5)
    # t=t*vel_front
    vel=vel[t]
    distances=distances[t]
    if len(distances)==0:
        return np.zeros(3),np.zeros(3),Rm
    # vel=vel[np.argmin(distances)]
    # x,y,z=vel.T
    # U=2
    # a=3
    # b=(x**2+y**2)>a**2/2
    # # print(x,y,z)
    # # c=(x**2+y**2)>2.5**2
    # Z=np.array([U*(1-a**2*(x**2-y**2)/((x**2+y**2)**2)),-2*U*a**2*x*y/((x**2+y**2)**2),0*x])
    # # rep=(b*Z+(1-b)*vel.T)/(1+0*distances**2)
    # rep=(b*Z-(1-b)*vel.T)
    # # rep=Z
    # # kvel=0.004
    # # rep=kvel*np.sum(rep,axis=1)
    # kvel=0.001
    # rep=vel.T/distances**3/(1+2*distances**2)
    # rep=-kvel*np.sum(rep.T,axis=0)
    # Vr=state[3:6]
    # v=np.linalg.norm(Vr)
    
    # # c=np.sum(vel*Vr,axis=1)
    # p=np.argmin(distances)
    # closest_point=vel[p]
    # dmin=distances[p]
    # # c=-(np.exp(3*(np.dot(closest_point/dmin,Vr)-1)))*closest_point/dmin
    # # rep=3*c+rep
    # # rep=-closest_point/dmin**2
    # z=np.array([0,0,1])
    # w0=0.75
    # r=dmin
    # w=w0*np.tanh(1/(0.05*r**5))*z
    # # rep=np.cross(w,Vr)+np.tanh(0.5*r**2)*u+np.tanh(0.2/r**2)*np.cross(w,u)
    # # rot=Rotation.from_rotvec(np.array([0,0,1.5*np.tanh(1/r**2)]))
    # rep=np.cross(w,Vr)+np.tanh(0.5*r)*u
    rep=np.zeros(3)
    return rep,vel,Rm


from scipy.optimize import fsolve
from scipy.optimize import newton
from numpy.linalg import norm

class oa_alg:
    def __init__(self,ptf=None,dt=1/30,r0=1,displayer=None):
        self.oa=False
        self.first_time=True
        # self.oa_ptf=Path_3D(np.array([[1,1,1],[1,2,1]]).T,type='waypoints')
        # self.ptf=Path_3D(np.array([[1,1,1],[1,2,1]]).T,type='waypoints')
        self.ptf=ptf
        self.r0=r0
        self.dt=dt
        self.sOA=0
        self.displayer=displayer
        self.i=0
    
    def pfc_eq(self,s):
        F=self.ptf.local_info(s)
        # print(F.X,s,norm(F.X-self.obstacle)-self.r0)
        return norm(F.X-self.obstacle)-self.r0

    def __call__(self,state,s,vel,sptf):
        Rm=Rotation.from_euler('XYZ',angles=state[6:9],degrees=False)
        # distances=np.linalg.norm(vel,axis=1)
        # t=(distances>0.4)*(vel[:,2]>-0.35)
        # vel=vel[t]
        # distances=distances[t]
        # if len(distances)==0:
        #     return np.zeros(3),np.zeros(3),Rm
        X=state[:3]
        Vr=state[3:6]
        
        # p=np.argmin(distances)
        # closest_point=vel[p]
        # d=distances[p]
        obstacles=np.array([[0.1,-2,0.4],[0.1,-4.5,0.4]])
        i=np.argmin(norm(obstacles-X,axis=1))
        closest_point=obstacles[i]
        self.obstacle=closest_point
        d=norm(X-closest_point)
        # print(closest_point)
        r0=1
        eps0=0.3
        F1=self.ptf.local_info(sptf)
        if d<r0+0.35 and F1.R[:,0]@(closest_point-X)>0:
            if self.first_time:
                # print('HEREEE',self.i,s)
                # S1=X+Rm.apply(Vr)/norm(Vr)*0.35*0
                S1=X+F1.R[:,0]*0.35
                s_S2=newton(self.pfc_eq, sptf+2.5*r0)
                F=self.ptf.local_info(s_S2)
                S2=F.X
                r1=S1-closest_point
                r2=S2-closest_point
                dot_prod=np.dot(r1,r2)/(norm(r1)*norm(r2))
                cross_prod=np.sign(np.cross(r1,r2)[-1])
                # print(np.cross(r1,r2))
                th0=np.arccos(dot_prod)*cross_prod
                # print(np.linspace(0,th0,200))
                waypoints=[list(closest_point+R(theta,'z')@r1) for theta in np.linspace(0,th0,200)]
                waypoints=np.array(waypoints).T
                # import matplotlib.pyplot as plt
                # plt.plot(waypoints[0],waypoints[1])
                # plt.plot([0,0],[0,-24])
                # plt.xlim(-2,2)
                # plt.ylim(-,0)
                # plt.quiver(*(closest_point[:2]),*(r1[:2]),color='blue')
                # plt.quiver(*(closest_point[:2]),*(r2[:2]),color='red')
                # plt.savefig('test1.png')
                self.oa_ptf=Path_3D(waypoints,headings=np.zeros(len(waypoints[0])), speeds=np.ones(len(waypoints[0]))*0.5, type='waypoints')
                self.displayer.oa_path.setData(pos=self.oa_ptf.points[:,:3])
                self.oa=True
                self.first_time=False
        # print(self.sOA)
        if self.oa:
            if self.oa_ptf.s_max-self.sOA<eps0:
                # print('OAAAA',self.oa_ptf.s_max,self.sOA)
                self.oa=False
                self.first_time=True
                self.i+=1
                self.sOA=0
                # self.i=np.clip(self.i,0,1,dtype=int)
        return self.oa
        




# O=oa_alg()
# O(np.zeros(12),1,[])
# vel=np.array([[0.5,0.5,0]])
# O(np.zeros(12),vel=vel)
# import matplotlib.pyplot as plt
# fig = plt.figure()
# ax = fig.subplots()
# x,y=O.s1[:2]
# ax.scatter(x,y,c='red')
# x,y=O.s2[:2]
# ax.scatter(x,y,c='red')

# ax.scatter(*O.waypoints[:2],c='red')

# ax.scatter(*vel[0,:2],c='navy')
# c1=plt.Circle(vel[0,:2], 1, color='cornflowerblue',alpha=0.5)

# ax.add_patch(c1)
# ax.grid()
# plt.show()