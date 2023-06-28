import numpy as np
from scipy.spatial.transform import Rotation
from controller_tools.tools import Path_3D, R
from scipy.optimize import newton,fsolve
from numpy.linalg import norm
from numpy import pi
import rospy
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import KMeans
def OA(state, vel):
    Rm = Rotation.from_euler('XYZ', angles=state[6:9], degrees=False)
    distances = np.linalg.norm(vel, axis=1)
    # t=(distances>0.4)*(distances<3)*(vel[:,2]>-0.4)
    t = (distances > 0.4)*(vel[:, 2] > -0.35)
    # vel_front=ptf.local_info(s).R.T@Rm.as_matrix()@(vel.T)
    # vel_front=(vel_front[0]>=-0.5)
    # t=t*vel_front
    vel = vel[t]
    distances = distances[t]
    if len(distances) == 0:
        return np.zeros(3), np.zeros(3), Rm
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
    rep = np.zeros(3)
    return rep, vel, Rm


class oa_alg:
    def __init__(self, ptf=None, dt=1/30, r0=1.5, pfc=None):
        self.oa = False
        self.first_time = True
        self.ptf = ptf
        self.pfc=pfc
        self.r0 = r0
        self.dt = dt
        self.sOA = 0
        # self.displayer = displayer
        self.i = 0
        self.current_obstacle = np.zeros(3)+np.inf
        self.path_pub = rospy.Publisher('/path/points/oa', Float32MultiArray, queue_size=1)
        self.kmeans = KMeans(n_clusters=2)
        

    def pfc_eq(self, s):
        # s=np.clip(s,0,self.ptf.s_max)
        F = self.ptf.local_info(s)
        return norm(F.X-self.obstacle)-self.r0

    def __call__(self, state, s, vel, sptf):
        Rm = Rotation.from_euler('XYZ', angles=state[6:9], degrees=False)
        distances = np.linalg.norm(vel, axis=1)
        t = (distances > 0.4)*(vel[:, 2] > -0.35)
        vel = vel[t]
        distances = distances[t]
        if len(distances) == 0:
            return False
        X = state[:3]
        Vr = state[3:6]
        # v=norm(Vr)

        p = np.argmin(distances)
        closest_point = Rm.apply(vel[p])+X
        d = distances[p]
        self.obstacle = closest_point
        self.pfc.closest_obstacle_distance=d
        r0 = self.r0
        eps0 = 0.2
        F1 = self.ptf.local_info(sptf)
        dt=1/30
        preventive_distance=np.min(norm(vel+Vr*dt,axis=1))
        if d < r0+0.35 and (F1.R[:, 0]@(closest_point-X) > 0) or preventive_distance<r0:
        # if d < r0+0.35 and (F1.R[:, 0]@(closest_point-X) > 0):
            if self.first_time:
                self.compute_path([X, closest_point, F1, sptf, r0],first_time=True)
            elif norm(self.current_obstacle-closest_point) > 0.2:
                self.compute_path([X, closest_point, F1, sptf, r0],first_time=False)
            points=self.oa_ptf.points.T
            path_points = Float32MultiArray()
            path_points.data = points.flatten()
            self.path_pub.publish(path_points)
        if self.oa:
            if self.oa_ptf.s_max-self.sOA < eps0:
            # print('ptf points',self.oa_ptf.points.shape)
            # distances_to_path=norm(X-self.ptf.points,axis=1)
            # print(distances_to_path.shape)
            # dist_to_path=np.min(distances_to_path)
            # if dist_to_path < 0.05:
                self.oa = False
                self.first_time = True
                self.i += 1
                self.sOA = 0
        return self.oa

    def compute_path(self, vars, first_time):
        X, closest_point, F1, sptf, r0 = vars
        # S1=X+F1.R[:,0]*0.35
        # S1=self.ptf.local_info(sptf+0.35).X
        # if self.first_time and norm(self.ptf.local_info(sptf).X-X)>2:
        #     S1=F.X

        # theta = np.zeros((40, 3))
        # theta[:, -1] = np.linspace(0, 2*pi, 40)
        # rot_obstacle = Rotation.from_rotvec(theta)
        # r1=np.array([r0,0,0])
        # waypoints = closest_point+rot_obstacle.apply(r1)
        # points = waypoints.T
        # path_points = Float32MultiArray()
        # path_points.data = points.flatten()
        # self.path_pub.publish(path_points)

        # f1=self.ptf.s_to_df
        # f=lambda s: 2*(self.ptf.s_to_XYZ(s)-self.obstacle)@f1(s)
        F=self.ptf.local_info(np.linspace(0,self.ptf.s_max,500))
        y=np.abs(norm(F.X.T-self.obstacle,axis=1)-r0)
        # z=F.X.T[y<0.1]
        z=F.s[y<0.15].reshape((-1,1))
        self.kmeans.fit(z)
        # Get the cluster labels for each point
        labels = self.kmeans.labels_
        # Separate points into two sets based on the cluster labels
        points_set1 = z[labels == 0].flatten()
        points_set2 = z[labels == 1].flatten()
        s_S1=points_set1[0]
        s_S2=points_set2[0]
        if s_S1>s_S2:
            s_S1p=s_S2
            s_S2=s_S1
            s_S1=s_S1p
        
        S1 = self.ptf.local_info(s_S1).X
        S2 = self.ptf.local_info(s_S2).X
        # if norm(S1-X)>norm(S2-X):
        #     S1p=S2.copy()
        #     S2=S1.copy()
        #     S1=S1p

        # s_S1 = newton(self.pfc_eq, max(0.1,sptf-0.5*r0),tol=1e-2,maxiter=10)
        # # s_S1 = fsolve(self.pfc_eq, max(0,sptf-0.5*r0))
        # s_S1=s_S1
        # F = self.ptf.local_info(s_S1)
        # S1 = F.X
        # s_S2 = newton(self.pfc_eq, min(sptf+2.5*r0,self.ptf.s_max-0.1),tol=1e-2,maxiter=10)
        # # s_S2 = fsolve(self.pfc_eq, min(sptf+2.5*r0,self.ptf.s_max))
        # s_S2=s_S2
        # F = self.ptf.local_info(s_S2)
        # S2 = F.X
        # print(s_S1,s_S2)


        r1 = S1-closest_point
        r2 = S2-closest_point
        self.pfc.s=s_S2
        dot_prod = np.dot(r1, r2)/(norm(r1)*norm(r2))
        dot_prod=np.clip(dot_prod,-1,1)
        angle_sign = np.sign(np.cross(r1, r2)[-1])
        th0 = np.arccos(dot_prod)*angle_sign
        if first_time:
            self.rot_dir=angle_sign
        else:
            if self.rot_dir*angle_sign<0:
                th0=th0*self.rot_dir
                th0=th0%(2*pi)
                th0=th0*self.rot_dir
        theta = np.zeros((40, 3))
        theta[:, -1] = np.linspace(0, th0, 40)
        rot_obstacle = Rotation.from_rotvec(theta)
        waypoints = closest_point+rot_obstacle.apply(r1)
        waypoints = waypoints.T
        self.oa_ptf = Path_3D(waypoints, headings=np.zeros(
            len(waypoints[0])), speeds=np.ones(len(waypoints[0]))*0.5, type='waypoints')
        self.oa_ptf.compute_path_properties_PTF()
        self.pfc.sOA=self.oa_ptf.s_max/2
        self.oa = True
        self.first_time = False
        self.current_obstacle = closest_point


