import numpy as np
from scipy.spatial.transform import Rotation
from controller_tools.tools import Path_3D
from controller_tools.rrt_opti import Graph
from controller_tools.oa_pkg_3D import Tree
from scipy.optimize import newton,fsolve
from numpy.linalg import norm
from numpy import pi
import rospy
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import KMeans
# from controller_tools.rrt_3D.rrt_connect3D import rrt_connect
# from controller_tools.rrt_3D.env3D import env
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

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
        # self.rrt=rrt_connect()

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
        
        self.vels=Rm.apply(vel)+X
        p = np.argmin(distances)
        # closest_point = Rm.apply(vel[p])+X
        closest_point = self.vels[p]
        # self.vels=closest_point
        d= self.obstacle_distance = distances[p]
        self.obstacle = closest_point
        self.pfc.closest_obstacle_distance=d
        r0 = self.r0
        eps0 = 0.2
        F1 = self.ptf.local_info(sptf)
        dt=1/30
        preventive_distance=np.min(norm(vel+Vr*dt,axis=1))
        # if d < r0+0.35 and (F1.R[:, 0]@(closest_point-X) > 0) or preventive_distance<r0:
        
        # if d < r0+0.35 and (F1.R[:, 0]@(closest_point-X) > 0):
        #     if self.first_time:
        #         self.compute_path([X, closest_point, F1, sptf, r0],first_time=True)
        #     elif norm(self.current_obstacle-closest_point) > 0.2:
        #         self.compute_path([X, closest_point, F1, sptf, r0],first_time=False)

        if d < r0+0.35 and (F1.R[:, 0]@(closest_point-X) > 0):
        # if self.first_time:
            if self.first_time:
                self.compute_path_1([X, closest_point, F1, sptf, r0],first_time=True)
            elif norm(self.current_obstacle-closest_point) > 0.2:
                self.compute_path_1([X, closest_point, F1, sptf, r0],first_time=False)
            # self.compute_path_1([X, closest_point, F1, sptf, r0])
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
    
    def compute_path_1(self, vars,first_time):
        X, closest_point, F1, sptf, r0 = vars
        # F=self.ptf.local_info(np.linspace(0,self.ptf.s_max,500))
        # y=np.abs(norm(F.X.T-self.obstacle,axis=1)-r0)
        # z=F.s[y<0.15].reshape((-1,1))
        # self.kmeans.fit(z)
        # # Get the cluster labels for each point
        # labels = self.kmeans.labels_
        # # Separate points into two sets based on the cluster labels
        # points_set1 = z[labels == 0].flatten()
        # points_set2 = z[labels == 1].flatten()
        # s_S1=points_set1[0]
        # s_S2=points_set2[0]
        # if s_S1>s_S2:
        #     s_S1p=s_S2
        #     s_S2=s_S1
        #     s_S1=s_S1p
        
        # S1 = self.ptf.local_info(s_S1).X
        # S2 = self.ptf.local_info(s_S2).X
        if self.obstacle_distance<r0:
            S1 = self.obstacle+(X-self.obstacle)/self.obstacle_distance*(r0+0.25)
        else:
            S1 = X
        S2=np.array([0.0, -4.5, 0.4])
        S1,S2=np.round(S1,2),np.round(S2,2)
        S1=tuple(S1)
        S2=tuple(S2)
        # radiuses=1.1*r0*np.ones(len(self.vels)).reshape((-1,1))
        # obstacles=np.hstack((self.vels,radiuses))
        # self.rrt.__init__(env(*(X[:2]-10),0.35,*(X[:2]+10),2),stepsize=0.15,maxiter=250)
        # self.rrt.__init__(env(-25,-25,0.35,25,25,2),stepsize=0.2,maxiter=250)
        # self.rrt.env.balls=obstacles
        # self.rrt.qinit=self.rrt.env.start=S1
        # self.rrt.qgoal=self.rrt.env.goal=S2
        # result=self.rrt.RRT_CONNECT_PLANNER(self.rrt.qinit, self.rrt.qgoal)
        t=Tree(start=S1,end=S2,obstacles=self.vels,safety_radius=1)
        t.rrt(search_space=[-10,10,-10,10,0.5,2])
        path=t.dijkstra()
        path=t.smoothen_path(path,iterations=10)
        # print(path.shape)
        # print(path)

        # S1=tuple(S1)
        # S2=tuple(S2)
        # G=Graph(S1,S2)
        # print(G.startpos,G.endpos,S1,S2)
        # n_iter = 125
        # radius = 1
        # stepSize = 0.35
        # print('params',self.vels.reshape((1,-1)),S1,S2)
        # print('params',self.vels.reshape((1,-1)),S1,S2)
        # G.RRT_star(self.vels, n_iter, radius, stepSize)
        # waypoints=G.smoothen_path()
        # patha=self.rrt.patha
        # pathb=self.rrt.pathb

        # patha,pathb=np.array(patha),np.array(pathb)
        # patha=self.rearange_path(patha)
        # pathb=self.rearange_path(pathb)
        # path=self.connect_paths(patha,pathb,X).T

        # s=np.linspace(0,1,len(path[0]))
        # path=interp1d(s,path)
        # s=np.linspace(0,1,50)
        # path=path(s)
        # path[0]=savgol_filter(path[0], window_length=35,polyorder=3)
        # path[1]=savgol_filter(path[1], window_length=35,polyorder=3)
        # path[2]=savgol_filter(path[2], window_length=35,polyorder=3)
        waypoints=path.T

        self.oa_ptf = Path_3D(waypoints, headings=np.zeros(
            len(waypoints[0])), speeds=np.ones(len(waypoints[0]))*0.25, type='waypoints')
        self.oa_ptf.compute_path_properties_PTF()
        self.pfc.sOA=self.oa_ptf.s_max/2
        self.oa = True
        self.first_time = False
        self.current_obstacle = closest_point

    def rearange_path(self,p):
        # check
        if (p[1:,0,:]==p[:-1,1,:]).all():
            pass # do not change the path
        elif (p[:-1,0,:]==p[1:,1,:]).all():
            print('permuted everything')
            p=p[:,::-1,:]
        else:
            print('something is wrong')
            return p
        p=np.vstack((p[:,0,:],p[-1,1,:]))
        return p
    
    def connect_paths(self,patha,pathb,X):
        eps=0.5
        if np.linalg.norm(patha[-1]-pathb[0])<eps:
            # join paths
            connected_path=np.vstack((patha,pathb))
        elif np.linalg.norm(patha[0]-pathb[0])<eps:
            # flip the path
            connected_path=np.vstack((patha[::-1],pathb))
        else:
            print('[ERROR 1] There is a problem, check the path or the threshold')
            return connected_path
        
        connected_path
        if np.linalg.norm(connected_path[0]-X)<eps:
            # leave the path as it is
            return connected_path
        elif np.linalg.norm(connected_path[-1]-X)<eps:
            # flip the path
            return connected_path[::-1]
        else:
            print('[ERROR 2] There is a problem, check the path or the threshold',np.linalg.norm(connected_path[-1]-X),np.linalg.norm(connected_path[0]-X))
            return connected_path

