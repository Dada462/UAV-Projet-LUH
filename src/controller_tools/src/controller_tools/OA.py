import numpy as np
from scipy.spatial.transform import Rotation
from controller_tools.tools import Path_3D
from controller_tools.oa_pkg_3D import Tree, Circle
from numpy.linalg import norm
from numpy import pi
import rospy
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import KMeans
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
from collections import deque


class oa_alg:
    def __init__(self, ptf=None, r0=1, pfc=None):
        global OA, ATCP, RCP, OAPC, STOPPED, FOLLOWPATH
        ATCP, OA, STOPPED = 'ATCP', 'OA', 'STOPPED'
        RCP, OAPC, FOLLOWPATH = 'RCP', 'OAPC', 'FOLLOWPATH'
        self.oa = False
        self.ptf = ptf
        self.pfc = pfc
        self.r0 = r0
        self.sOA = 0
        self.it_was_called=False
        self.path_pub = rospy.Publisher(
            '/path/points/oa', Float32MultiArray, queue_size=1)
        self.s_progress = 0
        self.compute_circular_rays()

    def __call__(self, state, sOA, vel, sptf):
        self.s_progress = np.max([self.s_progress, sptf])
        Rm = Rotation.from_euler('XYZ', angles=state[6:9], degrees=False)
        distances = np.linalg.norm(vel, axis=1)
        t = (distances > 0.4)*(vel[:, 2] > -0.35)
        vel = vel[t]
        distances = distances[t]
        if len(distances) == 0:
            return False
        X = state[:3]
        Vr = state[3:6]
        V=Rm.apply(Vr)
        self.vels = Rm.apply(vel)+X
        p = np.argmin(distances)
        d = self.obstacle_distance = distances[p]
        self.pfc.closest_obstacle_distance = d

        dt = 0.1  # Prediction window. It predict ahead for ~ 100 ms

        # Check the path ahead for about ~1m
        end_position_not_OF = False
        obstacles_circles = Circle(self.vels, self.r0)
        dS_ahead = 1  # 1m
        if not self.oa:
            # Check the normal path
            s_ahead = np.linspace(sptf, np.clip(
                sptf+dS_ahead, sptf, self.ptf.s_max), 10)  # check every 10 cm
            path_ahead = self.ptf.local_info(s_ahead).X.T
        else:
            # Check the OA path
            s_ahead = np.linspace(sOA, np.clip(
                sOA+dS_ahead, sOA, self.oa_ptf.s_max), 10)  # check every 10 cm
            path_ahead = self.oa_ptf.local_info(s_ahead).X.T
            end_position = self.oa_end_position.reshape((1, -1))
            end_position_not_OF = (
                obstacles_circles.points_in_circles(end_position)).any()
        path_not_OF = (obstacles_circles.points_in_circles(path_ahead).any())
        predicted_position = (X+dt*V).reshape((1, -1))
        predicted_position_not_OF = (
            obstacles_circles.points_in_circles(predicted_position)).any()
        # if path_ahead is not OF=Obstacle Free or the drone is entering the obstacle zone
        if path_not_OF | predicted_position_not_OF | end_position_not_OF or self.it_was_called:
            self.it_was_called=True
            self.pfc.sm.is_safe_zone = not predicted_position_not_OF
            if not self.oa:
                self.pfc.sm.userInput = OA
            else:
                self.pfc.sm.userInput = RCP
            if predicted_position_not_OF and not self.pfc.sm.safe_pos_computed:
                A,_=self.compute_start_end(X,getting_out=True)
                self.target=A
                self.pfc.sm.safe_pos_computed=True
            elif not predicted_position_not_OF:
                self.pfc.sm.safe_pos_computed=True
            if self.pfc.sm.state != ATCP:
                return False
            oa_path_succes = self.compute_path(X)
            self.it_was_called=False
            if not oa_path_succes:
                return False
            self.pfc.sm.userInput = OAPC
            points = self.oa_ptf.points.T
            path_points = Float32MultiArray()
            path_points.data = points.flatten()
            self.path_pub.publish(path_points)
        if self.oa:
            finish_threshold = 0.2  # 20 cm
            if self.oa_ptf.s_max-self.sOA < finish_threshold:
                self.oa = False
                self.sOA = 0
                self.pfc.sm.userInput = FOLLOWPATH
                self.pfc.sm.safe_pos_computed=False
                # self.it_was_called=False
        return self.oa

    def compute_path(self, X):
        self.path_error = 0.15  # 15 cm
        S1, S2 = self.compute_start_end(X)
        S1 = tuple(S1)
        S2 = tuple(S2)
        t = Tree(start=S1, end=S2, obstacles=self.vels,
                 safety_radius=self.r0+self.path_error)
        path_found = t.rrt(
            search_space=[X[0]-5, X[0]+5, X[1]-5, X[1]+5, 0.25, X[2]+0.5])
        if not path_found:
            # Cancel the PF and call for a new path
            print('OA failed to computed a path.')
            self.pfc.pathAction.oa_failed = True
            self.oa = False
            return False

        path = t.dijkstra()
        path = t.smoothen_path(path, iterations=7)
        waypoints = path.T

        self.oa_ptf = Path_3D(waypoints, headings=np.zeros(
            len(waypoints[0])), speeds=np.ones(len(waypoints[0]))*0.25, type='waypoints')
        self.oa_ptf.compute_path_properties_PTF()
        self.pfc.sOA = 0
        self.oa = True
        self.first_time = False
        return True

    def compute_start_end(self, X,getting_out=False):
        radius = self.r0+self.path_error
        # Computing A: find the points OF from the positions. The latest OF point is A
        positions = X.reshape((1, -1))
        circles = Circle(self.vels, radius)
        of_list = np.flip(circles.points_in_circles(positions))

        A_idx = ([i for i, e in enumerate(np.flip(of_list)) if not e])
        if len(A_idx) != 0:
            A_idx = A_idx[0]
            A = positions[A_idx]
        else:
            X = positions[-1]
            rays = self.rays+X
            result = circles.points_in_circles(rays)
            result = np.logical_not(result)
            rays = rays[result]
            A = rays[len(rays)//2]

        # Computing B
        # Check all the curvilinear abscissas that were passed
        # Look ahead by ~2m
        # Take the closest Y=F(s) such that |Y-A|>20cm and OF

        s_progress = self.s_progress  # should look only ~ 2m ahead
        s = np.linspace(s_progress, self.ptf.s_max, 50)
        points = self.ptf.local_info(s).X.T
        of_list = np.logical_not(circles.points_in_circles(
            points)) & (norm(points-A, axis=1) > 1)

        B_idx = [i for i, e in enumerate(of_list) if e][0]
        B = points[B_idx]
        if not getting_out:
            self.pfc.s = s[B_idx]
            self.oa_end_position = B
        return A, B

    def compute_circular_rays(self, nb_of_rays=20):
        rot_vecs = np.zeros((nb_of_rays, 3))
        rot_vecs[:, -1] = np.linspace(0, 2*np.pi, nb_of_rays, endpoint=False)
        r = Rotation.from_rotvec(rot_vecs)
        self.rays = self.r0*r.apply(np.array([1, 0, 0]))
