import numpy as np
from numpy.linalg import norm
from numpy import pi, cos, sin
from scipy import interpolate
from scipy.spatial.transform import Rotation


def sawtooth(x):
    return (x+pi) % (2*pi)-pi


def R(theta, which='2D'):
    if which == '2D':
        r = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
    elif which == 'x':
        r = np.array([[1, 0, 0], [0, cos(theta), -sin(theta)],
                      [0, sin(theta), cos(theta)]])
    elif which == 'y':
        r = np.array([[cos(theta), 0, -sin(theta)],
                      [0, 1, 0], [sin(theta), 0, cos(theta)]])
    elif which == 'z':
        r = np.array([[cos(theta), - sin(theta), 0],
                      [sin(theta), cos(theta), 0],
                      [0, 0, 1]])
    return r


class pathInfo():
    def __init__(self):
        pass


class Path_3D():
    def __init__(self, *args, **kwargs):
        if len(args) != 0:
            if 'type' not in kwargs:
                raise ValueError(
                    'You must specity a type, either type=\'parametric\' or \'waypoints\'')
            if kwargs['type'] == 'parametric':
                f = args[0]
                if 'range' in kwargs:
                    values_range = kwargs['range']
                else:
                    values_range = [-10, 10]
                t = np.linspace(*values_range, 6000)
                points = f(t)
                self.points = points.T
            elif kwargs['type'] == 'waypoints':
                points = args[0]
                self.points = points.T
            if 'speeds' not in kwargs:
                self.speeds = np.ones(len(points[0]))*0.5
            else:
                self.speeds = kwargs['speeds']
            if 'headings' not in kwargs:
                self.headings = np.zeros(len(points[0]))
            else:
                self.headings = kwargs['headings']
            # self.compute_path_properties()
            # self.compute_path_properties_PTF()

    def __call__(self, s):
        return self.local_info(s)

    def curvature(self, dr, d2r):
        return norm(np.cross(dr, d2r), axis=1)/(norm(dr, axis=1)**3)

    def sawtooth(self, x):
        return (x+pi) % (2*pi)-pi

    def adj(self, w):
        return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

    def adj_inv(self, A):
        return np.array([A[2, 1], A[0, 2], A[1, 0]])

    def compute_path_properties_PTF(self):
        points = self.points.T
        # each column of points is a point
        df = np.gradient(points, axis=1)
        ds = norm(df, axis=0)
        # Checking if the path sent was correct
        ds_check = (ds == 0)
        if ds_check.any():
            print('[WARNING] Two consecutives waypoints are the same, check the path planner')
            problematic_ds=np.where(ds_check)[0]
            points_to_reject=[]
            for i in problematic_ds:
                if i==0 or i == len(points.T)-1:
                    # Add the index to the points to reject
                    points_to_reject.append(i)
                    if i==0:
                        print('[WARNING] First and second points are the same. Only one waypoint will be kept')
                    else:
                        print('[WARNING] The last and second last points are the same. Only one waypoint will be kept')
                else:
                    print('[WARNING] Point {i:d} and {i2:d} are the same. Check the path planner in case it is an error.'.format(i=i,i2=i+2))
                    # Compute ds using normal difference
                    df[:,i]=points[:,i+1]-points[:,i]
                    ds[i]=norm(df[:,i])
            if 0 in points_to_reject:
                df=df[:,1:]
                ds=ds[1:]
                points=points[:,1:]
                self.speeds=self.speeds[1:]
                self.headings=self.headings[1:]
            if len(points.T)-1:
                df=df[:,:-1]
                ds=ds[:-1]
                points=points[:,:-1]
                self.speeds=self.speeds[:-1]
                self.headings=self.headings[:-1]
            print('[INFO] The waypoint error was corrected.')
        ds_check = (ds == 0)
        if ds_check.any():
            print('ERROR')
            path_computed_successfully = False
            return path_computed_successfully
        s = np.cumsum(ds)
        s = s-s[0]
        T = df/ds  # Columns

        d2f_ds = np.gradient(T, axis=1)
        C = norm(d2f_ds/ds, axis=0)
        dC = np.gradient(C)/ds

        # V0=np.cross(np.array([0,0,1]),T[:,0])
        V0 = T[:, 0]
        for x in [np.array([0, 0, 1]), np.array([0, 1, 0]), np.array([1, 0, 0])]:
            if np.linalg.norm(np.cross(x, V0)) > 0.01:
                V0 = np.cross(x, V0)
                V0 = V0/np.linalg.norm(V0)
                break
        N = np.zeros_like(T)
        N[:, 0] = V0

        for i in range(len(T.T)-1):
            B = np.cross(T[:, i], T[:, i+1])
            if np.linalg.norm(B) < 1e-7:
                N[:, i+1] = N[:, i]
            else:
                B = B/np.linalg.norm(B)
                theta = np.arccos(T[:, i]@T[:, i+1])
                r = Rotation.from_rotvec(theta*B).as_matrix()
                N[:, i+1] = r@N[:, i]
            N[:, i+1] = N[:, i+1]/np.linalg.norm(N[:, i+1])

        B = np.cross(T, N, axis=0)

        # Derivatives of the frame with respesct to the curvilinear abscissa
        dT = np.gradient(T, axis=1)/ds
        dN = np.gradient(N, axis=1)/ds
        dB = np.gradient(B, axis=1)/ds

        # Rotation matrix'
        R = np.stack((T.T, N.T, B.T), axis=1)
        dR = np.zeros_like(R)
        dR = np.stack((dT.T, dN.T, dB.T), axis=1)

        R = np.transpose(R, axes=(0, 2, 1))
        dR = np.transpose(dR, axes=(0, 2, 1))

        # Coefficients k1 and k2 from the Parallel Transport Frame Equation & curvature and torsion from SFF
        y1 = np.gradient(T, axis=1)
        y1 = y1/(norm(y1, axis=0)+1e-6)
        w1 = np.cross(T, y1, axis=0)
        dw1 = np.gradient(w1, axis=1)/ds

        Tr = -np.sum(dw1*y1, axis=0)
        dTr = np.gradient(Tr)/ds

        theta = np.cumsum(Tr*ds)
        k1 = C*np.cos(theta)
        k2 = C*np.sin(theta)

        self.s_to_XYZ = interpolate.interp1d(s, points)

        self.s_to_T = interpolate.interp1d(s, T)
        self.s_to_N = interpolate.interp1d(s, N)
        self.s_to_B = interpolate.interp1d(s, B)

        self.s_to_dT = interpolate.interp1d(s, dT)
        self.s_to_dN = interpolate.interp1d(s, dN)
        self.s_to_dB = interpolate.interp1d(s, dB)

        self.s_to_C = interpolate.interp1d(s, C)
        self.s_to_dC = interpolate.interp1d(s, dC)
        self.s_to_Tr = interpolate.interp1d(s, Tr)
        self.s_to_dTr = interpolate.interp1d(s, dTr)
        self.s_to_k1 = interpolate.interp1d(s, k1)
        self.s_to_k2 = interpolate.interp1d(s, k2)

        self.s_to_R = interpolate.interp1d(s, R, axis=0)
        self.s_to_dR = interpolate.interp1d(s, dR, axis=0)

        self.ds = ds
        self.s = s
        self.s_max = np.max(s)  # Length of the path
        self.s_to_w1 = interpolate.interp1d(s, w1)

        # Interpolation of the speeds and headings
        self.s_to_speeds = interpolate.interp1d(s, self.speeds)
        self.s_to_headings = interpolate.interp1d(s, self.headings)

        self.s_to_df = interpolate.interp1d(s, T)
        print('got hereeeeeeeeeeeeeeeeeeeeeeeeeeeeee')
        path_computed_successfully = True
        return path_computed_successfully

    def local_info(self, s):
        local_property = pathInfo()

        s = np.clip(s, 0, self.s_max)
        local_property.s = s
        local_property.X = self.s_to_XYZ(s)

        local_property.C = self.s_to_C(s)
        local_property.dC = self.s_to_dC(s)
        local_property.Tr = self.s_to_Tr(s)
        local_property.dTr = self.s_to_dTr(s)
        local_property.k1 = self.s_to_k1(s)
        local_property.k2 = self.s_to_k2(s)

        local_property.T = self.s_to_T(s)
        local_property.N = self.s_to_N(s)
        local_property.B = self.s_to_B(s)

        local_property.dT = self.s_to_dT(s)
        local_property.dN = self.s_to_dN(s)
        local_property.dB = self.s_to_dB(s)

        local_property.R = self.s_to_R(s)
        local_property.dR = self.s_to_dR(s)

        local_property.w1 = self.s_to_w1(s)
        local_property.speed = self.s_to_speeds(s)
        local_property.heading = self.s_to_headings(s)

        return local_property


if __name__ == '__main__':
    import pyqtgraph as pg
    pg.setConfigOptions(antialias=True)
    plot = pg.plot(pen={'color': '#0e70ec', 'width': 2}, background='w')
    plot.resize(1200, 850)
    plot.move(300, 115)
    scatter = pg.ScatterPlotItem(pen={'color': '#0e70ec', 'width': 2})
    plot.addItem(scatter)

    # f=lambda t : R(0.15,'x')@np.array([1*(1+0.25*np.sin(4*t))*np.cos(t),1*(1+0.25*np.sin(4*t))*np.sin(t),0*t+0.5])
    # p=Path_3D(lambda t : np.array([t**2,-10+t,10+0*t]),[-20,20],type='parametric')
    # f=lambda t : np.array([cos(6*t),sin(6*t),t**2])
    # f=lambda t : np.array([cos(t),sin(t),t*cos(t)])
    # f=lambda t : np.array([3*(1.5+np.sin(6*t))*np.cos(t),3*(1.5+np.sin(6*t))*np.sin(t),0*t+1])

    # 1: Line
    def line(t): return np.array([2*t, -t, 0*t+1.25])
    line_range = (-1, 1)
    # 2: U-Turn
    n = 6
    a, b = 1, 4

    n = 6
    a, b = 1, 4

    def uturn(t): return np.array(
        [-2+b*np.sign(np.sin(-t))*((1-np.cos(t)**n)**(1/n))+1, -a*np.cos(t), 0*t+0.5])
    uturn_range = (-pi, 0)
    # def uturn(t): return np.array(
    #     [1*np.cos(t), np.sign(np.sin(t))*(r**n-(r*np.cos(t))**n)**(1/n), 0*t+1.5])

    # 3: Obstacle avoidance 1
    def obs_av_1(t):
        b = np.array([0, 16, 0])
        X = uturn(t)*(t < pi/2)+(2*uturn(pi/2)-uturn(pi-t)) * \
            (1.5*pi > t >= pi/2)+(b+uturn(t))*(1.5*pi <= t)
        return X
    obs_av_1_range = (0, 2*pi)

    # 4: Obstacle avoidance 2
    T = 1
    A = 0.8
    def obs_av_2(t): return np.array([t, A*np.sin(2*pi*t/T), 0*t+1.5])
    obs_av_2_range = (0, 3)
    # 5: Circular

    def circular(t): return np.array(
        [5*np.cos(t)*np.sin(0.5*t), 3*np.sin(t), 1.5])
    circular_range = (0, 2*pi)

    # Other paths
    # p=Path_3D(f,range=[0,6],type='parametric')
    # p=Path_3D(lambda t : (2+sin(10*t))*np.array([cos(t),sin(t),0*t+1]),range=[-10,-9],type='parametric')
    # f=lambda t : np.array([t,t,0*t])
    # f=lambda t : np.array([np.cos(t),np.sin(t),0*t+1.5])

    rng = uturn_range
    f = uturn
    points = []
    for t in np.linspace(*rng, 350):
        points.append(f(t))
    points = np.array(points).T
    # print(points.shape)
    p = Path_3D(points, headings=np.ones(
        len(points[0]))*5, speeds=np.ones(len(points[0]))*69, type='waypoints')
    p.compute_path_properties_PTF()
    # p=Path_3D(f,range=[0,2*pi],type='parametric')
    F = p.local_info(p.s)
    # obstacle=np.array([-0.467792,-2.359100,0.4])
    obstacle = np.array([[2.359100, -0.467792, 0.4]])
    y = np.abs(norm(F.X.T-obstacle, axis=1)-1.5)

    z = F.X.T[y < 0.05]
    z1 = F.s[y < 0.05].reshape((-1, 1))
    print('Lenght:', np.round(p.s_max, 2), 'm')
    from sklearn.cluster import KMeans
    kmeans = KMeans(n_clusters=2)
    kmeans.fit(z1)
    # Get the cluster labels for each point
    labels = kmeans.labels_
    # Separate points into two sets based on the cluster labels
    points_set1 = z1[labels == 0].flatten()
    points_set2 = z1[labels == 1].flatten()

    points_set1 = p.local_info(points_set1).X.T
    points_set2 = p.local_info(points_set2).X.T
    # print(time()-t0)
    # print(points_set1.shape)
    # print(points_set1.shape)
    points = np.array(
        [p.local_info(2.5).X, p.local_info(7).X, obstacle[0], *points_set2])
    t = np.linspace(0, 2*pi, 100)
    circle = 1.5*np.array([np.cos(t), np.sin(t), t*0])+obstacle.T

    scatter.setData(pos=points)
    plot.plot(F.X[0], F.X[1], pen={'color': '#0e70ec', 'width': 2})
    # plot.plot(x=circle[0],y=circle[1], pen={'color': '#0e70ec', 'width': 2})
    # scatter.setData(pos=obstacle.reshape((-1,3)))
    # plot.plot(F.s, y, pen={'color': '#0e70ec', 'width': 2})

    # s = p.s
    # k1 = p.s_to_k1(p.s)
    # k2 = p.s_to_k2(p.s)
    # C = p.s_to_C(p.s)
    # Tr = p.s_to_Tr(p.s)
    # T = p.s_to_T(s)
    # N = p.s_to_N(s)
    # B = p.s_to_B(s)

    # dT = k1*N+k2*B
    # dN = -k1*T
    # dB = -k2*T

    # Rpath = p.s_to_R(s)[:, :, 0]
    # dRpath = p.s_to_dR(s)[:, :, 2]

    plot.showGrid(x=True, y=True)
    plot.show()
    pg.exec()
