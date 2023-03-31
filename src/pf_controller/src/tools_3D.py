import numpy as np
from numpy.linalg import norm
from numpy import pi,cos,sin
from scipy import interpolate

def R(theta, which='2D'):
    if which == '2D':
        r=np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
    elif which == 'x':
        r=np.array([[1, 0, 0], [0, cos(theta), -sin(theta)],
                 [0, sin(theta), cos(theta)]])
    elif which == 'y':
        r=np.array([[cos(theta), 0, -sin(theta)],
                  [0, 1, 0], [sin(theta), 0, cos(theta)]])
    elif which == 'z':
        r=np.array([[cos(theta),- sin(theta), 0],
                 [sin(theta), cos(theta), 0],
                 [0, 0, 1]])
    return r

def sawtooth(x):
    return (x+pi) % (2*pi)-pi   # or equivalently   2*arctan(tan(x/2))


class pathInfo():
    def __init__(self,s=None,C=None,dC=None,psi=None,X=None,s1=None,y1=None,w1=None,dR=None):
        self.s=s
        self.C=C
        self.dC=dC
        self.psi=psi
        self.s1=s1
        self.y1=y1
        self.w1=w1
        self.X=X
        self.dR=dR

class Path_3D():
    def __init__(self,*args,**kwargs):
        if 'type' not in kwargs:
            raise ValueError('You must specity a type, either type=\'parametric\' or \'waypoints\'')
        if kwargs['type']=='parametric':
            f=args[0]
            if len(args)!=1:
                values_range=args[1]
            else:
                values_range=[-10,10]
            t = np.linspace(*values_range, 4000)
            points=f(t)
            self.points=points.T
        elif kwargs['type']=='waypoints':
            points=args[0]
            self.points=points.T
        self.compute_path_properties()
    
    def __call__(self,s):
        return self.local_info(s)

    def curvature(self,dr,d2r):
        return norm(np.cross(dr,d2r),axis=1)/(norm(dr,axis=1)**3)
    
    def sawtooth(self,x):
            return (x+pi) % (2*pi)-pi
    
    def compute_path_properties(self):
        points=self.points.T
        df = np.gradient(points,axis=1)
        ds=norm(df,axis=0)
        s=np.cumsum(ds)
        s=s-s[0]
        self.s_max=np.max(s)
        # df=np.vstack((df,np.zeros_like(df[0])))
        # d2f=np.gradient(df,axis=1)
        # dt=(np.max(t)-np.min(t))*len(t)
        # C=self.curvature(df.T/dt,d2f.T/(dt**2))
        d2f_ds=np.gradient(df/ds,axis=1)
        C=norm(d2f_ds/ds,axis=0)
        dC=np.gradient(C)/ds
        psi=np.arctan2(df[1],df[0])
        
        s1=df/ds
        y1=np.gradient(s1,axis=1)
        y1=y1/norm(y1,axis=0)
        w1=np.cross(s1.T,y1.T).T
        
        ds1=np.gradient(s1,axis=1)/ds
        dy1=np.gradient(y1,axis=1)/ds
        dw1=np.gradient(w1,axis=1)/ds
        
        dR=np.stack((ds1.T,dy1.T,dw1.T),axis=1)
        dR=np.transpose(dR,axes=(0,2,1))

        self.s=s
        s_to_psi=interpolate.interp1d(s, psi)
        s_to_XYZ=interpolate.interp1d(s, points)
        s_to_C=interpolate.interp1d(s, C)
        s_to_dC=interpolate.interp1d(s, dC)
        s_to_s1=interpolate.interp1d(s, s1)
        s_to_y1=interpolate.interp1d(s, y1)
        s_to_w1=interpolate.interp1d(s, w1)
        s_to_dsyw=interpolate.interp1d(s, dR,axis=0)

        self.s_to_C=s_to_C
        self.s_to_dC=s_to_dC
        self.s_to_psi=s_to_psi
        self.s_to_XY=s_to_XYZ
        self.s_to_s1=s_to_s1
        self.s_to_y1=s_to_y1
        self.s_to_w1=s_to_w1
        self.s_to_dsyw=s_to_dsyw
    
    def local_info(self,s):
        s=np.clip(s,0,self.s_max)
        C=self.s_to_C(s)
        dC=self.s_to_dC(s)
        psi=self.s_to_psi(s)
        XYZ=self.s_to_XY(s)
        s1=self.s_to_s1(s)
        y1=self.s_to_y1(s)
        w1=self.s_to_w1(s)
        s_to_dsyw=self.s_to_dsyw(s)
        local_property=pathInfo(s,C,dC,psi,XYZ,s1,y1,w1, s_to_dsyw)
        return local_property


if __name__=='__main__':
    import pyqtgraph as pg
    pg.setConfigOptions(antialias=True)
    def f(t):
        x = np.cos(t)
        y = np.sin(3*t)
        z=0*t
        return np.array([x,y,z])

    points=f(np.linspace(-10,10, 5000))
    X,Y,Z=points
    # p=Path(points,type='waypoints')
    # p=Path(f,[-10,10],type='parametric')
    p=Path_3D(lambda t : 5*np.array([cos(t),sin(0.9*t),0*t]),[-10,10],type='parametric')

    plot=pg.plot(pen={'color': '#186ff6', 'width': 2},background='w')

    F=p.local_info(p.s)
    # plot.plot(F.X[0],F.X[1],pen={'color': 'blue', 'width': 2})
    plot.plot(F.s,F.C,pen={'color': 'green', 'width': 2})
    

    # path=mat_reading(f)
    # plot.plot(path.s,np.abs(path.C_c),pen={'color': 'green', 'width': 2})
    plot.showGrid(x=1,y=1)
    plot.show()
    pg.exec()