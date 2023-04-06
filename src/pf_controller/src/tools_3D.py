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

def R_multi(theta, which='2D'):
    if which == '2D':
        r=np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
    elif which == 'x':
        r=np.array([[np.ones_like(theta), np.zeros_like(theta), np.zeros_like(theta)], [np.zeros_like(theta), cos(theta), -sin(theta)],
                 [np.zeros_like(theta), sin(theta), cos(theta)]])
    elif which == 'y':
        r=np.array([[cos(theta), np.zeros_like(theta), -sin(theta)],
                  [np.zeros_like(theta), np.ones_like(theta), np.zeros_like(theta)], [sin(theta), np.zeros_like(theta), cos(theta)]])
    elif which == 'z':
        r=np.array([[cos(theta),- sin(theta), np.zeros_like(theta)],
                 [sin(theta), cos(theta), np.zeros_like(theta)],
                 [np.zeros_like(theta), np.zeros_like(theta), np.ones_like(theta)]])
    r=np.transpose(r.T,[0,2,1])
    return r

def sawtooth(x):
    return (x+pi) % (2*pi)-pi   # or equivalently   2*arctan(tan(x/2))


class pathInfo():
    def __init__(self,s=None,C=None,dC=None,psi=None,X=None,s1=None,y1=None,w1=None,dR=None,Tr=None):
        self.s=s
        self.C=C
        self.dC=dC
        self.psi=psi
        self.s1=s1
        self.y1=y1
        self.w1=w1
        self.X=X
        self.dR=dR
        self.Tr=Tr

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

        d2f_ds=np.gradient(df/ds,axis=1)
        C=norm(d2f_ds/ds,axis=0)
        dC=np.gradient(C)/ds
        psi=np.arctan2(df[1],df[0])
        
        s1=df/ds
        y1=np.gradient(s1,axis=1)
        y1=y1/(norm(y1,axis=0)+1e-6)
        w1=np.cross(s1,y1,axis=0)

        
        ds1=np.gradient(s1,axis=1)/ds
        dy1=np.gradient(y1,axis=1)/ds
        dw1=np.gradient(w1,axis=1)/ds
        # print(norm(s1,axis=0).all()==1.0)
        
        n=ds1.shape[1]
        dR=np.zeros((n,3,3))
        # for i in range(n):
        #     dR[i]=np.array([ds1[:,i],dy1[:,i],dw1[:,i]]).T
            # print(dR[i])
            # print(ds1[:,i],dy1[:,i],dw1[:,i])

        dR=np.stack((ds1.T,dy1.T,dw1.T),axis=1)
        dR=np.transpose(dR,axes=(0,2,1))

        # R=np.stack((s1.T,y1.T,w1.T),axis=1)
        # R=np.transpose(dR,axes=(0,2,1))
        # print(np.round(R[0]@dR[0],2))

        Tr=-np.sum(dw1*y1,axis=0)
        # Tr=np.sqrt(dw1[0]**2+dw1[1]**2+dw1[2]**2)
        # Tr=norm(dw1,axis=0)
        # pg.plot(s,Tr,pen={'color': '#186ff6', 'width': 2},background='w')
        # pg.exec()
        # print(dw1[:,0],s1[:,0])

        self.s=s
        s_to_psi=interpolate.interp1d(s, psi)
        s_to_XYZ=interpolate.interp1d(s, points)
        s_to_C=interpolate.interp1d(s, C)
        s_to_dC=interpolate.interp1d(s, dC)
        s_to_s1=interpolate.interp1d(s, s1)
        s_to_y1=interpolate.interp1d(s, y1)
        s_to_w1=interpolate.interp1d(s, w1)
        s_to_dsyw=interpolate.interp1d(s, dR,axis=0)
        s_to_Tr=interpolate.interp1d(s, Tr)

        self.s_to_C=s_to_C
        self.s_to_dC=s_to_dC
        self.s_to_psi=s_to_psi
        self.s_to_XY=s_to_XYZ
        self.s_to_s1=s_to_s1
        self.s_to_y1=s_to_y1
        self.s_to_w1=s_to_w1
        self.s_to_dsyw=s_to_dsyw
        self.s_to_Tr=s_to_Tr
    
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
        s_to_Tr=self.s_to_Tr(s)
        local_property=pathInfo(s,C,dC,psi,XYZ,s1,y1,w1,s_to_dsyw,s_to_Tr)
        return local_property


if __name__=='__main__':
    import pyqtgraph as pg
    pg.setConfigOptions(antialias=True)
    def f(t):
        x = np.cos(t)
        y = np.sin(3*t)
        z=10*t
        return np.array([x,y,z])

    points=f(np.linspace(-10,10, 5000))
    X,Y,Z=points
    # p=Path(points,type='waypoints')
    # p=Path(f,[-10,10],type='parametric')
    # p=Path_3D(lambda t : np.array([a*cos(t),a*sin(t),b*t]),[0,10],type='parametric')
    p=Path_3D(lambda t : np.array([5*cos(t),5*sin(0.9*t),15+0*t]),[-10,10],type='parametric')
    # p=Path_3D(lambda t : np.array([5*cos(t),5*sin(2*t),0*t+15]),[0,15],type='parametric')

    # f=lambda t : R(0.5*t,'x')@np.array([5*cos(t),5*sin(0.9*t),0*t])+15
    # points=[]
    # for t in np.linspace(-10,10,4000):
    #     points.append(f(t))
    # points=np.array(points).T

    # p=Path_3D(points,type='waypoints')
    
    # t=np.linspace(-10,10,10)
    # print(R_multi(0.5*t,'x')@(np.array([t,0*t,0*t])).shape)


    plot=pg.plot(pen={'color': '#186ff6', 'width': 2},background='w')

    F=p.local_info(p.s)
    plot.plot(F.s,F.C,pen={'color': 'blue', 'width': 2})
    # plot.plot(F.s,F.psi,pen={'color': 'red', 'width': 2})


    plot.showGrid(x=True,y=True)
    plot.show()
    pg.exec()