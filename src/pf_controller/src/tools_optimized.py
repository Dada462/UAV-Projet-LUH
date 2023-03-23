import numpy as np
from numpy.linalg import norm
from numpy import pi
from scipy import interpolate

class pathInfo():
    def __init__(self,s=None,C=None,dC=None,psi=None,XY=None):
        self.s=s
        self.C=C
        self.dC=dC
        self.psi=psi
        self.XY=XY

class Path():
    def __init__(self,points):
        self.points=points
        self.compute_path_properties()
    
    def __call__(self,s):
        return self.local_info(s)

    def curvature(self,dr,d2r):
        return norm(np.cross(dr,d2r),axis=1)/(norm(dr,axis=1)**3)
    
    def sawtooth(self,x):
            return (x+pi) % (2*pi)-pi
    
    def compute_path_properties(self):
        points=self.points
        df = np.gradient(points,axis=1)
        ds=norm(df,axis=0)
        s=np.cumsum(ds)
        s=s-s[0]
        df=np.vstack((df,np.zeros_like(df[0])))
        d2f=np.gradient(df,axis=1)

        dt=(np.max(t)-np.min(t))*len(t)
        C=self.curvature(df.T/dt,d2f.T/(dt**2))
        dC=np.gradient(C)/ds
        psi=np.arctan2(df[1],df[0])
        
        #########################################################
        # psi1=sawtooth(psi[1:]-psi[:-1])
        # psi1=np.cumsum(psi1)
        # psi=np.hstack((psi1,psi[-1]))
        # s_to_t = interpolate.interp1d(s, t)
        #########################################################
        
        self.s=s
        s_to_psi=interpolate.interp1d(s, psi)
        s_to_XY=interpolate.interp1d(s, points)
        s_to_C=interpolate.interp1d(s, C)
        s_to_dC=interpolate.interp1d(s, dC)
        self.s_to_C=s_to_C
        self.s_to_dC=s_to_dC
        self.s_to_psi=s_to_psi
        self.s_to_psi=s_to_psi
        self.s_to_XY=s_to_XY
    
    def local_info(self,s):
        C=self.s_to_C(s)
        dC=self.s_to_dC(s)
        psi=self.s_to_psi(s)
        XY=self.s_to_XY(s)
        local_property=pathInfo(s,C,dC,psi,XY)
        return local_property


if __name__=='__main__':
    import pyqtgraph as pg
    from tools import mat_reading
    pg.setConfigOptions(antialias=True)
    def f(t):
        x = np.cos(t)
        y = np.sin(t)
        return np.array([x,y])

    t = np.linspace(-10,10, 5000)
    points=f(t)
    X,Y=points
    p=Path(points)

    plot=pg.plot(pen={'color': '#186ff6', 'width': 2},background='w')

    F=p.local_info(p.s)
    plot.plot(F.s,F.C,pen={'color': 'blue', 'width': 2})

    path=mat_reading(f)
    plot.plot(path.s,path.C_c,pen={'color': 'green', 'width': 2})
    plot.showGrid(x=1,y=1)
    plot.show()
    pg.exec()