import numpy as np
from numpy.linalg import norm
from numpy import pi,cos,sin
from scipy import interpolate
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

def sawtooth(x):
    return (x+pi) % (2*pi)-pi

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

class pathInfo():
    def __init__(self):
        pass

class Path_3D():
    def __init__(self,*args,**kwargs):
        if 'type' not in kwargs:
            raise ValueError('You must specity a type, either type=\'parametric\' or \'waypoints\'')
        if kwargs['type']=='parametric':
            f=args[0]
            if 'range' in kwargs:
                values_range=kwargs['range']
            else:
                values_range=[-10,10]
            t = np.linspace(*values_range, 6000)
            points=f(t)
            self.points=points.T
        elif kwargs['type']=='waypoints':
            points=args[0]
            self.points=points.T
        from time import time
        t0=time()
        # self.compute_path_properties()
        self.compute_path_properties_PTF()
        print(time()-t0)
    
    def __call__(self,s):
        return self.local_info(s)

    def curvature(self,dr,d2r):
        return norm(np.cross(dr,d2r),axis=1)/(norm(dr,axis=1)**3)
    
    def sawtooth(self,x):
        return (x+pi) % (2*pi)-pi
    
    def adj(self,w):
        return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

    def adj_inv(self,A):
        return np.array([A[2,1],A[0,2],A[1,0]])
    
    def compute_path_properties_PTF(self):
        points=self.points.T
        df = np.gradient(points,axis=1)
        ds=norm(df,axis=0)
        s=np.cumsum(ds)
        s=s-s[0]

        d2f_ds=np.gradient(df/ds,axis=1)
        C=norm(d2f_ds/ds,axis=0)
        dC=np.gradient(C)/ds
        
        T=df/ds # Columns
        # V0=np.cross(np.array([0,0,1]),T[:,0])
        V0=T[:,0]
        for x in [np.array([0,0,1]),np.array([0,1,0]),np.array([1,0,0])]:
            if np.linalg.norm(np.cross(x,V0))>0.01:
                V0=np.cross(x,V0)
                V0=V0/np.linalg.norm(V0)
                break
        N=np.zeros_like(T)
        N[:,0]=V0

        # Bc=np.cross(T[:,:-1],T[:,1:],axis=0)
        # z=np.linalg.norm(Bc,axis=0)<1e-9
        # z=np.where(z==1)[0]
        # Bc=Bc/(1e-6+np.linalg.norm(Bc,axis=0))
        # print(Bc)
        # theta=np.sum(T[:,:-1]*T[:,1:],axis=0)
        # theta=np.clip(theta,-1,1)
        # theta=np.arccos(theta)
        # r=Rotation.from_rotvec((theta*Bc).T)
        # N[:,:-1]=Bc
        # N[:,1:]=r.apply(N[:,:-1].T).T
        # # a=r.apply(N[:,:-1].T).T
        # # print(a.shape)
        # for i in z:
        #     N[i+1:]=N[i:len(N)-1]

        for i in range(len(T.T)-1):
            B=np.cross(T[:,i],T[:,i+1])
            if np.linalg.norm(B)<1e-7:
                N[:,i+1]=N[:,i]
            else:
                B=B/np.linalg.norm(B)
                theta=np.arccos(T[:,i]@T[:,i+1])
                r=Rotation.from_rotvec(theta*B).as_matrix()
                N[:,i+1]=r@N[:,i]
            N[:,i+1]=N[:,i+1]/np.linalg.norm(N[:,i+1])
        
        B=np.cross(T,N,axis=0)
        
        # Derivatives of the frame with respesct to the curvilinear abscissa
        dT=np.gradient(T,axis=1)/ds
        dN=np.gradient(N,axis=1)/ds
        dB=np.gradient(B,axis=1)/ds
        
        # Rotation matrix'
        R=np.stack((T.T,N.T,B.T),axis=1)
        dR=np.zeros_like(R)
        dR=np.stack((dT.T,dN.T,dB.T),axis=1)
        
        R=np.transpose(R,axes=(0,2,1))
        dR=np.transpose(dR,axes=(0,2,1))
        
        # Coefficients k1 and k2 from the Parallel Transport Frame Equation & curvature and torsion from SFF

        y1=np.gradient(T,axis=1)
        y1=y1/(norm(y1,axis=0)+1e-6)
        w1=np.cross(T,y1,axis=0)
        dw1=np.gradient(w1,axis=1)/ds
        
        Tr=-np.sum(dw1*y1,axis=0)
        dTr=np.gradient(Tr)/ds
        
        theta=np.cumsum(Tr*ds)
        k1=C*np.cos(theta)
        k2=C*np.sin(theta)

        self.s_to_XYZ=interpolate.interp1d(s, points)

        self.s_to_T=interpolate.interp1d(s, T)
        self.s_to_N=interpolate.interp1d(s, N)
        self.s_to_B=interpolate.interp1d(s, B)

        self.s_to_dT=interpolate.interp1d(s, dT)
        self.s_to_dN=interpolate.interp1d(s, dN)
        self.s_to_dB=interpolate.interp1d(s, dB)

        self.s_to_C=interpolate.interp1d(s, C)
        self.s_to_dC=interpolate.interp1d(s, dC)
        self.s_to_Tr=interpolate.interp1d(s, Tr)
        self.s_to_dTr=interpolate.interp1d(s, dTr)
        self.s_to_k1=interpolate.interp1d(s, k1)
        self.s_to_k2=interpolate.interp1d(s, k2)

        self.s_to_R=interpolate.interp1d(s, R,axis=0)
        self.s_to_dR=interpolate.interp1d(s, dR,axis=0)

        self.ds=ds
        self.s=s
        self.s_max=np.max(s) # Length of the path
        self.s_to_w1=interpolate.interp1d(s, w1)


    def compute_path_properties(self):
        points=self.points.T
        df = np.gradient(points,axis=1)
        ds=norm(df,axis=0)
        self.ds=ds
        s=np.cumsum(ds)
        s=s-s[0]
        self.s_max=np.max(s)

        d2f_ds=np.gradient(df/ds,axis=1)
        C=norm(d2f_ds/ds,axis=0)
        dC=np.gradient(C)/ds
        
        s1=df/ds
        y1=np.gradient(s1,axis=1)
        y1=y1/(norm(y1,axis=0)+1e-6)
        w1=np.cross(s1,y1,axis=0)

        
        ds1=np.gradient(s1,axis=1)/ds
        dy1=np.gradient(y1,axis=1)/ds
        dw1=np.gradient(w1,axis=1)/ds
        
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

        Tr=-np.sum(dw1*y1,axis=0)
        dTr=np.gradient(Tr)/ds

        self.s=s
        s_to_XYZ=interpolate.interp1d(s, points)
        s_to_C=interpolate.interp1d(s, C)
        s_to_dC=interpolate.interp1d(s, dC)
        s_to_s1=interpolate.interp1d(s, s1)
        s_to_y1=interpolate.interp1d(s, y1)
        s_to_w1=interpolate.interp1d(s, w1)
        s_to_dsyw=interpolate.interp1d(s, dR,axis=0)
        s_to_Tr=interpolate.interp1d(s, Tr)
        s_to_dTr=interpolate.interp1d(s, dTr)

       

        self.s_to_C=s_to_C
        self.s_to_dC=s_to_dC
        self.s_to_XY=s_to_XYZ
        self.s_to_s1=s_to_s1
        self.s_to_y1=s_to_y1
        self.s_to_w1=s_to_w1
        self.s_to_dsyw=s_to_dsyw
        self.s_to_Tr=s_to_Tr
        self.s_to_dTr=s_to_dTr
        
        
        s_to_ds1=interpolate.interp1d(s, ds1)
        s_to_dy1=interpolate.interp1d(s, dy1)
        s_to_dw1=interpolate.interp1d(s, dw1)
        self.s_to_ds1=s_to_ds1
        self.s_to_dy1=s_to_dy1
        self.s_to_dw1=s_to_dw1

        def gauss(x):
            return np.exp(-x**2/2)/np.sqrt(2*pi)

        T=np.linspace(-1,1,100)*2
        k=gauss(T)
        k=k/np.sum(k)*2
        y=np.convolve(dC,k,mode='same')
        yd=np.abs(C[0]+np.cumsum(y*ds))
        self.max_speed=interpolate.interp1d(s, yd)
        # yd=np.sqrt(3/(1e-3+yd))
        # yd=np.clip(yd,0,1.5)

    
    def local_info(self,s):
        # C=self.s_to_C(s)
        # dC=self.s_to_dC(s)
        # psi=None
        # XYZ=self.s_to_XY(s)
        # s1=self.s_to_s1(s)
        # y1=self.s_to_y1(s)
        # w1=self.s_to_w1(s)
        # s_to_dsyw=self.s_to_dsyw(s)
        # s_to_Tr=self.s_to_Tr(s)
        # s_to_dTr=self.s_to_dTr(s)
        local_property=pathInfo()
        
        s=np.clip(s,0,self.s_max)
        local_property.s=s
        local_property.X=self.s_to_XYZ(s)
        
        local_property.C=self.s_to_C(s)
        local_property.dC=self.s_to_dC(s)
        local_property.Tr=self.s_to_Tr(s)
        local_property.dTr=self.s_to_dTr(s)
        local_property.k1=self.s_to_k1(s)
        local_property.k2=self.s_to_k2(s)

        local_property.T=self.s_to_T(s)
        local_property.N=self.s_to_N(s)
        local_property.B=self.s_to_B(s)

        local_property.dT=self.s_to_dT(s)
        local_property.dN=self.s_to_dN(s)
        local_property.dB=self.s_to_dB(s)

        local_property.R=self.s_to_R(s)
        local_property.dR=self.s_to_dR(s)

        local_property.w1=self.s_to_w1(s)
        
        return local_property


if __name__=='__main__':
<<<<<<< HEAD
    # import pyqtgraph as pg
    # pg.setConfigOptions(antialias=True)
    # plot=pg.plot(pen={'color': '#0e70ec', 'width': 2},background='w')
    # plot.resize(1200, 850)
    # plot.move(300, 115)
=======
    import pyqtgraph as pg
    pg.setConfigOptions(antialias=True)
    plot=pg.plot(pen={'color': '#0e70ec', 'width': 2},background='w')
    plot.resize(1200, 850)
    plot.move(300, 115)
>>>>>>> 882754d693cd6e37f43becade41c566e9298bbea
    
    # f=lambda t : R(0.15,'x')@np.array([1*(1+0.25*np.sin(4*t))*np.cos(t),1*(1+0.25*np.sin(4*t))*np.sin(t),0*t+0.5])
    # p=Path_3D(lambda t : np.array([t**2,-10+t,10+0*t]),[-20,20],type='parametric')
    # f=lambda t : np.array([cos(6*t),sin(6*t),t**2])
    # f=lambda t : np.array([cos(t),sin(t),t*cos(t)])
    # f=lambda t : np.array([3*(1.5+np.sin(6*t))*np.cos(t),3*(1.5+np.sin(6*t))*np.sin(t),0*t+1])
    
    # 1: Line
    line=lambda t : np.array([t,t,0*t])+np.array([-1,1,1.5])
    line_range=(-2,4)
    # 2: U-Turn
    n=6
    r=4
    uturn=lambda t : np.array([1*np.cos(t),np.sign(np.sin(t))*(r**n-(r*np.cos(t))**n)**(1/n),0*t+1.5])
    uturn_range=(0,pi)
    
<<<<<<< HEAD
    # 3: Obstacle avoidance 1:
=======
    # 3: Obstacle avoidance 1
>>>>>>> 882754d693cd6e37f43becade41c566e9298bbea
    def obs_av_1(t):
        b=np.array([0,16,0])
        X=uturn(t)*(t<pi/2)+(2*uturn(pi/2)-uturn(pi-t))*(1.5*pi>t>=pi/2)+(b+uturn(t))*(1.5*pi<=t)
        return X
    obs_av_1_range=(0,2*pi)

<<<<<<< HEAD
    # 4: Obstacle avoidance 2:
=======
    # 4: Obstacle avoidance 2
>>>>>>> 882754d693cd6e37f43becade41c566e9298bbea
    T=1
    A=0.8
    obs_av_2=lambda t : np.array([t,A*np.sin(2*pi*t/T),0*t+1.5])
    obs_av_2_range=(0,3)
<<<<<<< HEAD
    
=======
    # 5: Circular
    circular=lambda t : np.array([5*np.cos(t)*np.sin(0.5*t),3*np.sin(t),1.5])
    circular_range=(0,2*pi)

    # Other paths
>>>>>>> 882754d693cd6e37f43becade41c566e9298bbea
    # p=Path_3D(f,range=[0,6],type='parametric')
    # p=Path_3D(lambda t : (2+sin(10*t))*np.array([cos(t),sin(t),0*t+1]),range=[-10,-9],type='parametric')
    # f=lambda t : np.array([t,t,0*t])
    # f=lambda t : np.array([np.cos(t),np.sin(t),0*t+1.5])
<<<<<<< HEAD
    rng=obs_av_2_range
    f=obs_av_2
=======
    
    rng=circular_range
    f=circular
>>>>>>> 882754d693cd6e37f43becade41c566e9298bbea
    points=[]
    for t in np.linspace(*rng,6000):
        points.append(f(t))
    points=np.array(points).T
    p=Path_3D(points,type='waypoints')
<<<<<<< HEAD
    print('Lenght:',np.round(p.s_max,2),'m')
    # p=Path_3D(f,range=[0,2*pi],type='parametric')
    F=p.local_info(p.s)
    # V=p.compute_path_properties_PTF()
    import matplotlib.pyplot as plt

    # plt.plot(F.X[0],F.X[1])
    # plt.xlim(-10,10)
    # plt.ylim(-10,10)
    plt.plot(F.s,F.C)
    # plt.plot(F.s,p.s_to_s1(p.s)[0])
    # plt.plot(F.s,p.s_to_dT(p.s).T,c='green')
=======
    # p=Path_3D(f,range=[0,2*pi],type='parametric')
    F=p.local_info(p.s)


    print('Lenght:',np.round(p.s_max,2),'m')
    plot.plot(F.X[0],F.X[1],pen={'color': '#0e70ec', 'width': 2})

   
>>>>>>> 882754d693cd6e37f43becade41c566e9298bbea
    s=p.s
    k1=p.s_to_k1(p.s)
    k2=p.s_to_k2(p.s)
    C=p.s_to_C(p.s)
    Tr=p.s_to_Tr(p.s)
    T=p.s_to_T(s)
    N=p.s_to_N(s)
    B=p.s_to_B(s)
    
    dT=k1*N+k2*B
    dN=-k1*T
    dB=-k2*T

    Rpath=p.s_to_R(s)[:,:,0]
    dRpath=p.s_to_dR(s)[:,:,2]
<<<<<<< HEAD
    # plt.quiver(F.X[0],F.X[1],F.T[0],F.T[1],scale=10)
    # plt.plot(F.s,dRpath,c='blue')
    plt.show()


    # plot.plot(F.X[0],F.X[1],pen={'color': '#0e70ec', 'width': 4})
    # plot.showGrid(x=True,y=True)
    # plot.show()
    # pg.exec()
=======

    plot.showGrid(x=True,y=True)
    plot.show()
    pg.exec()
>>>>>>> 882754d693cd6e37f43becade41c566e9298bbea
