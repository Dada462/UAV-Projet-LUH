from tools_3D import R,Path_3D
import numpy as np
from numpy import pi,cos,sin
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
import pyqtgraph.opengl as gl
from scipy.linalg import expm,logm
from scipy.spatial.transform import Rotation
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton,QLineEdit,QLabel
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import Qt
pg.setConfigOptions(antialias=True)

# import pyqtgraph.examples
# pyqtgraph.examples.run()


# def state_function(state,u):
#     Vr=state[3:6]
#     dX=Vr
#     dVr=u[:3]
#     ds=u[3]
#     return np.hstack((dX,dVr,np.zeros(6),ds))

def state_function(state,u):
    dX=u[:3]
    ds=u[3]
    return np.hstack((dX,np.zeros(9),ds))

def controller(state):
    X = state[:3]
    s=state[-1]

    F=p.local_info(s)
    Rpath=np.vstack((F.s1,F.y1,F.w1)).T
    s1, y1, w1 = Rpath.T@(X-F.X)
    
    
    psi_a=pi/2
    e=np.array([s1,y1,w1])
    cross_e=np.cross(np.array([1,0,0]),e)
    delta=-psi_a*np.tanh(cross_e)
    Rpsi=expm(adj(delta))
    Rs=Rpath@Rpsi
    nu=2
    Vr=Rs@np.array([nu,0,0])

    ks=3
    ds = (np.array([nu,0,0])@Rpsi)[0]+ks*s1

    dX=np.array([*Vr,ds])
    
    return dX


def LPF_control_3D(state,F,params):
        X = state[0:3]
        Vr = state[3:6]
        s = state[12]
        phi,theta,psi=state[6:9]
        wr=state[9:12]

        # Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False)
        # dRm=Rm@self.adj(wr)
        Rm=np.eye(3)
        dRm=np.zeros((3,3))
        
        u, v, w = Vr
        nu = np.linalg.norm(Vr)
        beta=np.arctan2(v,u)
        gamma=np.arcsin(w/(nu+1e-6)) # Check
        Rs=R(beta,'z')@R(gamma,'y')

        
        # F=p.local_info(s)
        Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        dRpath=F.dR
        
        Rtheta = Rpath.T@Rm
        Rpsi=Rtheta@Rs
        
        dRtheta=dRpath.T@Rm+Rpath.T@dRm
        
        s1, y1, w1 = Rpath.T@(X-F.X)
        # print(100*np.linalg.norm([s1,y1,w1],ord=np.inf))

        #############################################################
        #############################################################

        ks = 3
        ds = (np.array([nu,0,0])@Rpsi)[0]+ks*s1
        # ds=0

        ds1, dy1,dw1 = Rtheta@Vr-ds*np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])
        # print('variable',np.round(ds1,2),np.round(dy1,2),np.round(dw1,2))
        # print('variable',ds)

        # psi_a=1

        psi_a,Ke,k1,Kpsi,nu_d=params

        e=np.array([s1,y1,w1])
        de=np.array([ds1,dy1,dw1])

        cross_e=np.cross(np.array([1,0,0]),e)
        dcross_e=np.cross(np.array([1,0,0]),de)

        # Ke=1
        # delta=-psi_a*np.tanh(Ke*cross_e*nu)
        delta=-psi_a*np.tanh(Ke*cross_e)
        # nu_d=2
        # k1=1
        dnu=k1*(nu_d-nu)
        # ddelta=-psi_a*Ke*(1-np.tanh(Ke*cross_e*nu)**2)*(dcross_e*nu+dnu*cross_e)
        ddelta=-psi_a*Ke*(1-np.tanh(Ke*cross_e)**2)*dcross_e
        
        Rdelta=expm(adj(delta))

        # Kpsi=3
        dRpsi=-Kpsi*logm(Rpsi@Rdelta.T)@Rpsi-Rpsi@adj(Rdelta.T@ddelta)

        dRs=Rtheta.T@(dRpsi-dRtheta@Rs)
        # dVr=dRs@np.array([nu,0,0])+Rs@np.array([dnu,0,0])
        dVr=nu*dRs[:,0]+dnu*Rs[:,0]
        # print('command',dVr)
        return np.hstack((dVr,ds))


def LPF_control_3D_kin_v2(state,F,params):
        X = state[0:3]
        Vr = state[3:6]
        s = state[12]
        # phi,theta,psi=state[6:9]
        # wr=state[9:12]

        Rm=np.eye(3)
        # dRm=np.zeros((3,3))
        
        # u, v, w = Vr
        # nu = np.linalg.norm(Vr)
        # beta=np.arctan2(v,u)
        # gamma=np.arcsin(w/(nu+1e-6)) # Check
        # Rs=R(beta,'z')@R(gamma,'y')

        
        # F=p.local_info(s)
        Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        # dRpath=F.dR
        
        # Rtheta = Rpath.T@Rm
        # Rpsi=Rtheta@Rs
        
        # dRtheta=dRpath.T@Rm+Rpath.T@dRm
        
        s1, y1, w1 = Rpath.T@(X-F.X)

        ks = 3

        # ds1, dy1,dw1 = Rtheta@Vr-ds*np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])

        psi_a,Ke,k1,Kpsi,nu_d=params

        e=np.array([s1,y1,w1])
        # de=np.array([ds1,dy1,dw1])

        Vp=np.array([nu_d*(1-np.tanh(np.abs(y1)+np.abs(w1))),-np.tanh(Ke*y1),-np.tanh(Ke*w1)])
        Vr=Rpath@Vp
        ds = (Vr@Rpath)[0]+ks*s1
        return np.hstack((Vr,ds)),0

def LPF_control_3D_v2(state,F,params):
        X = state[0:3]
        Vr = state[3:6]
        s = state[12]
        phi,theta,psi=state[6:9]
        wr=state[9:12]

        Rm=np.eye(3)
        dRm=np.zeros((3,3))
        
        F=p.local_info(s)
        Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        dRpath=F.dR
        
        Rtheta = Rpath.T@Rm
        dRtheta=dRpath.T@Rm+Rpath.T@dRm
        s1, y1, w1 = Rpath.T@(X-F.X)
        ks = 3
        ds = (Vr@Rpath)[0]+ks*s1
        


        ds1, dy1,dw1 = Rtheta@Vr-ds*np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])

        psi_a,Ke,k1,Kpsi,nu_d=params

        e=np.array([s1,y1,w1])
        de=np.array([ds1,dy1,dw1])
        v_e=3
        Vp=np.array([nu_d*(1-np.tanh(np.linalg.norm(e))),-v_e*np.tanh(Ke*y1),-v_e*np.tanh(Ke*w1)])
        dVp=-(1-np.tanh(np.array([np.linalg.norm(e),Ke*y1,Ke*w1]))**2)*np.array([de.T@de,dy1,dw1])
        D=np.diag((nu_d,v_e*Ke,v_e*Ke))
        dVp=D@dVp
        Vd=Rtheta.T@Vp
        dVd=dRtheta.T@Vp+Rtheta.T@dVp
        dVr=dVd+k1*(Vd-Vr)
        dVr=dVr+adj(wr)@Vr
        err=e
        return np.hstack((dVr,ds)),err

def adj(w):
    return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def adj_inv(A):
    return np.array([A[2,1],A[0,2],A[1,0]])



class MainWindow(QtWidgets.QMainWindow):
    def __init__(self,pts,pos,X,params,default_values,p, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.resize(850, 750)
        self.move(500,150)
        self.setWindowTitle('Mission Displayer')
        self.w = gl.GLViewWidget()
        self.setCentralWidget(self.w)
        self.w.setWindowTitle('Lab')
        # self.w.setBackgroundColor('w')
        self.w.setCameraPosition(distance=40)
        
        axis=gl.GLAxisItem(glOptions='opaque')
        axis.setSize(15,15,15)
        gx = gl.GLGridItem(color=(255,255,255,50))
        gx.setSize(100,100)
        gx.setSpacing(1,1)
        gx.translate(0,0,-10)
        gy = gl.GLGridItem(color=(255,255,255,50))
        gy.rotate(90,1,0,0)
        gy.setSize(100,100)
        gy.setSpacing(1,1)
        gy.translate(0,-50,40)
        gz = gl.GLGridItem(color=(255,255,255,50))
        gz.rotate(90,0,1,0)
        gz.setSize(100,100)
        gz.setSpacing(1,1)
        gz.translate(-50,0,40)
        self.w.addItem(gx)
        self.w.addItem(gy)
        self.w.addItem(gz)
        self.w.addItem(axis)
        
        self.plt = gl.GLLinePlotItem(pos=pts, color='red', width=3, antialias=True)
        self.robot_path=gl.GLLinePlotItem(color='navy', width=3, antialias=True)
        self.sp1 = gl.GLScatterPlotItem(pos=pos,size=0.5,color=(0.2,0.96,0.25,1), pxMode=False)
        self.robot = gl.GLScatterPlotItem(pos=X[:3],size=0.5,color=(0.5,0.96,0.25,1), pxMode=False)

        self.w.addItem(self.plt)
        self.w.addItem(self.sp1)
        self.w.addItem(self.robot)
        self.w.addItem(self.robot_path)


        self.s1_arrow = gl.GLLinePlotItem(width=3, color=(0, 0, 1, 0.8))
        self.y1_arrow = gl.GLLinePlotItem(width=3, color=(0, 1, 0, 0.8))
        self.w1_arrow = gl.GLLinePlotItem(width=3, color=(1, 0.5, 0.5, 0.8))


        self.w.addItem(self.s1_arrow)
        self.w.addItem(self.y1_arrow)
        self.w.addItem(self.w1_arrow)

        self.buttons_add={}
        self.buttons_sub={}
        self.textBoxes={}
        self.labels={}
        b_off=[0,500]
        textSize=40
        self.nb_of_params=len(params)
        self.params=params
        self.values=default_values
        for i in range(self.nb_of_params):
            self.buttons_add[i]=QPushButton(self)
            self.buttons_add[i].setText('+')
            self.buttons_add[i].setGeometry(QtCore.QRect(25+textSize+b_off[0],100+25*i+b_off[1],25,25))
            self.buttons_add[i].clicked.connect(self.click_function(True,i))
            self.buttons_sub[i]=QPushButton(self)
            self.buttons_sub[i].setText('-')
            self.buttons_sub[i].setGeometry(QtCore.QRect(50+textSize+b_off[0],100+25*i+b_off[1],25,25))
            self.buttons_sub[i].clicked.connect(self.click_function(False,i))

            self.labels[i]= QLabel(self)
            self.labels[i].setStyleSheet("background-color: lightgreen; border: 1px solid black;")
            self.labels[i].setText(params[i])
            self.labels[i].setGeometry(QtCore.QRect(0+b_off[0],100+25*i+b_off[1],25,25))

            self.textBoxes[i]=QLineEdit(self)
            self.textBoxes[i].setText(str(self.values[i]))
            self.textBoxes[i].setGeometry(QtCore.QRect(25+b_off[0],100+25*i+b_off[1],textSize,25))
        
        self.robot_info= QLabel(self)
        self.robot_info.setStyleSheet("background-color: lightgreen; border: 1px solid black;")
        self.robot_info.setGeometry(QtCore.QRect(0,0,75,25))
        self.robot_info.setStyleSheet("background-color: lightgreen; border: 1px solid black;")

        self.sim_button=QPushButton(self)
        self.sim_button.setText('simulate')
        self.sim_button.setGeometry(QtCore.QRect(b_off[0],75+b_off[1],75,25))
        self.p=p
        self.sim_button.clicked.connect(self.simulate)
        self.pressed_keys = set()
        self.show()
        self.timer = QtCore.QTimer()
        self.dt=0.01
        self.tmax=15
        self.timer.setInterval(int(1000*self.dt))
        self.timer.timeout.connect(self.update_plot_data)
        self.simulate()
        self.timer.start()
    
    def keyPressEvent(self, event):
        self.pressed_keys.add(event.key())
        for k in self.pressed_keys:
            if k==Qt.Key_Return or k==Qt.Key_Enter:
                self.update_values()

    def update_values(self):
        for i in range(self.nb_of_params):
            v=float(self.textBoxes[i].text())
            try:
                self.values[i]=v
            except:
                pass
        np.save('params_lab.npy',self.values)
        print('Parameters: ', self.values)
    
        
    def keyReleaseEvent(self, event):
        self.pressed_keys.discard(event.key())
    
    def click_function(self,add,id):
        def click_method():
            if add:
                self.values[id]=self.values[id]+0.1
            else:
                self.values[id]=self.values[id]-0.1
            self.textBoxes[id].setText(str(np.round(self.values[id],2)))
            # print(id,self.values[id])
        return click_method
    
    def simulate(self):
        global X
        X=X*0
        beta=pi/2
        gamma=pi/3
        V=R(beta,'z')@R(gamma,'y')@np.array([10,0,0])
        X[3:6]=V
        dt=self.dt
        self.r_pos=[[0,0,0]]
        # for t in np.arange(0,self.tmax,dt):
        s1,y1,w1=self.update_plot_data()
        self.sp1.setData(pos=pos)
        self.robot_path.setData(pos=self.r_pos)
        self.s1_arrow.setData(pos=s1)
        self.y1_arrow.setData(pos=y1)
        self.w1_arrow.setData(pos=w1)
        self.robot.setData(pos=X[:3])

    def update_plot_data(self):
        global X
        F=self.p.local_info(X[-1])
        pos=F.X.T
        s1=np.vstack((pos,pos+2*F.s1))
        y1=np.vstack((pos,pos+2*F.y1))
        w1=np.vstack((pos,pos+2*F.w1))
        
        # u=controller(X)
        # u[:3]=5*(u[:3]-X[3:6])
        
        # u=LPF_control_3D(X,F,self.values)

        u,err=LPF_control_3D_kin_v2(X,F,self.values)
        X[-1]=max(0,X[-1])
        err=np.linalg.norm(err)
        self.robot_info.setText('e={error:0.2f}'.format(error=err))
        self.sp1.setData(pos=pos)
        self.robot_path.setData(pos=self.r_pos)
        self.s1_arrow.setData(pos=s1)
        self.y1_arrow.setData(pos=y1)
        self.w1_arrow.setData(pos=w1)
        self.robot.setData(pos=X[:3])
        dt=self.dt
        X=X+dt*state_function(X,u)
        self.r_pos.append(list(X[:3]))

        return s1,y1,w1
    


# def f(t):
#     x = 10*np.cos(10*t)
#     y = 10*np.sin(10*t)-20
#     z=0*t
#     return np.array([x,y,z])

# p=Path_3D(f,[-10,10],type='parametric')
# f=lambda t : R(0.1*t,'x')@np.array([15*cos(t),15*sin(t),0*t])+np.array([0,0,45])*0
# points=[]
# for t in np.linspace(-10,20,4000):
#     points.append(f(t))
# points=np.array(points).T
# p=Path_3D(points,type='waypoints')
p=Path_3D(lambda t : np.array([0.1*t**2+1,5*sin(0.25*t**2),0*t]),[0,15],type='parametric')

F=p.local_info(p.s)
pts=F.X.T
pos=np.zeros(3)



X=np.zeros(13)
X[-1]=0



t=0

app = pg.mkQApp()
try:
    values=list(np.load('params_lab.npy'))
except:
    values=np.array([1,1,1,1,1])
window=MainWindow(pts,pos,X,['ψ','Ke','k1','Kψ','ν'],values,p)


from time import time
t0=time()




pg.exec()
# sys.exit(app.exec_())