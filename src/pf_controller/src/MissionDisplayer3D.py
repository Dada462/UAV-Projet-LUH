import pyqtgraph as pg
import numpy as np
import sys
from datetime import datetime
import time
from numpy import pi
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QPainter, QBrush, QPen
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton,QLineEdit,QLabel
from std_msgs.msg import Float64
from PyQt5.QtCore import Qt
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation

pg.setConfigOptions(antialias=True)

class RobotMesh():
    def __init__(self,size=np.eye(3)):
        verts = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
            [0, 1, 1],
            [0, 0, 1],
            [1, 0, 1],
            [1, 1, 1]
        ])
        faces = np.array([
            [0, 1, 2],
            [2,3,0],
            [3,4,5],
            [5,0,3],
            [0,1,6],
            [5,6,0],
            [1,2,7],
            [7,6,1],
            [7,4,3],
            [3,2,7],
            [4,5,6],
            [6,7,4]
        ])
        vcenter=np.sum(verts,axis=0)/len(verts)
        verts=verts-vcenter
        verts=verts@size
        colors = np.array([
            [1, 0, 0, 1],
            [1, 0, 0, 1],
            [0, 0, 0, 1],
            [0, 0, 0, 1],
            [0, 1, 0, 1],
            [0, 1, 0, 1],
            [0.5, 0.5, 1, 1],
            [0.5, 0.5, 1, 1],
            [0, 1, 0, 1],
            [0, 1, 0, 1],
            [1, 0, 0, 1],
            [1, 0, 0, 1],
        ])
        self.base_vertices=verts
        self.vertices=verts
        self.center=vcenter
        self.position=np.zeros(3)
        self.colors=colors
        self.faces=faces
    
    def rotate(self,seq='XYZ',angles=[0,0,0],deg=False):
        r=Rotation.from_euler(seq=seq,angles=angles,degrees=deg).as_matrix()
        self.vertices=self.base_vertices@r.T


class GLViewWidget_Modified(gl.GLViewWidget):
    def __init__(self, parent=None, devicePixelRatio=None, rotationMethod='euler'):
        super().__init__(parent, devicePixelRatio, rotationMethod)
        self.pressed_keys = set()
        self.keyboard=[0,0,0,0,0,0]
    def evalKeyState(self):
        pass
    
    def keyPressEvent(self, event):
        self.pressed_keys.add(event.key())
        key_ids=[Qt.Key_Up,Qt.Key_Down,Qt.Key_Left,Qt.Key_Right,Qt.Key_G,Qt.Key_H]
        keys={key_ids[i]:i for i in range(len(key_ids))}
        for k in self.pressed_keys:
            if k in keys:
                self.keyboard[keys[k]]=1
            if k==Qt.Key_Return or k==Qt.Key_Enter:
                self.clickMethod()
                self.update_values()

    def keyReleaseEvent(self, event):
        self.pressed_keys.discard(event.key())
        key_ids=[Qt.Key_Up,Qt.Key_Down,Qt.Key_Left,Qt.Key_Right,Qt.Key_G,Qt.Key_H]
        keys={key_ids[i]:i for i in range(len(key_ids))}
        if event.key() in keys:
            self.keyboard[keys[event.key()]]=0

class plot2D(QtWidgets.QMainWindow):
    def __init__(self,PF_controller=None, *args, **kwargs):
        super(plot2D, self).__init__(*args, **kwargs)

        self.graphWidget = pg.GraphicsLayoutWidget()
        self.graphWidget.setBackground('w')
        self.setCentralWidget(self.graphWidget)
        self.resize(650, 550)
        self.setWindowTitle('Plot Window')
        
        # Buttons
        start_mission = QPushButton(self.graphWidget)

        start_mission.setText('Test button')

        start_mission.setGeometry(QtCore.QRect(0,0,100,25))

        # start_mission.clicked.connect(self.start_recording_mission)

        
        # Test data
        self.positions = np.zeros((10**4,6))
        self.positions_times=np.zeros(10**4)-1
        self.pos_counter=0
        self.state=None
        self.pfc=PF_controller

        # Creating the widgets
        self.p = self.graphWidget.addPlot(col=0,row=0)
        self.data_plot={0:pg.PlotCurveItem(pen=({'color': '#f12828', 'width': 3}), skipFiniteCheck=True,name='0')}
        self.N=3000
        self.values={0:np.zeros((self.N,2))}
        self.cursors={0:0}
        self.p.addItem(self.data_plot[0])

        # Setting the plot
        # self.p.setLabel('left', 'y', **{'color': 'r', 'font-size': '20px'})
        self.p.setLabel('bottom', 't (s)', **{'color': 'r', 'font-size': '20px'})
        self.p.addLegend()
        self.p.showGrid(x=True, y=True)
        self.p.setTitle("Plot", color="k", size="20px")

        self.i = 0
        
        self.timer = QtCore.QTimer()
        self.timer.setInterval(75)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        self.show()
    
    def plot(self,x,y,id=0):
        if not id in self.data_plot:
            if id==1:
                print('hi')
                self.data_plot[id]=pg.PlotCurveItem(pen=({'color': '#186ff6', 'width': 3}), skipFiniteCheck=True,name=str(id))
            elif id==2:
                self.data_plot[id]=pg.PlotCurveItem(pen=({'color': '#3df618', 'width': 3}), skipFiniteCheck=True,name=str(id))
                print('hi 2')
            else:
                self.data_plot[id]=pg.PlotCurveItem(pen=({'color': 'navy', 'width': 3}), skipFiniteCheck=True,name=str(id))
            self.p.addItem(self.data_plot[id])
            self.values[id]=np.zeros((self.N,2))
            self.cursors[id]=0
        if type(x)==np.float64 or type(x)==float or type(y)==np.float64 or type(y)==float:
            self.values[id][self.cursors[id]]=x,y
            values=self.values[id][:self.cursors[id]]
            self.data_plot[id].setData(x=values[:,0],y=values[:,1])
            self.cursors[id]+=1
        else:
            self.data_plot[id].setData(x=x,y=y)
        self.cursors[id]=self.cursors[id]%self.N
    
    def update_plot_data(self):
        # T=np.linspace(-10,10,4000)
        # for id in self.data_plot:
            # self.data_plot[id].setData(x=T,y=np.cos(T-self.i*0.075))
        self.i +=1

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self,PF_controller=None, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.w = GLViewWidget_Modified()
        self.setCentralWidget(self.w)
        self.resize(850, 750)
        self.move(500,150)
        self.setWindowTitle('Mission Displayer')
        
        self.w.setBackgroundColor('w')
        self.w.setCameraPosition(distance=20)
        axis=gl.GLAxisItem(glOptions='opaque')
        axis.setSize(15,15,15)
        gx = gl.GLGridItem(color=(0,0,0,40))
        gx.setSize(100,100)
        gx.setSpacing(1,1)
        self.w.addItem(gx)
        self.w.addItem(axis)
        
        # r=Rotation.from_euler('ZYX',(45,0,0),degrees=True).as_matrix()
        # verts=(verts-vcenter)@r.T+vcenter
        
        self.vehicle_mesh=RobotMesh(0.25*np.eye(3))
        self.vehicle = gl.GLMeshItem(vertexes=self.vehicle_mesh.vertices, faces=self.vehicle_mesh.faces, faceColors=self.vehicle_mesh.colors, smooth=False)
        self.w.addItem(self.vehicle)


        # Buttons
        start_mission = QPushButton(self)
        stop_mission = QPushButton(self)
        reset_mission = QPushButton(self)
        keyboard_mode = QPushButton(self)

        start_mission.setText('Start Mission')
        stop_mission.setText('Stop Mission')
        reset_mission.setText('Reset Data')
        keyboard_mode.setText('Keyboard')

        start_mission.setGeometry(QtCore.QRect(0,0,100,25))
        stop_mission.setGeometry(QtCore.QRect(105,0,100,25))
        reset_mission.setGeometry(QtCore.QRect(205,0,100,25))
        keyboard_mode.setGeometry(QtCore.QRect(305,0,100,25))

        start_mission.clicked.connect(self.start_recording_mission)
        stop_mission.clicked.connect(self.stop_recording_mission)
        reset_mission.clicked.connect(self.reset_mission_data)
        keyboard_mode.clicked.connect(self.keyboard_mode)
        
        self.robot_info_label_1 = QLabel(self)
        self.robot_info_label_1.setText('')
        self.robot_info_label_1.setFixedSize(100,100)
        self.robot_info_label_1.move(10,650)

        self.parameters_box = QLineEdit(self)
        self.parameters_box.move(600, 0)
        self.parameters_box.resize(150, 25)
        self.parameters_box.setText('0.4,1,2.5,1,1.5')

        self.mission_state={'start':False,'go_home':True,'keyboard':False}
        
        # Test data
        self.positions = np.zeros((10**4,12))
        self.positions_times=np.zeros(10**4)-1
        self.pos_counter=0
        self.state=None
        self.pfc=PF_controller
        self.i = 0

        # Position trace
        self.trace = gl.GLLinePlotItem(color='#f12828', width=2, antialias=True)
        self.trace.setGLOptions('opaque')
        self.w.addItem(self.trace)

        if __name__!='__main__':
            # Path
            self.path= gl.GLLinePlotItem( color='#3486F4', width=3, antialias=True)
            self.path.setGLOptions('opaque')
            self.point_to_follow = gl.GLScatterPlotItem(size=0.25,color=(52/255, 244/255, 76/255,1), pxMode=False)
            self.point_to_follow.setGLOptions('translucent')
            self.path.setData(pos=self.pfc.path_to_follow.points[:,:3])
            
            self.w.addItem(self.path)
            self.w.addItem(self.point_to_follow)
        else:
            # Path
            self.point_to_follow = gl.GLScatterPlotItem(pos=2*np.ones(3),size=0.1,color=(52/255, 244/255, 76/255,1), pxMode=False)
            self.point_to_follow.setGLOptions('translucent')
            self.w.addItem(self.point_to_follow)
        
        params=['Ke','k0','k1','Ks','Kth','ν','Kc','νc']
        default_values=list(np.load('params.npy'))
        if len(default_values)!=len(params):
            default_values=np.zeros(len(params))
        self.nb_of_params=len(params)
        self.params=params
        self.values=default_values
        self.create_params_boxes()
        self.control_output = gl.GLLinePlotItem(width=3, color=(0, 0, 1, 1),glOptions='opaque')
        self.w.addItem(self.control_output)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(75)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        self.pressed_keys = set()
        self.keyboard=[0,0,0,0,0,0]
        
        
        
        self.s1_arrow = gl.GLLinePlotItem(width=3, color=(0, 0, 1, 0.8),glOptions='opaque')
        self.y1_arrow = gl.GLLinePlotItem(width=3, color=(0, 1, 0, 0.8),glOptions='opaque')
        self.w1_arrow = gl.GLLinePlotItem(width=3, color=(1, 0.5, 0.5, 0.8),glOptions='opaque')
        
        self.w.addItem(self.s1_arrow)
        self.w.addItem(self.y1_arrow)
        self.w.addItem(self.w1_arrow)
        # if __name__!='__main__':
        #     xrange=[np.min(self.pfc.path_to_follow.points[:,0]),np.max(self.pfc.path_to_follow.points[:,0])]
        #     yrange=[np.min(self.pfc.path_to_follow.points[:,1]),np.max(self.pfc.path_to_follow.points[:,1])]
        #     range=np.min([xrange[0],yrange[0]]),np.max([xrange[1],yrange[1]])
        #     self.w.setCameraPosition()
        #     self.w.setXRange(*range)
        #     self.w.setYRange(*range)
        self.show()
    
    def create_params_boxes(self):
        self.buttons_add={}
        self.buttons_sub={}
        self.textBoxes={}
        self.labels={}
        b_off=[0,300]
        textSize=40
        
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
            self.labels[i].setText(self.params[i])
            self.labels[i].setGeometry(QtCore.QRect(0+b_off[0],100+25*i+b_off[1],25,25))

            self.textBoxes[i]=QLineEdit(self)
            self.textBoxes[i].setText(str(self.values[i]))
            self.textBoxes[i].setGeometry(QtCore.QRect(25+b_off[0],100+25*i+b_off[1],textSize,25))
    
    def click_function(self,add,id):
        def click_method():
            if add:
                self.values[id]=self.values[id]+0.1
            else:
                self.values[id]=self.values[id]-0.1
            self.textBoxes[id].setText(str(np.round(self.values[id],2)))
        return click_method

    def keyPressEvent(self, event):
        self.pressed_keys.add(event.key())
        key_ids=[Qt.Key_Up,Qt.Key_Down,Qt.Key_Left,Qt.Key_Right,Qt.Key_G,Qt.Key_H]
        keys={key_ids[i]:i for i in range(len(key_ids))}
        for k in self.pressed_keys:
            if k in keys:
                self.keyboard[keys[k]]=1
            if k==Qt.Key_Return or k==Qt.Key_Enter:
                self.clickMethod()
                self.update_values()

    def keyReleaseEvent(self, event):
        self.pressed_keys.discard(event.key())
        key_ids=[Qt.Key_Up,Qt.Key_Down,Qt.Key_Left,Qt.Key_Right,Qt.Key_G,Qt.Key_H]
        keys={key_ids[i]:i for i in range(len(key_ids))}
        if event.key() in keys:
            self.keyboard[keys[event.key()]]=0
    
    def keyboard_mode(self):
        self.mission_state['start']=False
        self.mission_state['keyboard']=True

    def clickMethod(self):
        vars=self.parameters_box.text().split(',')
        try:
            vars=[float(v) for v in vars]
            self.pfc.vars=vars
        except:
            pass
        # print('Parameters: ', vars)
    
    def update_values(self):
        for i in range(self.nb_of_params):
            v=float(self.textBoxes[i].text())
            try:
                self.values[i]=v
            except:
                pass
        np.save('params.npy',self.values)
        print('Parameters enter: ', self.values)
    
    def update_plot_data(self):
        if self.state is not None:
            try:
                # speed=np.linalg.norm(self.state[3:6])
                speed=self.pfc.v1
                self.robot_info_label_1.setText('x={x:0.2f} m\ny={y:0.2f} m\nz={z:0.2f} m\ne={error:0.2f} cm\nv={speed:0.2f} m/s'.format(x=self.state[0],y=self.state[1],z=self.state[2],error=self.pfc.error,speed=speed))
            except:
                pass
                self.robot_info_label_1.setText('x={x:0.2f}\ny={y:0.2f}\nz={z:0.2f}\n'.format(x=self.state[0],y=self.state[1],z=self.state[2]))

            self.vehicle.setMeshData(vertexes=self.vehicle_mesh.vertices+self.state[:3], faces=self.vehicle_mesh.faces, faceColors=self.vehicle_mesh.colors)
            self.trace.setData(pos=self.positions[:self.pos_counter,:3])
            self.i +=1
            self.keyboard=self.w.keyboard
            # t=self.i/20
            # pos=np.array([0,0,5])
            # dir=np.array([3*np.cos(t),3*np.sin(t),0])
            # arrow=np.vstack((pos,pos+dir))
            # self.control_output.setData(pos=arrow)

    def start_recording_mission(self):
        # Reinitialize the data
        print('Mission started')
        self.mission_state['start']=True
        self.mission_state['keyboard']=False
        # self.pfc.calculate_metrics(self.positions)
    
    def stop_recording_mission(self):
        # Stop recording and save the data
        print('Mission is over')
        
        # self.mission_results()
        now = datetime.now()
        dt_string = now.strftime("%d.%m.%Y_%H.%M.%S")
        # print(self.positions_times.shape,self.positions.shape)
        # positions=np.hstack((self.positions,self.positions_times.reshape((-1,1))))
        
        # np.save('results/positions_'+dt_string+'.npy',positions)
        # np.save('results/path_to_follow_'+dt_string+'.npy',self.pfc.path_to_follow.X)

        self.mission_state['start']=False
        self.mission_state['keyboard']=False
        # self.pfc.s=0
        # self.positions=self.positions*0

    def reset_mission_data(self):
        self.pfc.s=0
        self.positions=self.positions*0
        # self.pfc.PID.I=0
        self.data_to_plotx=[]
        self.data_to_ploty1=[]
        self.data_to_ploty2=[]
        self.data_to_ploty3=[]

    def update_state(self,state,s_pos):
        self.state=state
        self.positions[self.pos_counter]=state
        self.positions_times[self.pos_counter]=time.time()
        self.point_to_follow.setData(pos=s_pos)
        self.vehicle_mesh.position=state[:3]
        self.vehicle_mesh.rotate('XYZ',state[6:9])
        self.pos_counter =(self.pos_counter+1)%(10**4)

    def sawtooth(self, x):
        return (x+pi) % (2*pi)-pi   # or equivalently   2*arctan(tan(x/2))

    def mission_results(self):
        print('calculating results')

if __name__=='__main__':
    ################################## Pyqt ##################################
    app = QtWidgets.QApplication(sys.argv)
    main=plot2D()
    T=np.linspace(-10,10,500)
    main.plot(T,0.1*T**2,0)
    # main.plot(T,np.sin(T),1)
    T=np.linspace(0,10,500)
    for t in T:
        main.plot(t,np.sin(t),2)

    # main = MainWindow()
    # main.state=np.zeros(12)
    # main.update_plot_data()
    sys.exit(app.exec_())
    ################################## Pyqt ##################################
