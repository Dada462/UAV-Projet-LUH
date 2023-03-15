import pyqtgraph as pg
import numpy as np
import sys
import threading
from numpy import pi
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QPainter, QBrush, QPen
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton,QLineEdit,QLabel
from std_msgs.msg import Float64

pg.setConfigOptions(antialias=True)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self,PF_controller=None, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graphWidget)
        self.resize(850, 850)
        self.setWindowTitle('Mission Displayer')
        
        # Buttons
        start_mission = QPushButton(self.graphWidget)
        stop_mission = QPushButton(self.graphWidget)
        reset_mission = QPushButton(self.graphWidget)

        start_mission.setText('Start Mission')
        stop_mission.setText('Stop Mission')
        reset_mission.setText('Reset Mission')

        start_mission.setGeometry(QtCore.QRect(0,0,100,25))
        stop_mission.setGeometry(QtCore.QRect(105,0,100,25))
        reset_mission.setGeometry(QtCore.QRect(205,0,100,25))

        start_mission.clicked.connect(self.start_recording_mission)
        stop_mission.clicked.connect(self.stop_recording_mission)
        self.mission_state={'start':False,'go_home':False}
        # Text box
        self.nameLabel = QLabel(self)
        self.nameLabel.setText('Ψa,Ky1')
        self.parameters_box = QLineEdit(self)

        self.parameters_box.move(600, 0)
        self.parameters_box.resize(125, 25)
        self.nameLabel.move(545,-7)

        pybutton = QPushButton('OK', self)
        pybutton.clicked.connect(self.clickMethod)
        pybutton.resize(100,25)
        pybutton.move(730,0)
        
        # Test data
        self.positions = np.zeros((10**4,3))
        self.pos_counter=0
        self.state=None
        self.pfc=PF_controller

        self.vehicle = pg.ArrowItem(angle=180, tipAngle=30, baseAngle=-30, headLen=40, tailLen=None, brush=(40, 138, 241, 180))
        self.vehicle.setPos(0, 0)

        # Creating the widgets
        self.graphWidget.setBackground('w')
        self.p = self.graphWidget.addPlot()

        # Setting the plot
        self.p.setLabel('left', 'y position (m)', **
                        {'color': 'r', 'font-size': '20px'})
        self.p.setLabel('bottom', 'x position (m)', **
                        {'color': 'r', 'font-size': '20px'})
        self.p.addLegend()
        self.p.showGrid(x=True, y=True)
        self.p.setTitle("UAV Project Mission Displayer", color="k", size="20px")

        self.p.addItem(self.vehicle)
        self.i = 0

        # Position trace
        self.trace = pg.PlotCurveItem(pen=({'color': '#f12828', 'width': 3, 'style': QtCore.Qt.DashLine}), skipFiniteCheck=True)
        self.p.addItem(self.trace)
        
        if __name__!='__main__':
            # Path
            self.path = pg.PlotCurveItem(pen=({'color': '#3486F4', 'width': 3}), skipFiniteCheck=True)
            self.point_to_follow = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush='#34F44C')
            self.path.setData(x=self.pfc.path_to_follow.X[:,0], y=self.pfc.path_to_follow.X[:,1])
            
            self.p.addItem(self.path)
            self.p.addItem(self.point_to_follow)


        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        
        if __name__!='__main__':
            xrange=[np.min(self.pfc.path_to_follow.X[:,0]),np.max(self.pfc.path_to_follow.X[:,0])]
            yrange=[np.min(self.pfc.path_to_follow.X[:,1]),np.max(self.pfc.path_to_follow.X[:,1])]
            range=np.min([xrange[0],yrange[0]]),np.max([xrange[1],yrange[1]])
            self.p.setXRange(*range)
            self.p.setYRange(*range)
        self.show()
    
    def clickMethod(self):
        vars=self.parameters_box.text().split(',')
        try:
            vars=[float(v) for v in vars]
            self.pfc.vars=vars
        except:
            pass
        print('Parameters: ', vars)

    
    def update_plot_data(self):
        if self.state is not None:
            x,y=self.state[:2]
            self.vehicle.setPos(x,y)
            self.vehicle.setStyle(angle=180*(1-self.state[2]/pi))
            self.trace.setData(x=self.positions[:self.pos_counter,0], y=self.positions[:self.pos_counter,1])
            self.i +=1

    def start_recording_mission(self):
        # Reinitialize the data
        print('Mission started')
        self.mission_state['start']=True
        # self.pfc.calculate_metrics(self.positions)
    
    def stop_recording_mission(self):
        # Stop recording and save the data
        self.mission_state['start']=False
        self.pfc.s=0
        self.positions=self.positions*0
        print('Mission is over')

    def update_state(self,state,s_pos):
        self.state=state
        # self.positions[:-1]=self.positions[1:]
        self.positions[self.pos_counter]=state
        # spots = [{'pos': s_pos}]
        # self.point_to_follow.addPoints(spots)
        self.point_to_follow.setData(x=s_pos[0],y=s_pos[1])
        self.pos_counter =(self.pos_counter+1)%(10**4)

    def sawtooth(self, x):
        return (x+pi) % (2*pi)-pi   # or equivalently   2*arctan(tan(x/2))


if __name__=='__main__':
    ################################## Pyqt ##################################
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.state=[0,0,pi/3]
    main.update_plot_data()
    sys.exit(app.exec_())
    ################################## Pyqt ##################################
