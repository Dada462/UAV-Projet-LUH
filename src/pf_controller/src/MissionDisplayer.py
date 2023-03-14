import pyqtgraph as pg
import numpy as np
import sys
import threading
from numpy import pi
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QPainter, QBrush, QPen
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton
from std_msgs.msg import Float64

pg.setConfigOptions(antialias=True)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graphWidget)
        self.resize(850, 600)
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
        stop_mission.clicked.connect(self.start_recording_mission)
        # Test data
        self.positions = np.zeros((30,2))
        self.state=None

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
        ############### Cage ###############

        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        self.p.setXRange(-5, 5)
        self.p.setYRange(-1, 15)
        self.show()
    
    def update_plot_data(self):
        if self.state is not None:
            x,y=self.state[:2]
            self.vehicle.setPos(x,y)
            self.vehicle.setStyle(angle=self.state[2])
            self.trace.setData(x=self.positions[:,0], y=self.positions[:,1])
            self.i += 1

    def start_recording_mission(self):
        # Reinitialize the data
        print('Mission is recording')
    
    def stop_recording_mission(self):
        # Stop recording and save the data
        print('Mission is over, stopped recording')

    def update_state(self,state):
        self.state=state
        self.positions[:-1]=self.positions[1:]
        self.positions[-1]=state[:2]

    def sawtooth(self, x):
        return (x+pi) % (2*pi)-pi   # or equivalently   2*arctan(tan(x/2))


if __name__=='__main__':
    ################################## Pyqt ##################################
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    sys.exit(app.exec_())
    ################################## Pyqt ##################################
