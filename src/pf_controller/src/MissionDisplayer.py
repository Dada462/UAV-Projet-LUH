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
        # Test data
        self.positions = [[0,0]]

        self.vehicle = pg.ArrowItem(angle=180, tipAngle=30, baseAngle=-30, headLen=40, tailLen=None, brush=(40, 138, 241, 180))
        self.vehicle.setPos(0, 0)

        pen = pg.mkPen(color='#288af1', width=3, style=QtCore.Qt.DashLine)

        # Creating the widgets
        self.graphWidget.setBackground('w')
        # self.graph_item = pg.GraphItem(pen=pen,symbol='star',symbolSize=30, symbolBrush='#288af1',name='Sensor 1')
        # self.graph_item = pg.GraphItem(pen=pen)
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
        # self.p.addItem(self.graph_item)
        self.i = 0

        ############### Cage ###############
        # Cage
        # s1 = pg.ScatterPlotItem(size=50, pen=pg.mkPen(None), symbol='s', brush=pg.mkBrush(255, 0, 0, 225))
        # spots = [{'pos': [0, 0]}]
        # s1.addPoints(spots)
        # self.p.addItem(s1)

        # Line
        # curve = pg.PlotCurveItem(pen=({'color': '#12f843', 'width': 3}), skipFiniteCheck=True)
        # self.p.addItem(curve)
        # curve.setData(x=[0, 0], y=[0, 1000])

        # Position trace
        self.trace = pg.PlotCurveItem(pen=({'color': '#f12828', 'width': 3, 'style': QtCore.Qt.DashLine}), skipFiniteCheck=True)
        self.p.addItem(self.trace)
        ############### Cage ###############

        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

        self.p.setXRange(-5, 5)
        self.p.setYRange(-1, 15)
        self.show()

    def update_plot_data(self):
        # self.vehicle.setPos(self.usbl_data.position.y,-self.usbl_data.position.x)
        # self.vehicle.setStyle(angle=270-self.NED_to_cage(self.heading))
        X = np.array(self.positions)
        self.trace.setData(x=X.T[1], y=-X.T[0])
        self.i += 1

    def sawtooth(self, x):
        return (x+pi) % (2*pi)-pi   # or equivalently   2*arctan(tan(x/2))


if __name__=='__main__':
    ################################## Pyqt ##################################
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    sys.exit(app.exec_())
    ################################## Pyqt ##################################
