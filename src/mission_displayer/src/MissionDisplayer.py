import pyqtgraph as pg
import numpy as np
import sys
import rospy
import threading
from numpy import pi
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QPainter, QBrush, QPen
from std_msgs.msg import Float64

pg.setConfigOptions(antialias=True)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graphWidget)
        self.resize(850, 600)
        self.setWindowTitle('Mission Displayer')

        # Test data
        self.usbl_data = None
        self.heading = 0.
        self.postions = []

        self.vehicle = pg.ArrowItem(
            angle=180, tipAngle=30, baseAngle=-30, headLen=40, tailLen=None, brush=(40, 138, 241, 180))
        self.vehicle.setPos(0, 0)

        pen = pg.mkPen(color='#288af1', width=3, style=QtCore.Qt.DashLine)

        # Creating the widgets
        self.graphWidget.setBackground('w')
        # self.graph_item = pg.GraphItem(pen=pen,symbol='star',symbolSize=30, symbolBrush='#288af1',name='Sensor 1')
        self.graph_item = pg.GraphItem(pen=pen)
        self.p = self.graphWidget.addPlot()

        # Setting the plot
        self.p.setLabel('left', 'y position (m)', **
                        {'color': 'r', 'font-size': '20px'})
        self.p.setLabel('bottom', 'x position (m)', **
                        {'color': 'r', 'font-size': '20px'})
        self.p.addLegend()
        self.p.showGrid(x=True, y=True)
        self.p.setTitle("ROV Mission Displayer", color="k", size="20px")

        self.p.addItem(self.vehicle)
        self.p.addItem(self.graph_item)
        self.i = 0

        ############### Cage ###############
        # Cage
        s1 = pg.ScatterPlotItem(size=50, pen=pg.mkPen(
            None), symbol='s', brush=pg.mkBrush(255, 0, 0, 225))
        spots = [{'pos': [0, 0]}]
        s1.addPoints(spots)
        self.p.addItem(s1)

        # Line
        curve = pg.PlotCurveItem(
            pen=({'color': '#12f843', 'width': 3}), skipFiniteCheck=True)
        self.p.addItem(curve)
        curve.setData(x=[0, 0], y=[0, 1000])

        # Position trace
        self.trace = pg.PlotCurveItem(pen=(
            {'color': '#f12828', 'width': 3, 'style': QtCore.Qt.DashLine}), skipFiniteCheck=True)
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
        if self.usbl_data != None:
            self.vehicle.setPos(self.usbl_data.position.y,
                                -self.usbl_data.position.x)
            # print('heading=',self.heading,'Cage heading=',self.NED_to_cage(self.heading))
            self.vehicle.setStyle(angle=270-self.NED_to_cage(self.heading))
            X = np.array(self.postions)
            self.trace.setData(x=X.T[1], y=-X.T[0])
            self.i += 1

    def NED_to_cage(self, heading):
        cage_heading = 90  # deg
        return -self.sawtooth((heading-cage_heading)/180*pi)*180/pi

    def sawtooth(self, x):
        return (x+pi) % (2*pi)-pi   # or equivalently   2*arctan(tan(x/2))


def usbl_callback(data):
    global main
    try:
        main.usbl_data = data
        main.postions.append(
            [data.position.x, data.position.y, data.position.z])
    except:
        pass


def ros_compass(data):
    global main
    try:
        main.heading = data.data
    except:
        pass


################################## ROS ##################################
rospy.init_node('usbl_data_viewer', anonymous=True)
rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, ros_compass)
ros_thread = threading.Thread(target=rospy.spin)
ros_thread.start()
################################## ROS ##################################


################################## Pyqt ##################################
app = QtWidgets.QApplication(sys.argv)
main = MainWindow()
sys.exit(app.exec_())
################################## Pyqt ##################################
