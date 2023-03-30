from tools_3D import R,Path
import numpy as np
from numpy import pi,cos,sin
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5.QtCore import Qt
from pyqtgraph.Qt import QtCore

# import pyqtgraph.examples
# pyqtgraph.examples.run()

pg.setConfigOptions(antialias=True)
app = pg.mkQApp("GLLinePlotItem Example")
w = gl.GLViewWidget()
w.show()
w.setWindowTitle('pyqtgraph example: GLLinePlotItem')
w.setCameraPosition(distance=40)

gx = gl.GLGridItem()
gx.rotate(90, 0, 1, 0)
gx.translate(-10, 0, 0)
w.addItem(gx)
gy = gl.GLGridItem()
gy.rotate(90, 1, 0, 0)
gy.translate(0, -10, 0)
w.addItem(gy)
gz = gl.GLGridItem()
gz.translate(0, 0, -10)
w.addItem(gz)

def f(t):
    x = 5*np.cos(t)
    y = 5*np.sin(0.9*t)
    z=t
    return np.array([x,y,z])

# def f(t):
#     x = t**2
#     y = t
#     z=t
#     return np.array([x,y,z])

p=Path(f,[-10,10],type='parametric')
F=p.local_info(p.s)
pts=F.X.T
plt = gl.GLLinePlotItem(pos=pts, color='red', width=3, antialias=True)





pos=np.zeros(3)
sp1 = gl.GLScatterPlotItem(pos=pos,size=0.5,color=(0.2,0.96,0.25,1), pxMode=False)

w.addItem(plt)
w.addItem(sp1)

import pyqtgraph.opengl as gl
import numpy as np

# create the arrow item
s1_arrow = gl.GLLinePlotItem(width=3, color=(0, 0, 1, 0.8))
y1_arrow = gl.GLLinePlotItem(width=3, color=(0, 1, 0, 0.8))
w1_arrow = gl.GLLinePlotItem(width=3, color=(1, 0.5, 0.5, 0.8))

# add the arrow item to the view
w.addItem(s1_arrow)
w.addItem(y1_arrow)
w.addItem(w1_arrow)


t=0
def update_plot_data():
    global pts,t,pos,s1_arrow,y1_arrow,F
    F=p.local_info(t//3)
    pos=F.X.T
    s1=np.vstack((pos,pos+2*F.s1))
    y1=np.vstack((pos,pos+2*F.y1))
    w1=np.vstack((pos,pos+2*F.w1))
    # Rpsi=np.vstack((F.s1,F.y1,F.w1)).T
    # s1=Rpsi@np.array([1,0,0])
    # y1=Rpsi@np.array([0,1,0])
    # w1=Rpsi@np.array([0,0,1])

    # s1=np.vstack((pos,pos+2*s1))
    # y1=np.vstack((pos,pos+2*y1))
    # w1=np.vstack((pos,pos+2*w1))
    
    sp1.setData(pos=pos)
    s1_arrow.setData(pos=s1)
    y1_arrow.setData(pos=y1)
    w1_arrow.setData(pos=w1)
    t+=1

timer = QtCore.QTimer()
timer.setInterval(75)
timer.timeout.connect(update_plot_data)
timer.start()
pg.exec()