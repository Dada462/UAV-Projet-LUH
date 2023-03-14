# import pyqtgraph.examples
# pyqtgraph.examples.run()

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.parametertree as ptree
from time import perf_counter

translate = QtCore.QCoreApplication.translate

app = pg.mkQApp()

# pt = ptree.ParameterTree(showHeader=False)
# param = ptree.Parameter.create(name=translate("ScatterPlot", "Parameters"), type="group")
# pt.setParameters(param)
p = pg.PlotWidget()
p1 = pg.PlotWidget()
splitter = QtWidgets.QSplitter()
# splitter.addWidget(pt)
splitter.addWidget(p)
splitter.addWidget(p1)
splitter.setSizes([300, p.width()])
splitter.show()

w = pg.GraphicsLayoutWidget()
p1 = w.addPlot(row=0, col=0)
p2 = w.addPlot(row=0, col=1)
p=w.addItem()
v = w.addViewBox(row=1, col=0, colspan=2)
w.show()
pg.exec()