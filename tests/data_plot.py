import numpy as np
# import pyqtgraph as pg
# n=1000
# g=9.81
# tmax=100
# c=np.linspace(0,tmax,n)
# s=np.sqrt(g/(2*c))
# X0=5
# x=X0
# cst=0.5
# # z=np.arcsinh(np.exp(-c)*np.sinh(X0))
# z=np.arcsinh(np.sinh(X0)*np.exp(-cst*c) + np.cosh(X0))
# dt=tmax/n
# y=[]
# def f(x):
#     return -np.tanh(x)+cst
# for t in np.linspace(0,tmax,n):
#     y.append(x)
#     x=x+dt*f(x)
# w=pg.plot(c,y,background='w',pen={'color':'#FF0000','width':3},grid=True)
# w.plot(c,z,background='w',pen={'color':'#0000FF','width':3},grid=True)

# t=np.linspace(-10,10,1000)
# f=lambda t : np.array([1*(1+0.25*np.sin(4*t))*np.cos(t),1*(1+0.25*np.sin(4*t))*np.sin(t),0*t+10])
# w=pg.plot(f(t)[0],f(t)[1],background='w',pen={'color':'#FF0000','width':3},grid=True)
# w.showGrid(x=True,y=True)
# pg.exec()



import rospy
from geometry_msgs.msg import PoseStamped
def callback(msg):
    print(msg)
rospy.init_node('Test')
s=rospy.Subscriber('vision',PoseStamped,callback)
rospy.spin()


