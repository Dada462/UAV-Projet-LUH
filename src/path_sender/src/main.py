import rospy
import actionlib
from uavr_nav_msgs.msg import FollowPathAction,Path,FollowPathGoal
from geometry_msgs.msg import Pose, Point,Quaternion,Twist,Vector3
from controller_tools.tools import R
import numpy as np
from numpy import pi

def feedback_cb(msg):
    print('Feedback received:', msg)


from scipy.special import comb

def get_bezier_parameters(X, Y, degree=3):
    """ Least square qbezier fit using penrose pseudoinverse.

    Parameters:

    X: array of x data.
    Y: array of y data. Y[0] is the y point for X[0].
    degree: degree of the Bézier curve. 2 for quadratic, 3 for cubic.

    Based on https://stackoverflow.com/questions/12643079/b%C3%A9zier-curve-fitting-with-scipy
    and probably on the 1998 thesis by Tim Andrew Pastva, "Bézier Curve Fitting".
    """
    if degree < 1:
        raise ValueError('degree must be 1 or greater.')

    if len(X) != len(Y):
        raise ValueError('X and Y must be of the same length.')

    if len(X) < degree + 1:
        raise ValueError(f'There must be at least {degree + 1} points to '
                         f'determine the parameters of a degree {degree} curve. '
                         f'Got only {len(X)} points.')

    def bpoly(n, t, k):
        """ Bernstein polynomial when a = 0 and b = 1. """
        return t ** k * (1 - t) ** (n - k) * comb(n, k)
        #return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

    def bmatrix(T):
        """ Bernstein matrix for Bézier curves. """
        return np.matrix([[bpoly(degree, t, k) for k in range(degree + 1)] for t in T])

    def least_square_fit(points, M):
        M_ = np.linalg.pinv(M)
        return M_ * points

    T = np.linspace(0, 1, len(X))
    M = bmatrix(T)
    points = np.array(list(zip(X, Y)))
    
    final = least_square_fit(points, M).tolist()
    final[0] = [X[0], Y[0]]
    final[len(final)-1] = [X[len(X)-1], Y[len(Y)-1]]
    return final

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

def bezier_curve(points, nTimes=50):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals


def main():
    
    
    client = actionlib.SimpleActionClient('followPath', FollowPathAction)
    client.wait_for_server()
    print('Sending path')
    p=Path()
    
    # Stuff for obs. avoidance
    liney=lambda t : np.array([0*t,-t,0*t])+np.array([0,0,0.4])
    liney_range=(0,24)

    linex=lambda t : np.array([-t,0*t,0*t])+np.array([0,0,0.4])
    linex_range=(0,15)

    half_circle=lambda t : np.array([1*np.cos(t)-3,1*np.sin(t),0*t+0.4])
    half_circle_range=(0,pi)
    

    # 1: Line
    line = lambda t: np.array([2*t, -t, 0*t+0.5])
    line_range = (-1,1)
    # 2: U-Turn
    n=6
    a,b=1,4
    uturn=lambda t: np.array([-2+b*np.sign(np.sin(-t))*((1-np.cos(t)**n)**(1/n)),-a*np.cos(t),0*t+0.5])
    uturn_range= (-pi,0)
    
    # uturn=lambda t: np.array([-a*np.cos(t),+b*np.sign(np.sin(t))*((1-np.cos(t)**n)**(1/n)),0*t+0.5])
    # uturn_range= (0,pi)
    
    # 3: Obstacle avoidance 1
    def obs_av_1(t):
        b=np.array([0,16,0])
        X=uturn(t)*(t<pi/2)+(2*uturn(pi/2)-uturn(pi-t))*(1.5*pi>t>=pi/2)+(b+uturn(t))*(1.5*pi<=t)
        return X
    obs_av_1_range=(0,2*pi)

    # 4: Obstacle avoidance 2
    T=1
    A=0.8
    obs_av_2=lambda t : np.array([t,A*np.sin(2*pi*t/T),0*t+1.5])
    obs_av_2_range=(0,3)
    
    # 5: Circular
    circular=lambda t : np.array([5*np.cos(t)*np.sin(0.5*t),2*np.sin(t),1.5])
    circular_range=(0,2*pi)

    # 6: Other paths


    flower=lambda t : R(0.15,'x')@np.array([1*(1+0.25*np.sin(5*t))*np.cos(t),1*(1+0.25*np.sin(5*t))*np.sin(t),0*t+0.5])
    flower_range=(-10,0)
    
    #sin_wave=lambda t : np.array([t+3,1*np.cos(2*pi*t/2.5)+5,0*np.cos(2*pi*t/3)+5])
    # f=lambda t : np.array([1*np.cos(t),1*np.sin(t),0.5])+np.array([0,0,2.5])
    # f=lambda t : np.array([t,-6*t*(t-3)-6,0])+np.array([0,0,1.5])
    # n=2
    # r=2
    # g=lambda t : np.array([r*np.cos(t),np.sign(np.sin(t))*(r**n-(r*np.cos(t))**n)**(1/n),0*t+1.5])
    # p=Path_3D(f,range=[0,6],type='parametric')
    # p=Path_3D(lambda t : (2+sin(10*t))*np.array([cos(t),sin(t),0*t+1]),range=[-10,-9],type='parametric')
    # f=lambda t : np.array([t,t,0*t])
    # f=lambda t : np.array([np.cos(t),np.sin(t),0*t+1.5])
    # f=lambda t : R(0.1*t,'x')@(np.array([5*cos(t),5*sin(t),0*t]))+np.array([0,0,15])
    # f=lambda t : np.array([1*cos(t),1*sin(t),0*t])+np.array([0,0,10])
    # f=lambda t : R(t,'y')@np.array([5,0,0])+np.array([0,0,10])+np.array([0*t,sin(15*t),0*t])
    # points=[]
    # for t in np.linspace(-10,20,4000):
    #     points.append(f(t))
    # points=np.array(points).T
    # self.path_to_follow=Path_3D(points,type='waypoints')
    # f=lambda t : np.array([(0.5+0.5*t)*1.25*np.cos(t),(0.5+0.5*t)*1.25*np.sin(t),0*t+1])
    # f=lambda t : np.array([1.25*np.cos(t),1.25*np.sin(t),0*t+0.5])
    # f=lambda t : R(0.1*t,'x')@(np.array([5*np.cos(t),5*np.sin(t),0*t]))+np.array([0,0,15])
    # f=lambda t : np.array([2*(1.5+np.sin(3.5*t))*np.cos(t),2*(1.5+np.sin(3.5*t))*np.sin(t),0*t+5])
    
   
    

    rng=uturn_range
    f=uturn

    nb_points=250
    plot_points=np.zeros((nb_points,3))
    for i,t in enumerate(np.linspace(*rng,nb_points)):
        p.poses.append(Pose(Point(*f(t)),Quaternion()))
        plot_points[i]=f(t)
        p.velocities.append(Twist(Vector3(2,0,0),Vector3(0,0,0)))
    
    # # Spiral
    # xpoints,ypoints=[-1,-0.8,-0.5,0.5,1,0.5,0],[-2,1.5,1.5,1.5,1.5,1.5,-1.5]
    # a=np.array([xpoints,ypoints])
    # a=R(-pi/2,'2D')@a
    # xpoints,ypoints=a
    # print(np.round(a,2))
    # data = get_bezier_parameters(xpoints, ypoints, degree=3)
    # xvals, yvals = bezier_curve(data, nTimes=350)
    # xvals, yvals = np.flip(xvals),np.flip(yvals)
    # plot_points=np.zeros((350,3))
    # for i in range(350):
    #     plot_points[i]=xvals[i],yvals[i],1.25
    #     p.poses.append(Pose(Point(xvals[i],yvals[i],0.5),Quaternion()))
    #     p.velocities.append(Twist(Vector3(0.5,0,0),Vector3(0,0,0)))
    
    
    # p.poses=np.flip(p.poses)
    ############################## PLOT ##############################
    # import pyqtgraph as pg
    # pg.setConfigOptions(antialias=True)
    # plt = pg.plot(pen={'color': '#0e70ec', 'width': 2}, background='w')
    # plt.resize(1200, 850)
    # plt.move(300, 115)
    # plt.plot(plot_points[:,0], plot_points[:,1], pen={'color': '#0e70ec', 'width': 2})
    # plt.showGrid(x=True, y=True)
    # plt.show()
    # pg.exec()


    goal = FollowPathGoal(path=p)
    client.send_goal(goal,feedback_cb=feedback_cb)
    print('Path sent')
    # client.cancel_goal()
    client.wait_for_result()
    print('The result is: ',client.get_result())  # A FibonacciResult

if __name__ == '__main__':
    rospy.init_node('pathClient')
    result = main()