import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from collections import deque
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

class Line():
    ''' Define line '''
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist  # normalize

    def path(self, t):
        return self.p + t * self.dirn

class Lines():
    ''' Define line '''
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - self.p
        self.dist = np.linalg.norm(self.dirn,axis=1)
        self.dirn = (self.dirn.T/self.dist).T  # normalize

    def path(self, t):
        return self.p + t * self.dirn

class Graph:
    ''' Define graph '''
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos: 0}
        self.neighbors = {0: []}
        self.distances = {0: 0}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]
        self.sz = endpos[2] - startpos[2]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self):
        rx,ry,rz = 2*np.random.random(3)-0.1

        posx = self.startpos[0] +self.sx*(rx)
        posy = self.startpos[1] +self.sy*(ry)
        posz = self.startpos[2] +self.sz*(rz)
        return posx, posy, posz

    def Intersection(self,line, center, radius):
        r2=center-line.p
        norm_sq_r2=np.dot(r2,r2)
        dx=np.dot(r2,line.dirn)
        isIntersecting=(norm_sq_r2-dx**2)<radius**2
        return isIntersecting

    def Intersection_multi(self,line, center, radius):
        ''' Check line-sphere (circle) intersection '''
        a = 1
        b = 2 * np.dot(line.dirn, line.p - center)
        c = np.dot(line.p - center, line.p - center) - radius * radius

        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False

        t1 = (-b + np.sqrt(discriminant)) / (2 * a)
        t2 = (-b - np.sqrt(discriminant)) / (2 * a)

        if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
            return False
        return True


    def distance(self,x, y):
        return np.linalg.norm(np.array(x) - np.array(y))


    def isInObstacle(self,vex, obstacles, radius):
        # print(vex)
        for obs in obstacles:
            # print('obstacle',obs)
            # print('distance and radius',self.distance(obs, vex),radius)
            if self.distance(obs, vex) < radius:
                return True
        return False


    def isThruObstacle(self,line, obstacles, radius):
        for obs in obstacles:
            if self.Intersection(line, obs, radius):
                return True
        return False


    def nearest(self,G, vex, obstacles, radius):
        Nvex = None
        Nidx = None
        minDist = float("inf")
        # t0=time()
        # v=G.vertices
        # lines=Lines(v,[vex]*len(v))
        # print('ppp',lines.p,'ppp')

        for idx, v in enumerate(G.vertices):
            line = Line(v, vex)
            if self.isThruObstacle(line, obstacles, radius):
                continue
            dist = self.distance(v, vex)
            if dist < minDist:
                minDist = dist
                Nidx = idx
                Nvex = v
        # print('this',1000*(time()-t0),' ms')
        return Nvex, Nidx


    def newVertex(self,randvex, nearvex, stepSize):
        dirn = np.array(randvex) - np.array(nearvex)
        length = np.linalg.norm(dirn)
        dirn=dirn*np.clip(length,length,stepSize)
        newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1],nearvex[2]+dirn[2])
        return newvex

    def RRT(self,obstacles, n_iter, radius, stepSize):
        ''' RRT algorithm '''
        for nb_of_iter in range(n_iter):
            randvex = self.randomPosition()
            if self.isInObstacle(randvex, obstacles, radius):
                continue

            nearvex, nearidx = self.nearest(self, randvex, obstacles, radius)
            if nearvex is None:
                continue
            
            newvex = self.newVertex(randvex, nearvex, stepSize)

            newidx = self.add_vex(newvex)
            dist = self.distance(newvex, nearvex)
            self.add_edge(newidx, nearidx, dist)

            dist = self.distance(newvex, self.endpos)
            
            line = Line(newvex, self.endpos)
            if dist < 2 * radius or not self.isThruObstacle(line, obstacles, radius):
                endidx = self.add_vex(self.endpos)
                self.add_edge(newidx, endidx, dist)
                self.success = True
                break
        print('nb of iter',nb_of_iter)
        return self

    def RRT_star(self, obstacles, n_iter, radius, stepSize):
        ''' RRT star algorithm '''

        for _ in range(n_iter):
            randvex = self.randomPosition()
            if self.isInObstacle(randvex, obstacles, radius):
                continue

            nearvex, nearidx = self.nearest(self, randvex, obstacles, radius)
            if nearvex is None:
                continue

            newvex = self.newVertex(randvex, nearvex, stepSize)
            newidx = self.add_vex(newvex)
            dist = self.distance(newvex, nearvex)
            self.add_edge(newidx, nearidx, dist)
            self.distances[newidx] = self.distances[nearidx] + dist

            # update nearby vertices distance (if shorter)
            for vex in self.vertices:
                if vex == newvex:
                    continue

                dist = self.distance(vex, newvex)
                if dist > radius:
                    continue

                line = Line(vex, newvex)
                if self.isThruObstacle(line, obstacles, radius):
                    continue

                idx = self.vex2idx[vex]
                if self.distances[newidx] + dist < self.distances[idx]:
                    self.add_edge(idx, newidx, dist)
                    self.distances[idx] = self.distances[newidx] + dist

            dist = self.distance(newvex, self.endpos)
            if dist < 2 * radius:
                endidx = self.add_vex(self.endpos)
                self.add_edge(newidx, endidx, dist)
                try:
                    self.distances[endidx] = min(
                        self.distances[endidx], self.distances[newidx]+dist)
                except:
                    self.distances[endidx] = self.distances[newidx]+dist

                self.success = True
                break
        return self

    def dijkstra(self):
        '''
        Dijkstra algorithm for finding shortest path from start position to end.
        '''
        srcIdx = self.vex2idx[self.startpos]
        dstIdx = self.vex2idx[self.endpos]

        # build dijkstra
        nodes = list(self.neighbors.keys())
        dist = {node: float('inf') for node in nodes}
        prev = {node: None for node in nodes}
        dist[srcIdx] = 0

        while nodes:
            curNode = min(nodes, key=lambda node: dist[node])
            nodes.remove(curNode)
            if dist[curNode] == float('inf'):
                break

            for neighbor, cost in self.neighbors[curNode]:
                newCost = dist[curNode] + cost
                if newCost < dist[neighbor]:
                    dist[neighbor] = newCost
                    prev[neighbor] = curNode

        # retrieve path
        path = deque()
        curNode = dstIdx
        while prev[curNode] is not None:
            path.appendleft(self.vertices[curNode])
            curNode = prev[curNode]
        path.appendleft(self.vertices[curNode])
        return list(path)
    
    def smoothen_path(self):
        print('Did it succeed?',self.success)
        if self.success:
            path = self.dijkstra()
        else:
            path=[[0,0,0.5],[1,1,0.5]]

        path=np.array(path).T
        s=np.linspace(0,1,len(path[0]))
        path=interp1d(s,path)
        s=np.linspace(0,1,50)
        path=path(s)
        path[0]=savgol_filter(path[0], window_length=40,polyorder=3)
        path[1]=savgol_filter(path[1], window_length=40,polyorder=3)
        path[2]=savgol_filter(path[2], window_length=40,polyorder=3)
        return path

def plot(G, obstacles, radius, path=None):
    '''
    Plot RRT, obstacles and shortest path
    '''
    from mpl_toolkits.mplot3d import Axes3D
    px = [x for x, y, z in G.vertices]
    py = [y for x, y, z in G.vertices]
    pz = [z for x, y, z in G.vertices]
    ax=plt.figure().add_subplot(projection='3d')

    for obs in obstacles:
        # circle = ax.Circle(obs, radius, color='red')
        # u = np.linspace(0, 2 * np.pi, 100)
        # v = np.linspace(0, np.pi, 100)

        # draw sphere
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = np.cos(u)*np.sin(v)
        y = np.sin(u)*np.sin(v)
        z = np.cos(v)
        x = radius*x + obs[0]
        y = radius*y + obs[1]
        z = radius*z + obs[2]

        ax.plot_wireframe(x, y, z, color="r")

    ax.scatter(px, py, pz, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1],G.startpos[2], c='red',s=10)
    ax.scatter(G.endpos[0], G.endpos[1],G.endpos[2], c='black')
    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = Line3DCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = Line3DCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)
    ax.autoscale(tight=1)
    # ax.margins(0.1)
    # ax.set_xlim(0,4)
    # ax.set_ylim(-1,5)
    # ax.set_zlim(0,5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


if __name__ == '__main__':
    from time import time
    from numpy import pi


    startpos = (-0, 2.04, 0.59)
    endpos = (0.0, 4.87, 0.4)
    # T=np.linspace(0,2*pi,10)
    # obstacles=0.5*np.array([np.cos(T),np.sin(T)]).T+np.array([2,2])
    # obstacles=2+0.5*np.random.rand(5,2)
    obstacles=np.array([[-0.15276801 ,2.80242407 ,0.55397379]])

    n_iter = 125
    radius = 0.25
    stepSize = 0.5

    t0=time()
    G=Graph(startpos, endpos)
    G.RRT(obstacles, n_iter, radius, stepSize)
    # G.RRT_star(obstacles, n_iter, radius, stepSize)
    print('end time',1000*(time()-t0),' ms')
    if G.success:
        print('success')
        path=G.smoothen_path()
        plot(G, obstacles, radius, path.T)
    else:
        print('failed')
        plot(G, obstacles, radius)
