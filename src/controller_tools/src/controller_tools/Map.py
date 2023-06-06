import numpy as np
from time import time

class Map:
    def __init__(self,real_map_size=10, map_size=500):
        self.real_map_size=real_map_size
        self.map_size = map_size
        self.data = np.zeros((self.map_size, self.map_size,self.map_size), dtype=np.uint8)
        self.discretization_block_size = real_map_size/map_size  # discretization of the map for the graph (in pixels)
        self.p=np.zeros((self.map_size, self.map_size,self.map_size,3), dtype=np.uint8)

    def X_to_Map(self, X):
        X = np.floor(X/self.discretization_block_size+0.5).astype(int)-self.map_size//2
        return X

    def Map_to_X(self, X):
        X = (X-self.map_size//2) * self.discretization_block_size
        return X
    def array_to_points(self):
        X=np.where(self.data==1)
        X=np.array([X[0],X[1],X[2]])
        return self.Map_to_X(X).T

    def __call__(self, X):
        y = self.X_to_Map(X)
        self.data[y[0],y[1],y[2]] = 1

if __name__=='__main__':
    m=Map(map_size=500,real_map_size=5)
    t1=time()
    X=np.random.random((3,10000))*2-1
    m(X)
    m.array_to_points()
    # import cv2 as cv
    # cv.imshow('test',m.data[:,:,0]*244)
    # cv.waitKey(0)
    # n=5
    # a=np.arange(0,n**2,1).reshape((n,n))
    # y=np.array([[0,-1,3],
    #             [1,2,1]])
    # a[y[0],y[1]]=69
    # print(a)

    # plt.scatter(m.data[:,:,0])
    # print(m.data[:,:,0])
