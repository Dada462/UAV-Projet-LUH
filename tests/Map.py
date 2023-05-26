import numpy as np
from time import time

class Map:
    def __init__(self, origin=np.zeros(3),real_map_size=10, map_size=500):
        self.real_map_size=real_map_size
        self.map_size = map_size
        self.data = np.zeros((self.map_size, self.map_size,self.map_size), dtype=np.uint8)
        self.discretization_block_size = real_map_size/map_size  # discretization of the map for the graph (in pixels)

    def X_to_Map(self, X):
        X = np.floor(X/self.discretization_block_size+0.5).astype(int)-self.map_size//2
        return X

    def Map_to_X(self, X):
        X = X * self.discretization_block_size
        return X
    
    def __call__(self, X):
        t0=time()
        y = self.X_to_Map(X)
        # np.zeros((self.map_size, self.map_size,self.map_size), dtype=np.uint8)
        self.data[y[0],y[1],y[2]] = 1
        # print((time()-t0)*1000)

if __name__=='__main__':
    m=Map(map_size=500,real_map_size=5)
    t1=time()
    i=0
    X=np.random.random((3,10000))*2-1
    while time()-t1<5:
        m(X)
        i+=1
    print('end',i,i/5)
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
