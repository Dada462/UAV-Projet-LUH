import numpy as np
import matplotlib.pyplot as plt

positions=np.load('results/positions_15.03.2023_14.23.17.npy')
path=np.load('results/path_to_follow_15.03.2023_14.23.17.npy')

print(positions.shape)
print(positions[:,-1])
# plt.plot(path[:,0],path[:,1],c='red')
t=np.argmin(positions[:,-1])
print(t)
plt.plot(positions[:t,-1],positions[:t,0],c='red')
plt.show()