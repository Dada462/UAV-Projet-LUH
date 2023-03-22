import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.insert(1, './src/pf_controller/src')
from tools import path_info_update


positions=np.load('results/positions_15.03.2023_14.23.17.npy')
path=np.load('results/path_to_follow_15.03.2023_14.23.17.npy')

# print(positions.shape)
# print(positions[:,-1])
# print(path.shape)
# plt.plot(path[:,0],path[:,1],c='red')
t=np.argmin(positions[:,-1])
# plt.plot(positions[:t,-1],positions[:t,1],c='red')
# plt.plot(positions[:t,-1]+5,path[:t,1],c='navy')

from scipy.interpolate import interp1d,interp2d

t_min,t_max=np.min(positions[:,-1]),np.max(positions[:,-1])
interp_times=np.linspace(0,1,t)
x = interp1d(interp_times,positions[:t,0], kind='linear')
y = interp1d(interp_times,positions[:t,1], kind='linear')
pathx = interp1d(interp_times,path[:t,0], kind='linear')
pathy = interp1d(interp_times,path[:t,1], kind='linear')

t=np.linspace(0,1,10000)
# plt.plot(pathx(t),pathy(t))
# plt.plot(x(t),y(t))

path=np.array([pathx(t),pathy(t)])
traj=np.array([x(t),y(t)])
err=path-traj
# print(err.shape)
plt.plot(t,err.T)
print(np.mean(np.abs(err),axis=1))

plt.show()