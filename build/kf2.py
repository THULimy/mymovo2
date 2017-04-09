
from __future__ import division, print_function
from numpy.random import randn
import copy
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
from filterpy.stats import plot_covariance_ellipse

sensor = open("allNOISEresults00.txt")
line = sensor.readline()
zs = []
while line:
    num1 = float(line.strip().split(" ")[0])
    num2 = float(line.strip().split(" ")[2])
    zs.append(np.array([num1,num2]))
    line = sensor.readline()
zs = np.array(zs)   
zs = np.reshape(zs,(zs.shape[0],2,1))
sensor.close() 

GT = open("GTresults00.txt")
line = GT.readline()
gtvdata = []
gtpdata = []
while line:
    num1 = float(line.strip().split(" ")[0])
    num2 = float(line.strip().split(" ")[2])
    num3 = float(line.strip().split(" ")[3])
    num4 = float(line.strip().split(" ")[5])   
    gtvdata.append(np.array([num1,num2]))
    gtpdata.append(np.array([num3,num4]))
    line = GT.readline()
gtvdata = np.array(gtvdata)   
gtvdata = np.reshape(gtvdata,(gtvdata.shape[0],2,1))
gtpdata = np.array(gtpdata)   
gtpdata = np.reshape(gtpdata,(gtpdata.shape[0],2,1))
GT.close()  

VO = open("VOresults00.txt")
line = VO.readline()
vovdata = []
vopdata = []
while line:
    num1 = float(line.strip().split(" ")[0])
    num2 = float(line.strip().split(" ")[2])
    num3 = float(line.strip().split(" ")[3])
    num4 = float(line.strip().split(" ")[5]) 
    vovdata.append(np.array([num1,num2]))
    vopdata.append(np.array([num3,num4]))
    line = VO.readline()
vovdata = np.array(vovdata)   
vovdata = np.reshape(vovdata,(vovdata.shape[0],2,1))
vopdata = np.array(vopdata)   
vopdata = np.reshape(vopdata,(vopdata.shape[0],2,1))
VO.close()  

deltavdata=vovdata-gtvdata
print(deltavdata)

R_std = 10.35
Q_std = 0.04

def tracker1():
    tracker = KalmanFilter(dim_x=4, dim_z=2)
    dt = 1.0   # time step

    tracker.F = np.array([[1, dt, 0,  0],
                          [0,  1, 0,  0],
                          [0,  0, 1, dt],
                          [0,  0, 0,  1]])
    tracker.u = 0.
    tracker.H = np.array([[1, 0, 0, 0],
                          [0, 0, 1, 0]])

    tracker.R = np.eye(2) * R_std**2
    q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_std**2)
    tracker.Q = block_diag(q, q)
    tracker.x = np.array([[0, 0, 0, 0]]).T
    tracker.P = np.eye(4) * 500.
    return tracker

plt.figure()
# simulate robot movement
N = 1000
#sensor = open

#zs = np.array([np.array([sensor.read()]).T for _ in range(N)])

# run filter
car_tracker = tracker1()
mu, cov, _, _ = car_tracker.batch_filter(deltavdata)

for x, P in zip(mu, cov):
    # covariance of x and y
    cov = np.array([[P[0, 0], P[2, 0]], 
                    [P[0, 2], P[2, 2]]])
    mean = (x[0, 0], x[2, 0])
    #plot_covariance_ellipse(mean, cov=cov, fc='g', std=3, alpha=0.5)
print(mu[:,2])
temp=np.concatenate((mu[:,0],mu[:,2]),axis=1)
temp=np.reshape(temp,(temp.shape[0],2,1))
newvovdata=vovdata-temp

print(temp)

x=0
y=0
fusionresult=[]
for i in range(newvovdata.shape[0]):
    x=x+newvovdata[i][0]
    y=y+newvovdata[i][1] 
    fusionresult.append(np.array([x,y]))
fusionresult=np.array(fusionresult)

#print(vovdata.shape)    
#plot results
print(mean) 
#plt.autoscale(tight=True)
#plt.plot(mu[:, 0], mu[:, 2],linestyle='-',color='k',lw=2,label="filter")
plt.plot(fusionresult[:, 0], fusionresult[:, 1],linestyle='-',color='b',label="fusionresult")
plt.plot(gtpdata[:, 0], gtpdata[:, 1],linestyle='-',color='r',lw=2,label="ground truth")
plt.plot(vopdata[:, 0], vopdata[:, 1],linestyle='-',color='g',lw=2,label="vo result")
plt.legend(loc=2)
#plt.xlim((0, 20))
plt.show()