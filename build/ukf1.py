
from __future__ import division, print_function
from numpy.random import randn
import copy
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
from filterpy.stats import plot_covariance_ellipse

sensor = open("someNOISEresults00.txt")
line = sensor.readline()
noisevdata = []
noisepdata = []
while line:
    num1 = float(line.strip().split(" ")[0])
    num2 = float(line.strip().split(" ")[2])
    num3 = float(line.strip().split(" ")[3])
    num4 = float(line.strip().split(" ")[5]) 
    noisevdata.append(np.array([num1,num2]))
    noisepdata.append(np.array([num3,num4]))
    line = sensor.readline()
noisevdata = np.array(noisevdata)   
noisevdata = np.reshape(noisevdata,(noisevdata.shape[0],2,1))
noisepdata = np.array(noisepdata)   
noisepdata = np.reshape(noisepdata,(noisepdata.shape[0],2,1))
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

deltavdata=vovdata-noisevdata[:999]

R_std = 0.6
Q_std = 0.005
'''
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
'''
def f_cv(x, dt):
    """ state transition function for a 
    constant velocity aircraft"""
    
    F = np.array([[1, dt, 0,  0],
                  [0,  1, 0,  0],
                  [0,  0, 1, dt],
                  [0,  0, 0,  1]])
    return np.dot(F, x)

def h_cv(x):
    return np.array([x[0], x[2]])

sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=1.)
ukf = UKF(dim_x=4, dim_z=2, fx=f_cv,
          hx=h_cv, dt=dt, points=sigmas)
ukf.x = np.array([0., 0., 0., 0.])
ukf.R = np.diag([0.09, 0.09]) 
ukf.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=1, var=0.02)
ukf.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=1, var=0.02)

N = 1000
#sensor = open

#zs = np.array([np.array([sensor.read()]).T for _ in range(N)])

# run filter
car_tracker = tracker1()

mu1, cov1, _, _ = car_tracker.batch_filter(deltavdata[:250])
car_tracker.R=100*car_tracker.R
mu2, cov2, _, _ = car_tracker.batch_filter(deltavdata[250:400])
car_tracker.R=car_tracker.R/100
mu3, cov3, _, _ = car_tracker.batch_filter(deltavdata[400:600])
car_tracker.R=100*car_tracker.R
mu4, cov4, _, _ = car_tracker.batch_filter(deltavdata[600:800])
car_tracker.R=car_tracker.R/100
mu5, cov5, _, _ = car_tracker.batch_filter(deltavdata[800:999])
'''
for x, P in zip(mu, cov):
    # covariance of x and y
    cov = np.array([[P[0, 0], P[2, 0]], 
                    [P[0, 2], P[2, 2]]])
    mean = (x[0, 0], x[2, 0])
    #plot_covariance_ellipse(mean, cov=cov, fc='g', std=3, alpha=0.5)
print(mu[:,2])
'''
temp1=np.concatenate((mu1[:,0],mu1[:,2]),axis=1)
temp1=np.reshape(temp1,(temp1.shape[0],2,1))
temp2=np.concatenate((mu2[:,0],mu2[:,2]),axis=1)
temp2=np.reshape(temp2,(temp2.shape[0],2,1))
temp3=np.concatenate((mu3[:,0],mu3[:,2]),axis=1)
temp3=np.reshape(temp3,(temp3.shape[0],2,1))
temp4=np.concatenate((mu4[:,0],mu4[:,2]),axis=1)
temp4=np.reshape(temp4,(temp4.shape[0],2,1))
temp5=np.concatenate((mu5[:,0],mu5[:,2]),axis=1)
temp5=np.reshape(temp5,(temp5.shape[0],2,1))

temp=np.concatenate((temp1,temp2,temp3,temp4,temp5),axis=0)
newvovdata=vovdata-temp

x=0;y=0
noisevresult=[]
for i in range(noisevdata.shape[0]):
    x=x+noisevdata[i][0]
    y=y+noisevdata[i][1] 
    noisevresult.append(np.array([x,y]))
noisevresult=np.array(noisevresult)
'''
noisevdata.shape[0]
for i in range(noisevdata.shape[0]-1):
    deltavdata=noisevdata[i]-vovdata[i]
    car_tracker.predict()
    car_tracker.update(deltavdata)
    print("check:")
    print(deltavdata)
    print(car_tracker.x)
    if i < 998:
        noisevdata[i+1][0][0]=noisevdata[i+1][0][0]- car_tracker.x[0]
        noisevdata[i+1][1][0]=noisevdata[i+1][1][0]- car_tracker.x[2]
 '''

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
#print(mean) 
#plt.autoscale(tight=True)
plt.plot(noisevresult[:, 0], noisevresult[:, 1],linestyle='-',color='k',lw=2,label="noise result")
plt.plot(fusionresult[:, 0], fusionresult[:, 1],linestyle='-',color='b',label="fusionresult")
plt.plot(gtpdata[:, 0], gtpdata[:, 1],linestyle='-',color='r',lw=2,label="ground truth")
plt.plot(vopdata[:, 0], vopdata[:, 1],linestyle='-',color='g',lw=2,label="vo result")
plt.legend(loc=2)
#plt.xlim((0, 20))
plt.show()