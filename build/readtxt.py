import numpy as np

sensor = open("allNOISEresults00.txt")
line = sensor.readline()
zs = []
while line:
	num1 = float(line.strip().split(" ")[3])
	num2 = float(line.strip().split(" ")[5])
	zs.append(np.array([num1,num2]))
	line = sensor.readline()
zs = np.array(zs)
print zs.shape[1]	
sensor.close()	