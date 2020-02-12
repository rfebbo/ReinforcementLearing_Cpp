import re
import numpy as np
import matplotlib.pyplot as plt

f = open("test","r")
lines = f.readlines()

x1 = [] #time
x2 = [] #pole angle
x3 = [] #cart position
x4 = [] #pole velocity
x5 = [] #force applied
y = []


for l in lines:
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+",l)
    y.append(float(numbers[0]))
    x1.append(float(numbers[1])*180/np.pi)
    x2.append(float(numbers[2]))
    x3.append(float(numbers[3]))
    x4.append(float(numbers[4]))
    x5.append(float(numbers[5]))


plt.subplot(3,2,1)
plt.plot(y, x1,'b')
plt.subplot(3,2,2)
plt.plot(y, x3,'g')
plt.subplot(3,2,3)
plt.plot(y, x2,'r')
plt.subplot(3,2,4)
plt.plot(y, x4,'y')
plt.subplot(3,2,5)
plt.plot(y, x5,'m')
plt.show()