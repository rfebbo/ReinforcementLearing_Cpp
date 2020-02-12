import re
import numpy as np
import matplotlib.pyplot as plt

f = open("100000-test","r")
lines = f.readlines()

x1 = []
x2 = []
x3 = []
x4 = []
y = []


for l in lines:
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+",l)
    y.append(float(numbers[0]))
    x1.append(float(numbers[1])*180/np.pi)
    x2.append(float(numbers[2]))
    x3.append(float(numbers[3]))
    x4.append(float(numbers[4]))


plt.subplot(2,2,1)
plt.plot(y, x1,'b')
plt.subplot(2,2,2)
plt.plot(y, x3,'g')
plt.subplot(2,2,3)
plt.plot(y, x2,'r')
plt.subplot(2,2,4)
plt.plot(y, x4,'y')
plt.show()