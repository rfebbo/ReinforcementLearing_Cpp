import re
import numpy as np
import matplotlib.pyplot as plt
import sys

plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=0.3, hspace=0.8)

f = open(sys.argv[1],"r")
lines = f.readlines()

y = [] #time
x1 = [] #pole angle
x2 = [] #cart position
x3 = [] #pole velocity
x4 = [] #cart velocity
x5 = [] #force applied
# x1 = np.array()

read_numbers = False


for l in lines:
    if(not read_numbers):
        if(l.rstrip("\n") == "SIMULATION BEGIN"):
            read_numbers = True
        else: 
            print(l.rstrip("\n"))

    elif(read_numbers):
        numbers = re.findall(r"[-+]?\d*\.\d+|\d+",l)
        y.append(float(numbers[0]))
        x1.append(float(numbers[1])*180/np.pi)
        x2.append(float(numbers[2]))
        x3.append(float(numbers[3])*180/np.pi)
        x4.append(float(numbers[4]))
        x5.append(float(numbers[5]))


plt.subplot(2,2,1)
plt.plot(y, x1, 'b')
plt.title('pole angle')
plt.xlabel('time(s)')
plt.ylabel('angle (deg)')

plt.subplot(2,2,2)
plt.plot(y, x2, 'g')
plt.xlabel('time (s)')
plt.title('cart position')
plt.ylabel('position (m)')

plt.subplot(2,2,3)
plt.plot(y, x3, 'r')
plt.xlabel('time (s)')
plt.title('pole velocity')
plt.ylabel('velocity (deg/s)')

plt.subplot(2,2,4)
plt.plot(y, x4, 'y')
plt.xlabel('time (s)')
plt.title('cart velocity')
plt.ylabel('velocity (m/s)')

# plt.subplot(3,2,5)
# plt.plot(y, x5, 'm')
# plt.xlabel('time (s)')
# plt.title('force applied')
# plt.ylabel('force (N)')

plt.show()