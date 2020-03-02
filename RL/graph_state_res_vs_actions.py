import re
import numpy as np
import matplotlib.pyplot as plt
import sys

class Counter(dict):
    def __missing__(self, key):
        return 0

def average_counts(sums,counts):
    for key in sums.keys():
        sums[key] /= float(counts[key])

    # return sums

def add_plot(avg,plt,title,xlabel,ylabel,pltnum,ytick):   

    list1 = sorted(avg.items())
    y1,x1 = zip(*list1)     

    print(min(y1),max(y1)+1,(max(y1)-min(y1))/(len(y1)),len(y1))

    plt.subplot(pltnum)
    if(len(y1) > 1 and len(y1) < 10):
        xtick = np.arange(min(y1),max(y1)+1,((max(y1)-min(y1))/(len(y1)-1)))
        plt.xticks(xtick)
    
    plt.plot(y1, x1, 'b')
    plt.yticks(ytick)
    plt.tick_params(axis ='x', rotation =-50) 
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

plt.subplots_adjust(left=0.15, bottom=None, right=None, top=None, wspace=0.8, hspace=0.8)

f = open(sys.argv[1],"r")
lines = f.readlines()

pole_pos_sum = Counter()
pole_pos_count = Counter()
pole_vel_sum = Counter()
pole_vel_count = Counter()
cart_pos_sum = Counter()
cart_pos_count = Counter()
cart_vel_sum = Counter()
cart_vel_count = Counter()

actions = []
number = -1

for l in lines:
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+",l)
    if(number == -1):
        number = float(numbers[0])
    actions.append(float(numbers[4]))
    pole_pos_sum[float(numbers[0])] += float(numbers[4])
    pole_pos_count[float(numbers[0])] += float(1)
    pole_vel_sum[float(numbers[1])] += float(numbers[4])
    pole_vel_count[float(numbers[1])] += float(1)
    cart_pos_sum[float(numbers[2])] += float(numbers[4])
    cart_pos_count[float(numbers[2])] += float(1)
    cart_vel_sum[float(numbers[3])] += float(numbers[4])
    cart_vel_count[float(numbers[3])] += float(1)


# print((0,max(actions),f,(max(actions)/float(pole_pos_count[number]))/5))
# print(ytick)
plt.suptitle(sys.argv[1], fontsize=16)

average_counts(pole_pos_sum, pole_pos_count)
average_counts(pole_vel_sum,pole_vel_count)
average_counts(cart_pos_sum,cart_pos_count)
average_counts(cart_vel_sum,cart_vel_count)

ymax = max((max(pole_pos_sum.values()),max(pole_vel_sum.values()),max(cart_pos_sum.values()),max(cart_vel_sum.values())))
ymin = min((min(pole_pos_sum.values()),min(pole_vel_sum.values()),min(cart_pos_sum.values()),min(cart_vel_sum.values())))
print(ymin,ymax)
ytick = np.arange(ymin,ymax+1,(ymax-ymin)/5)
print(ytick)

add_plot(pole_pos_sum,plt,"pole angle states","number of pole angle states","avgerage actions taken",221,ytick)
add_plot(pole_vel_sum,plt,"pole vel states","number of pole vel states","avgerage actions taken",222,ytick)
add_plot(cart_pos_sum,plt,"cart pos states","number of cart pos states","avgerage actions taken",223,ytick)
add_plot(cart_vel_sum,plt,"cart vel states","number of cart vel states","avgerage actions taken",224,ytick)


plt.show()