import re
import numpy as np
import matplotlib.pyplot as plt
import sys

class Counter(dict):
    def __missing__(self, key):
        return 0

    # return sums

def add_plot(list1,plt,title,xlabel,ylabel,pltnum,ytick):   

    # list1 = sorted(avg.items())
    # y1,x1 = zip(*list1)     

    # print(min(y1),max(y1)+1,(max(y1)-min(y1))/(len(y1)),len(y1))

    plt.subplot(pltnum)
    # if(len(y1) > 1 and len(y1) < 10):
    #     xtick = np.arange(min(y1),max(y1)+1,((max(y1)-min(y1))/(len(y1)-1)))
    #     plt.xticks(xtick)
    
    plt.plot(list1, 'o')
    plt.yticks(ytick)
    plt.tick_params(axis ='x', rotation =-50) 
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

plt.subplots_adjust(left=0.15, bottom=None, right=None, top=None, wspace=0.8, hspace=0.8)

f = open(sys.argv[1],"r")
lines = f.readlines()

q_table1 = []
q_table2 = []
q_table3 = []


for l in lines:
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+",l)
    q_table1.append(float(numbers[0]))
    q_table2.append(float(numbers[1]))
    q_table3.append(float(numbers[2]))
    


# print((0,max(actions),f,(max(actions)/float(pole_pos_count[number]))/5))
# print(ytick)
plt.suptitle(sys.argv[1], fontsize=16)


ymax = max((max(q_table1),max(q_table2),max(q_table3)))
ymin = min((min(q_table1),min(q_table2),min(q_table3)))
print(ymin,ymax)
ytick = np.arange(ymin,ymax+1,(ymax-ymin)/5)
print(ytick)

add_plot(q_table1,plt,"left","","",131,ytick)
add_plot(q_table2,plt,"nothing","","",132,ytick)
add_plot(q_table3,plt,"right","","",133,ytick)


plt.show()