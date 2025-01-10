# plotting x and y coordinates from 2 csv files

import matplotlib.pyplot as plt
import numpy

data1 = numpy.genfromtxt('estimated_path_right.csv', delimiter=',')
#data2 = numpy.genfromtxt('estimated_path_dual.csv', delimiter=',')
#data3 = numpy.genfromtxt('estimated_path_right.csv', delimiter=',')

# divide by 10
data1 = data1 
# data2 = data2 / 10
#data3 = data3 / 300

plt.plot(data1[:, 0], data1[:, 1], label='camera1')
#plt.plot(data3[:, 0], data3[:, 1], label='camera2')
#plt.plot(data2[:, 0], data2[:, 1], label='dual')
plt.legend()
# plt.savefig('plot.png')
plt.show()