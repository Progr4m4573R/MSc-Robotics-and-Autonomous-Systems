import numpy as np
w = np.zeros((46,), dtype=int)
x = np.zeros((46,), dtype=int)
x[1]=1
x[2]=0
x[0]=1
w[13] = 3
w[14] = 6
w[3] = 1
w[4] = -6
w[23] = 4
w[24] = 5
w[5] = -3.93
w[35] = 2
w[45] = 4
summed = 0
for i in range(46):
    summed += (w[i]*x[i])
    print(summed)

