##########################################################################
# Thanks to Rachel J. Trimble for providing a solution to Workshop8-Task2
##########################################################################

import numpy as np
import tensorflow as tf

e = np.matrix([[1,0,0,1,2],
               [0,0,2,0,1],
               [0,1,0,0,3]])

WQ = np.matrix([[2,0,0],
                [3,0,0],
                [1,1,0],
                [0,1,1],
                [1,1,1]])

WK = np.matrix([[1,2,3],
                [2,0,0],
                [1,2,0],
                [0,0,3],
                [1,2,0]])

WV = np.matrix([[1,1,1],
                [3,0,0],
                [1,1,0],
                [0,1,1],
                [2,0,0]])

q = np.matmul(e,WQ)
k = np.matmul(e,WK)
v = np.matmul(e,WV)
temp1 = np.matmul(q,np.transpose(k)) /np.sqrt(5)
temp2 = tf.nn.softmax(temp1)
z = np.matmul(temp2, v)
print(z)
