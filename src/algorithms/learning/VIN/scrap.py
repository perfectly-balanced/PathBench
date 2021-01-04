import numpy as np

arr1 = np.array([[1 ,1,0], [0, 1, 1], [1,0,0]])

arr2 = np.array([[1 ,0,0], [1, 0, 1], [1,1,1]])

G = np.logical_or.reduce((arr1, arr2))
W = np.array(G, dtype=np.int8)
M = np.maximum(arr1,arr2)
Q = np.intersect2d
print(G)
print(M)
print(W)

