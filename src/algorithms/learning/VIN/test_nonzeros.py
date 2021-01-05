import numpy as np
a = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
print(a>3) #Where is A>3?

'''
[[False False False]
 [ True  True  True]
 [ True  True  True]]

gives the above. 
So in 0th list, none are true. Then you have 1st list, 0th is true. 1st list 1th is true. 1st list 2nd is true
so you have 1-0, 1-1, 1-2, where 1st # is the list # and 2nd # is the index in that list  
In the 2nd list (3rd one), 0th is true, 1st etc.

'''
print(np.nonzero(a>3))
'''
(array([1, 1, 1, 2, 2, 2]), array([0, 1, 2, 0, 1, 2])) 
gives above. So the first array is the list #, and 2nd array is the index within that list
so list 1  number 0, list 1 number 1, list 1 number 2
list 2 number 0, list 2 number 1, list 2 number 2

'''