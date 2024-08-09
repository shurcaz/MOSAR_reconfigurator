import numpy as np

# A library of useful utility functions used to increase the speed of basic operations

def increment_tuple(pos, increment):
    return tuple([x + increment for x in pos])

def increment_tuple_val(pos, index, increment):
    a = list(pos)
    a[index] += increment
    return tuple(a)

def max_min(array):
    min = 0
    max = 0
    for n in np.asarray(array).flatten():
        if n > max:
            max = n
        if n < min:
            min = n
    return max, min