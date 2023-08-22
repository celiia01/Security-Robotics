import math
import numpy as np

def hom(x: np.array):
    '''
    Return the homogeneus coordinate matrix of the location x

    '''
    return np.array([
        [np.cos(x[2]), -np.sin(x[2]), x[0]],
        [np.sin(x[2]),  np.cos(x[2]), x[1]],
        [			0,			   0,    1]
    ])

def loc(mat: np.array):
    '''
    Retun the location of the homogeneus coordinate matrix (mat)
    '''
    return np.array([mat[0][2], mat[1][2], math.atan2(mat[1][0], mat[0][0])])

def norm(th: float):
    '''
    Return the angle normalised
    '''

    th_n = np.mod(th, 2 * math.pi)

    return th_n if th_n <= math.pi else th_n - 2 * math.pi