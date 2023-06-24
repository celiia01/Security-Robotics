import math
import numpy as np

def hom(x: np.array):
    return np.array([
        [np.cos(x[2]), -np.sin(x[2]), x[0]],
        [np.sin(x[2]),  np.cos(x[2]), x[1]],
        [			0,			   0,    1]
    ])

def loc(mat: np.array):
    return np.array([mat[0][2], mat[1][2], math.atan2(mat[1][0], mat[0][0])])