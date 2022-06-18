import numpy as np

def normalizeVect (vector):
    result = np.subtract(vector, vector[0])
    result = np.divide(result, np.subtract(vector[-1],vector[0]))
    return result

def identify (v1, v2, v3):
    norm = np.asmatrix([
        [v1, np.max(np.diff(normalizeVect(v1)))],
        [v2, np.max(np.diff(normalizeVect(v2)))],
        [v3, np.max(np.diff(normalizeVect(v3)))]
    ])
    norm = norm[np.argsort(norm.A[:, 1])]

    ## FIXME convert from matrix to list
    return norm[0][0], norm[2][0], norm[1][0] # time, step, response
