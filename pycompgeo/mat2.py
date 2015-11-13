def scale(v):
    return (v[0], 0,
            0, v[1])

def fromColumns(col1, col2):
    return (col1[0], col2[0], 
            col1[1], col2[1])

def mul(mat, vector):
    return (mat[0] * vector[0] + mat[1] * vector[1], 
            mat[2] * vector[0] + mat[3] * vector[1])

def determinant(mat):
    return mat[0] * mat[3] - mat[1] * mat[2]

def trace(mat):
    return mat[0] + mat[3]