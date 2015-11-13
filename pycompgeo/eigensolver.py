import math

########################################
# copied from wild magic 4 eigen solver
########################################
class EigenSolver:
    def __init__(self, matrix):
        self.isRotation = False
        self.matrix = matrix
        self.diagonal = [0.0, 0.0, 0.0]
        self.subdivision = [0.0, 0.0, 0.0]

    def solve(self):
        self.tridiagonal()
        self.qlAlgorithm()
        self.guaranteeRotation()

    def eigenVector(self, i):
        return (self.matrix[i], self.matrix[3 + i], self.matrix[6 + i])

    def sortDescreasingly(self):
        self.tridiagonal()
        self.qlAlgorithm()
        self.decreasingSort()
        self.guaranteeRotation()

    def guaranteeRotation(self):
        if not self.isRotation:
            for i in range(0, 3):
                self.matrix[(i * 3)] *= -1 
                
    def tridiagonal(self):
        m00 = self.matrix[0]
        m01 = self.matrix[1]
        m02 = self.matrix[2]
        m11 = self.matrix[4]
        m12 = self.matrix[5]
        m22 = self.matrix[8]
        self.diagonal[0] = m00
        self.subdivision[2] = 0.0
        
        if math.fabs(m02) > 0.00000001:
            length = math.sqrt(m01 * m01 + m02 * m02)
            invLength = 1.0 / length
            m01 *= invLength
            m02 *= invLength
            q = 2.0 * m01 * m12 + m02 * (m22 - m11)
            self.diagonal[1] = m11 + m02 * q
            self.diagonal[2] = m22 - m02 * q
            self.subdivision[0] = length
            self.subdivision[1] = m12 - m01 * q
            self.matrix[0] = 1.0
            self.matrix[1] = 0.0
            self.matrix[2] = 0.0
            self.matrix[3] = 0.0
            self.matrix[4] = m01
            self.matrix[5] = m02
            self.matrix[6] = 0.0
            self.matrix[7] = m02
            self.matrix[8] = -m01
            self.isRotation = False
        else:
            self.diagonal[1] = m11
            self.diagonal[2] = m22
            self.subdivision[0] = m01
            self.subdivision[1] = m12
            self.matrix[0] = 1.0
            self.matrix[1] = 0.0
            self.matrix[2] = 0.0
            self.matrix[3] = 0.0
            self.matrix[4] = 1.0
            self.matrix[5] = 0.0
            self.matrix[6] = 0.0
            self.matrix[7] = 0.0
            self.matrix[8] = 1.0
            self.isRotation = True

    def qlAlgorithm(self):
        for i0 in range(0, 3):
            i1 = 0
            while i1 < 32:
                i2 = i0

                while i2 <= 1:
                    tmp = math.fabs(self.diagonal[i2]) + math.fabs(self.diagonal[i2 + 1])
                    if (math.fabs(self.subdivision[i2]) + tmp) == tmp:
                        break
                    i2 += 1

                if i2 == i0:
                    break

                g = (self.diagonal[i0 + 1] - self.diagonal[i0]) / (2.0 * self.subdivision[i0])
                r = math.sqrt(g * g + 1.0)

                if g < 0.0:
                    g = self.diagonal[i2] - self.diagonal[i0] + self.subdivision[i0] / (g - r)
                else:
                    g = self.diagonal[i2] - self.diagonal[i0] + self.subdivision[i0] / (g + r)

                sin = 1.0
                cos = 1.0
                p = 0.0

                i3 = i2 - 1
                while i3 >= i0:
                    f = sin * self.subdivision[i3]
                    b = cos * self.subdivision[i3]

                    if math.fabs(f) >= math.fabs(g):
                        cos = g / f
                        r = math.sqrt(cos * cos + 1.0)
                        self.subdivision[i3 + 1] = f * r
                        sin = 1.0 / r
                        cos *= sin
                    else:
                        sin = f / g
                        r = math.sqrt(sin * sin + 1.0)
                        self.subdivision[i3 + 1] = g * r
                        cos = 1.0 / r
                        sin *= cos

                    g = self.diagonal[i3 + 1] - p
                    r = (self.diagonal[i3] - g) * sin + 2.0 * b * cos
                    p = sin * r
                    self.diagonal[i3 + 1] = g + p
                    g = cos * r - b

                    i4 = 0
                    while i4 < 3:
                        f = self.matrix[(i4 * 3) + i3 + 1]
                        self.matrix[(i4 * 3) + i3 + 1] = sin * self.matrix[(i4 * 3) + i3] + cos * f
                        self.matrix[(i4 * 3) + i3] = cos * self.matrix[(i4 * 3) + i3] - sin * f
                        i4 += 1

                    i3 -= 1

                self.diagonal[i0] -= p
                self.subdivision[i0] = g
                self.subdivision[i2] = 0.0
            i1 += 1

            if i1 == 32:
                return False

        return True

    def decreasingSort(self):
        # sort eigenvalues in decreasing order, e[0] >= ... >= e[iSize-1]
        for i0 in range(0, 2):
            # locate maximum eigenvalue
            i1 = i0    
            _max = self.diagonal[i1]    
            i2 = i0 + 1
            while i2 < 3:
                if self.diagonal[i2] > _max:
                    i1 = i2    
                    _max = self.diagonal[i1]
                i2 += 1    

            if i1 != i0:
                # swap eigenvalues
                self.diagonal[i1] = self.diagonal[i0]
                self.diagonal[i0] = _max

                # swap eigenvectors
                i2 = 0
                while i2 < 3:
                    tmp = self.matrix[(i2 * 3) + i0]    
                    self.matrix[(i2 * 3) + i0] = self.matrix[(i2 * 3) + i1]    
                    self.matrix[(i2 * 3) + i1] = tmp    
                    self.isRotation = not self.isRotation
                    i2 += 1
