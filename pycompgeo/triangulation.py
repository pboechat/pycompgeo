import pycompgeo.utils

###################################################
# source: https://github.com/mrbaozi/triangulation
###################################################

def isClockwise(poly):
    # initialize sum with last element
    sum = (poly[0][0] - poly[len(poly)-1][0]) * (poly[0][1] + poly[len(poly)-1][1])
    # iterate over all other elements (0 to n-1)
    for i in range(len(poly)-1):
        sum += (poly[i+1][0] - poly[i][0]) * (poly[i+1][1] + poly[i][1])
    if sum > 0:
        return True
    return False

def inTriangle(a, b, c, p):
    L = [0, 0, 0]
    # calculate barycentric coefficients for point p
    # pycompgeo.utils.EPSILON is needed as error correction since for very small distances denom->0
    L[0] = ((b[1] - c[1]) * (p[0] - c[0]) + (c[0] - b[0]) * (p[1] - c[1])) \
        /(((b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])) + pycompgeo.utils.EPSILON)
    L[1] = ((c[1] - a[1]) * (p[0] - c[0]) + (a[0] - c[0]) * (p[1] - c[1])) \
        /(((b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])) + pycompgeo.utils.EPSILON)
    L[2] = 1 - L[0] - L[1]
    # check if p lies in triangle (a, b, c)
    for x in L:
        if x >= 1 or x <= 0:
            return False  
    return True

def isConvex(a, b, c):
    # only convex if traversing anti-clockwise!
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if cross >= 0:
        return True 
    return False 

def getEar(poly):
    size = len(poly)
    if size < 3:
        return []
    if size == 3:
        tri = (poly[0], poly[1], poly[2])
        del poly[:]
        return tri
    for i in range(size):
        tritest = False
        p1 = poly[(i-1) % size]
        p2 = poly[i % size]
        p3 = poly[(i+1) % size]
        if isConvex(p1, p2, p3):
            for x in poly:
                if not (x in (p1, p2, p3)) and inTriangle(p1, p2, p3, x):
                    tritest = True
                if tritest == False:
                    del poly[i % size]
                    return (p1, p2, p3)
    #print('getEar(): no ear found')
    return []

def earClipping(poly):
    triangles = []
    pts = poly[::-1] if isClockwise(poly) else poly[:]
    while len(pts) >= 3:
        ear = getEar(pts)
        if ear == []:
            break
        triangles.append(ear)
    return triangles
