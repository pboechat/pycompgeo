import math
import random
import pycompgeo.vec2

EPSILON = 0.0000001

def approx1D(a, b):
    return a >= (b - EPSILON) and a <= (b + EPSILON)

def approx2D(a, b):
    return approx1D(a[0], b[0]) and approx1D(a[1], b[1])

def approx3D(a, b):
    return approx1D(a[0], b[0]) and approx1D(a[1], b[1]) and approx1D(a[2], b[2])

def lerp(t, a, b):
    return a + t * (b - a)

def lerp2D(t, a, b):
    return lerp(t, a[0], b[0]), lerp(t, a[1], b[1])

def isCW(points, axis = (0, 1)):
    sum = 0
    j = len(points) - 1
    for i in range(0, j):
        sum = sum + (points[j][axis[0]] + points[i][axis[0]]) * (points[j][axis[1]] - points[i][axis[1]])
        j = i
    return sum > 0
    
def isCW3D_Convex(points, v = (0, -1, 0)):
    if len(points) < 3:
        raise Exception('polygon has less then 3 sides')
    v0 = points[0]
    v1 = points[1]
    v2 = points[2]
    return vec3.dot(vec3.cross(vec3.sub(v2, v0), vec3.sub(v1, v0)), vec3.sub(v0, v)) < 0

def polygonArea(points):
    area = 0
    j = len(points) - 1
    for i in range(0, j):
        area = area + (points[j][0] + points[i][0]) * (points[j][1] - points[i][1])
        j = i
    return area / 2.0

####################################################################
# source: http://en.wikipedia.org/wiki/Centroid#Centroid_of_polygon
####################################################################
def centroid(points):
    i = 0
    doubleArea = 0
    centroidX = 0
    centroidY = 0
    while i < len(points):
        p0 = points[i - 1]
        p1 = points[i]
        cross = p0[0] * p1[1] - p1[0] * p0[1]
        centroidX += (p0[0] + p1[0]) * cross
        centroidY += (p0[1] + p1[1]) * cross
        doubleArea += cross
        i += 1
    m = 3.0 * doubleArea
    area = doubleArea * 0.5
    if m != 0:
        return (centroidX / m, centroidY / m)
    else:
        return (0, 0)

def orderCW(points, axes = (0, 1)):
    # choose the left bottom most point as anchor
    anchor = points[0]
    i = 0
    for j, p in enumerate(points[1:]):
        if p[axes[0]] < anchor[axes[0]] or (p[axes[0]] == anchor[axes[0]] and p[axes[1]] < anchor[axes[1]]):
            anchor = p
            i = j + 1
    # reverse sort according to angle((0, -1), point - anchor)
    order = sorted(range(0, len(points)), key = lambda k: -1.0 if k == i else pycompgeo.vec2.angle((0, -1), pycompgeo.vec2.sub(points[k], anchor)), reverse = True)
    order = order[:-1]
    order.append(i)
    return order

def orderCCW(points, axes = (0, 1)):
    # choose the left bottom most point as anchor
    anchor = points[0]
    i = 0
    for j, p in enumerate(points[1:]):
        if p[axes[0]] < anchor[axes[0]] or (p[axes[0]] == anchor[axes[0]] and p[axes[1]] < anchor[axes[1]]):
            anchor = p
            i = j + 1
    # sort according to angle((0, -1), point - anchor)
    order = sorted(range(0, len(points)), key = lambda k: -1.0 if k == i else pycompgeo.vec2.angle((0, -1), pycompgeo.vec2.sub(points[k], anchor)))
    order = order[1:]
    order.insert(0, i)
    return order

def sortCW(points, axes = (0, 1)):
    return [points[i] for i in orderCW(points, axes)]

def sortCCW(points, axes = (0, 1)):
    return [points[i] for i in orderCCW(points, axes)]

def armPoints(a, b, c, width):
        a0 = pycompgeo.vec2.normalize(pycompgeo.vec2.sub(a, b))
        a1 = pycompgeo.vec2.normalize(pycompgeo.vec2.sub(c, b))
        
        r = 0.5 * width
        
        dot = pycompgeo.vec2.dot(a0, a1)
        
        if abs(dot) < (1.0 - EPSILON):
            elbow = pycompgeo.vec2.add(b, pycompgeo.collision.lineIntersection((-a0[1], a0[0],  r), (-a1[1], a1[0], -r))) # left
            joint = pycompgeo.vec2.add(b, pycompgeo.collision.lineIntersection((-a0[1], a0[0], -r), (-a1[1], a1[0], r))) # right
        else:
            if dot > EPSILON:
                n = pycompgeo.vec2.normalize(pycompgeo.vec2.add((-a0[1], a0[0]), (-a1[1], a1[0])))
                elbow = pycompgeo.vec2.add(b, pycompgeo.vec2.scale(-r, n)) # left
                joint = pycompgeo.vec2.add(b, pycompgeo.vec2.scale( r, n)) # right
            else:
                n = pycompgeo.vec2.normalize(pycompgeo.vec2.add((-a0[1], a0[0]), ( a1[1],-a1[0])))
                elbow = pycompgeo.vec2.add(b, pycompgeo.vec2.scale(-r, n)) # left
                joint = pycompgeo.vec2.add(b, pycompgeo.vec2.scale( r, n)) # right

        return (elbow, joint)
   
def barycentricCoordinates(triangle, point):
    # det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
    det = (triangle[1][2] - triangle[2][2]) * (triangle[0][0] - triangle[2][0]) + (triangle[2][0] - triangle[1][0]) * (triangle[0][2] - triangle[2][2])
    if det == 0:
        return 0, 0, 0
    # L1 =(y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)
    L1 = ((triangle[1][2] - triangle[2][2]) * (point[0] - triangle[2][0]) + (triangle[2][0] - triangle[1][0]) * (point[1] - triangle[2][2])) / det
    # L2 = (y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)
    L2 = ((triangle[2][2] - triangle[0][2]) * (point[0] - triangle[2][0]) + (triangle[0][0] - triangle[2][0]) * (point[1] - triangle[2][2])) / det
    # L3 = 1 - L1 - L2
    L3 = 1.0 - L1 - L2
    return L1, L2, L3
   
def barycentricInterpolation1D(triangle, L, parameterIdx = 0):
    return L[0] * triangle[0][parameterIdx] + L[1] * triangle[1][parameterIdx] + L[2] * triangle[2][parameterIdx]
    
def barycentricInterpolation1D2(triangle, L1, L2, L3, parameterIdx = 0):
    return L1 * triangle[0][parameterIdx] + L2 * triangle[1][parameterIdx] + L3 * triangle[2][parameterIdx]

def barycentricInterpolation2D(triangle, L, axes = (0, 1)):
    return barycentricInterpolation1D(triangle, L, axes[0]), barycentricInterpolation1D(triangle, L, axes[1])

# TODO: can be optimized because barycentric coordinates are not the optimal solution for continence test
def isPointInsideTriangle(triangle, point):
    return barycentricPointInTriangle2(barycentricCoordinates(triangle, point))

def barycentricPointInTriangle(L1, L2, L3):
    sum = L1 + L2 + L3
    return L1 >= -EPSILON and L2 >= -EPSILON and L3 >= -EPSILON and approx1D(sum, 1.0)

def barycentricPointInTriangle2(L):
    sum = L[0] + L[1] + L[2]
    return L[0] >= -EPSILON and L[1] >= -EPSILON and L[2] >= -EPSILON and approx1D(sum, 1.0)
    
#################################################################################
# triangle point picking: http://mathworld.wolfram.com/TrianglePointPicking.html
#################################################################################
def pickRandomPointInTriangle(triangle):
    o = (triangle[0][0], triangle[0][1])
    v1 = (triangle[1][0] - o[0], triangle[1][1] - o[1])
    v2 = (triangle[2][0] - o[0], triangle[2][1] - o[1])
    a1 = random.random()
    a2 = random.random()
    return (o[0] + a1 * v1[0] + a2 * v2[0], o[1] + a1 * v1[1] + a2 * v2[1])
