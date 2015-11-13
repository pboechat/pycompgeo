import sys
import math
import pycompgeo.vec2
import pycompgeo.vec3
import pycompgeo.mat2
import pycompgeo.mat3
import pycompgeo.mat4
import pycompgeo.eigensolver

def lineIntersection(l1, l2):
    det = l2[0] * l1[1] - l1[0] * l2[1]
    if abs(det) != 0.0:
        return pycompgeo.vec2.scale(1.0 / det, (l2[1] * l1[2] - l1[1] * l2[2], l1[0] * l2[2] - l2[0] * l1[2]))
    return (float('nan'), float('nan'))

def lineSegmentIntersection(p, e0, q, e1):
        r = pycompgeo.vec2.sub(e0, p)
        s = pycompgeo.vec2.sub(e1, q)
        q_m_p = pycompgeo.vec2.sub(q, p)
        r_x_s = pycompgeo.vec2.cross(r, s)
        q_m_p_x_r = pycompgeo.vec2.cross(q_m_p, r)
        if r_x_s == 0:
            if q_m_p_x_r == 0:
                a = pycompgeo.vec2.dot(q_m_p, r)
                b = pycompgeo.vec2.dot(pycompgeo.vec2.sub(p, q), s)
                if (a >= 0 and a <= pycompgeo.vec2.dot(r, r)) or (b >= 0 and b <= pycompgeo.vec2.dot(s, s)):
                    # TODO: improve
                    raise Exception('unsupported case: overlapping lines')
            return None, 0, 0
        else:
            t = pycompgeo.vec2.cross(q_m_p, s) / r_x_s
            u = q_m_p_x_r / r_x_s
            if t >= 0 and t <= 1 and u >= 0 and u <= 1:
                return pycompgeo.vec2.add(p, pycompgeo.vec2.scale(t, r)), t, u
            else:
                return None, 0, 0

class AABB:
    def __init__(self, centroid=(0,0,0), halfExtent=(0,0,0)):
        self.centroid = centroid
        self.halfExtent = halfExtent
        self.vertices = [
            (self.centroid[0] + self.halfExtent[0], self.centroid[1] - self.halfExtent[1], self.centroid[2] + self.halfExtent[2]), 
            (self.centroid[0] - self.halfExtent[0], self.centroid[1] - self.halfExtent[1], self.centroid[2] + self.halfExtent[2]),
            (self.centroid[0] + self.halfExtent[0], self.centroid[1] + self.halfExtent[1], self.centroid[2] + self.halfExtent[2]),
            (self.centroid[0] - self.halfExtent[0], self.centroid[1] + self.halfExtent[1], self.centroid[2] + self.halfExtent[2]), 
            (self.centroid[0] + self.halfExtent[0], self.centroid[1] - self.halfExtent[1], self.centroid[2] - self.halfExtent[2]),
            (self.centroid[0] - self.halfExtent[0], self.centroid[1] - self.halfExtent[1], self.centroid[2] - self.halfExtent[2]),
            (self.centroid[0] + self.halfExtent[0], self.centroid[1] + self.halfExtent[1], self.centroid[2] - self.halfExtent[2]), 
            (self.centroid[0] - self.halfExtent[0], self.centroid[1] + self.halfExtent[1], self.centroid[2] - self.halfExtent[2])
        ]

    def getLargestAxis(self):
        if self.halfExtent[0] > self.halfExtent[1]:
            if self.halfExtent[0] > self.halfExtent[2]:
                return 0
            else:
                return 2
        else:
            if self.halfExtent[1] > self.halfExtent[2]:
                return 1
            else:
                return 2
                

def aabbCollision(a, b):
    if math.fabs(a.centroid[0] - b.centroid[0]) > (a.halfExtent[0] + b.halfExtent[0]): 
        return False
    if math.fabs(a.centroid[1] - b.centroid[1]) > (a.halfExtent[1] + b.halfExtent[1]): 
        return False
    if math.fabs(a.centroid[2] - b.centroid[2]) > (a.halfExtent[2] + b.halfExtent[2]): 
        return False
    return True

def createAABB(points):
    min_ = [sys.maxsize, sys.maxsize, sys.maxsize]
    max_ = [-sys.maxsize, -sys.maxsize, -sys.maxsize]
    for p in points:
        for d in [0,1,2]:
            if p[d] < min_[d]:
                min_[d] = p[d]
            if p[d] > max_[d]:
                max_[d] = p[d]
    halfExtent = ((max_[0] - min_[0]) * 0.5, (max_[1] - min_[1]) * 0.5, (max_[2] - min_[2]) * 0.5)
    centroid = (min_[0] + halfExtent[0], min_[1] + halfExtent[1], min_[2] + halfExtent[2])
    return AABB(centroid, halfExtent)
    
class OBB:
    def __init__(self, axis = [(0,0,0), (0,0,0), (0,0,0)], min_ = (sys.maxsize, sys.maxsize, sys.maxsize), max_ = (-sys.maxsize, -sys.maxsize, -sys.maxsize)):
        self.axis = axis
        self.min = min_
        self.max = max_
        
        self.width = self.max[0] - self.min[0]
        self.height = self.max[1] - self.min[1]
        self.depth = self.max[2] - self.min[2]

        tmp = pycompgeo.vec3.scale(0.5, pycompgeo.vec3.add(self.max, self.min))
        self.centroid = (self.axis[0][0] * tmp[0] + self.axis[1][0] * tmp[1] + self.axis[2][0] * tmp[2],
                         self.axis[0][1] * tmp[0] + self.axis[1][1] * tmp[1] + self.axis[2][1] * tmp[2],
                         self.axis[0][2] * tmp[0] + self.axis[1][2] * tmp[1] + self.axis[2][2] * tmp[2])

def createOBB(points):
    centroid = (0.0, 0.0, 0.0)
    for point in points:
        centroid = pycompgeo.vec3.add(centroid, point)
    centroid = pycompgeo.vec3.scale(1.0 / float(len(points)), centroid)

    covariance = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
    for point in points:
        v = pycompgeo.vec3.sub(point, centroid);

        m11 = v[0] * v[0]
        m12 = v[0] * v[1]
        m13 = v[0] * v[2]
        m21 = m12
        m22 = v[1] * v[1]
        m23 = v[1] * v[2]
        m31 = m13
        m32 = m23
        m33 = v[2] * v[2]

        covariance = pycompgeo.mat3.add(covariance, [m11, m12, m13,
                                          m21, m22, m23,
                                          m31, m32, m33])

    solver = pycompgeo.eigensolver.EigenSolver(covariance);
    solver.sortDescreasingly();
    axis = [None] * 3
    axis[0] = pycompgeo.vec3.normalize(solver.eigenVector(0))
    axis[1] = pycompgeo.vec3.normalize(solver.eigenVector(1))
    axis[2] = pycompgeo.vec3.normalize(solver.eigenVector(2))

    min_ = [sys.maxsize, sys.maxsize, sys.maxsize]
    max_ = [-sys.maxsize, -sys.maxsize, -sys.maxsize]
    for point in points:
        v = (pycompgeo.vec3.dot(axis[0], point), pycompgeo.vec3.dot(axis[1], point), pycompgeo.vec3.dot(axis[2], point))
        min_[0] = min(v[0], min_[0])
        min_[1] = min(v[1], min_[1])
        min_[2] = min(v[2], min_[2])
        max_[0] = max(v[0], max_[0])
        max_[1] = max(v[1], max_[1])
        max_[2] = max(v[2], max_[2])
        
    return OBB(axis, min_, max_)

class ConvexPolyhedra:
    def __init__(self, vertices=[], edges=[], faces=[], faceNormals=[], centroid=None):
        self.vertices = vertices[::-1]
        self.edges = edges[::-1]
        self.faces = faces[::-1]
        self.faceNormals = faceNormals[::-1]
        self.centroid = centroid
        
def createPrism(size, model, cap):
    prism = ConvexPolyhedra()
    halfHeight = size[1] * 0.5
    scale = pycompgeo.mat2.scale(pycompgeo.vec2.xz(size))
    numPoints = len(cap)
    prism.vertices = [None] * 2 * numPoints
    prism.edges = [None] * 3 * numPoints
    prism.faces = [None] * (numPoints + 2)
    prism.faceNormals = [None] * len(prism.faces)
    i = 0
    j = numPoints
    k = 2 * numPoints
    l = numPoints - 1
    m = k - 1
    while i < len(cap):
        v0 = cap[i]
        v1 = cap[l]
        sv0 = pycompgeo.mat2.mul(scale, v0)
        prism.vertices[i] = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (sv0[0], halfHeight, sv0[1], 1.0)))
        prism.vertices[j] = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (sv0[0], -halfHeight, sv0[1], 1.0)))
        prism.edges[i] = ( l, i )
        prism.edges[j] = ( m, j )
        prism.edges[k] = ( i, j )
        prism.faces[i] = i
        prism.faceNormals[i] = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (v0[1] - v1[1], 0, -(v0[0] - v1[0]), 0)))
        l = i
        m = j
        i = i + 1
        j = j + 1
        k = k + 1
    prism.faces[len(cap)] = 0
    prism.faceNormals[len(cap)] = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (0, 1, 0, 0)))
    prism.faces[numPoints + 1] = numPoints
    prism.faceNormals[numPoints + 1] = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (0, -1, 0, 0)))
    prism.centroid = (model[3], model[7], model[11])
    return prism

def createCube(size, model):
	halfExtents = pycompgeo.vec3.scale(0.5, size)
	v0 = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (halfExtents[0], -halfExtents[1], halfExtents[2], 1.0)))
	v1 = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (-halfExtents[0], -halfExtents[1], halfExtents[2], 1.0)))
	v2 = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (halfExtents[0], halfExtents[1], halfExtents[2], 1.0)))
	v3 = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (-halfExtents[0], halfExtents[1], halfExtents[2], 1.0)))
	v4 = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (halfExtents[0], -halfExtents[1], -halfExtents[2], 1.0)))
	v5 = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (-halfExtents[0], -halfExtents[1], -halfExtents[2], 1.0)))
	v6 = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (halfExtents[0], halfExtents[1], -halfExtents[2], 1.0)))
	v7 = pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (-halfExtents[0], halfExtents[1], -halfExtents[2], 1.0)))
	box = ConvexPolyhedra(
		[
			v0, v1, v2, v3,
			v4, v5, v6, v7
		],
		[
			( 0, 1 ),
			( 0, 2 ),
			( 0, 4 ),
			( 1, 3 ),
			( 1, 5 ),
			( 2, 3 ),
			( 2, 6 ),
			( 4, 5 ),
			( 4, 6 ),
			( 3, 7 ),
			( 5, 7 ),
			( 6, 7 )
		],
		[
			0,#front
			0,#right
			0,#bottom
			1,#left
			2,#top
			4 #back
		],
		[
			pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (0, 0, 1, 0))),
			pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (1, 0, 0, 0))),
			pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (0, -1, 0, 0))),
			pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (-1, 0, 0, 0))),
			pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (0, 1, 0, 0))),
			pycompgeo.vec3.xyz(pycompgeo.mat4.mul(model, (0, 0, -1, 0)))
		],
        (model[3], model[7], model[11]))
	return box

#################################################################################
# source: http://www.geometrictools.com/Documentation/MethodOfSeparatingAxes.pdf
#################################################################################
    
def whichSide(V, D, P):
    positive = 0
    negative = 0
    for i in range(0, len(V)):
        t = pycompgeo.vec3.dot(D, pycompgeo.vec3.sub(V[i], P))
        if t > 0: 
            positive = positive + 1
        elif t < 0: 
            negative = negative + 1
        if positive and negative:
            return 0
    return 1 if positive else -1

def sat(a, b):
    for i in range(0, len(a.faces)):
        if whichSide(b.vertices, a.faceNormals[i], a.vertices[a.faces[i]]) > 0:
            return False
    for i in range(0, len(b.faces)):
        if whichSide(a.vertices, b.faceNormals[i], b.vertices[b.faces[i]]) > 0:
            return False
    for i in range(0, len(a.edges)):
        e0 = a.edges[i]
        for j in range(0, len(b.edges)):
            e1 = b.edges[j]
            D = pycompgeo.vec3.cross(pycompgeo.vec3.sub(a.vertices[e0[1]], a.vertices[e0[0]]), pycompgeo.vec3.sub(b.vertices[e1[1]], b.vertices[e1[0]]))
            b0 = whichSide(a.vertices, D, a.vertices[e0[0]])
            if b0 == 0:
                continue
            b1 = whichSide(b.vertices, D, a.vertices[e0[0]])
            if b1 == 0:
                continue
            if b0 * b1 < 0:
                return False
    return True
