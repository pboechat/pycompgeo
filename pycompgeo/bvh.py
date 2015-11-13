import math
import bisect
import pycompgeo.collision

#############################################################################
# source: http://fileadmin.cs.lth.se/cs/Education/EDAN30/lectures/S2-bvh.pdf
#############################################################################
class Node:
    def __init__(self, aabb):
        self.aabb = aabb
        self.leaf = False
        self.numObjects = 0
        # if leaf, index maps to left child node, otherwise index maps to first object in objects vector
        self.index = -1

class BVH:
    def __init__(self):
        self.objects = None
        self.nodes = None
        self.threshold = 0

    def createAABB(self, left, right):
        vertices = []
        for i in range(left, right):
            vertices = vertices + self.objects[self.objectIndices[i]].vertices
        return pycompgeo.collision.createAABB(vertices)

    def aabbQuery(self, aabb, node = None):
        if node == None:
            node = self.nodes[0]
        if pycompgeo.collision.aabbCollision(aabb, node.aabb):
            if node.leaf:
                return [self.objects[i] for i in self.objectIndices[node.index:node.index + node.numObjects]]
            else:
                return self.aabbQuery(aabb, self.nodes[node.index]) + self.aabbQuery(aabb, self.nodes[node.index + 1])
        else:
            return []

    def buildRecursive(self, left, right, nodeIdx = 0, depth = 0):
        node = self.nodes[nodeIdx]
        numObjects = (right - left)
        assert numObjects > 0
        if numObjects <= self.threshold:
            node.leaf = True
            node.numObjects = numObjects
            node.index = left
        else:
            aabb = node.aabb
            largestAxis = aabb.getLargestAxis()
            midPoint = aabb.centroid[largestAxis]
            self.objectIndices[left:right] = sorted(self.objectIndices[left:right], key = lambda i: self.objects[i].centroid[largestAxis])
            axisValues = [self.objects[i].centroid[largestAxis] for i in self.objectIndices[left:right]]
            split = bisect.bisect_right(axisValues, midPoint)
            if split == 0:
                split = left + 1
            elif split == len(axisValues):
                split = right - 1
            else:
                split = split + left
            node.leaf = False
            leftChild = nodeIdx * 2 + 1
            node.index = leftChild
            self.nodes[leftChild] = Node(self.createAABB(left, split))
            self.nodes[leftChild + 1] = Node(self.createAABB(split, right))
            self.buildRecursive(left, split, leftChild, depth + 1)
            self.buildRecursive(split, right, leftChild + 1, depth + 1)


    def build(self, objects, threshold = 4):
        self.objects = objects[::-1]
        self.threshold = threshold
        N = len(objects)
        self.nodes = {}
        self.objectIndices = list(range(0, N))
        aabb = self.createAABB(0, N)
        self.nodes[0] = Node(aabb)
        self.buildRecursive(0, N)
    