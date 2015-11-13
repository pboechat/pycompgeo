import pycompgeo.vec2

def grahamScan(input, angleThreshold = 0):
    a = list(input)
    # choose the left bottom most point as anchor
    anchor = a[0]
    i = 0
    for j, p in enumerate(a[1:]):
        if p[0] < anchor[0] or (p[0] == anchor[0] and p[1] < anchor[1]):
            anchor = p
            i = j + 1
    # sort in CCW order
    order = sorted(range(0, len(a)), key = lambda k: -1.0 if k == i else pycompgeo.vec2.angle((0.0, -1.0), pycompgeo.vec2.sub(a[k], anchor)))
    order = order[1:]
    order.insert(0, i)
    points = [a[i] for i in order]

    i = 2
    while i < len(points):
        a = points[i - 2]
        b = points[i - 1]
        c = points[i]
        e0 = pycompgeo.vec2.sub(b, a)
        e1 = pycompgeo.vec2.sub(c, b)
        cross = pycompgeo.vec2.cross(e0, e1)
        angle = pycompgeo.vec2.rawAngle(e0, e1)
        while cross < 0 or angle < angleThreshold: # NOTE: right turn or angle < angle threshold
            points.remove(b)
            i -= 1
            b = a
            a = points[i - 2]
            e0 = pycompgeo.vec2.sub(b, a)
            e1 = pycompgeo.vec2.sub(c, b)
            cross = pycompgeo.vec2.cross(e0, e1)
            angle = pycompgeo.vec2.rawAngle(e0, e1)
        i += 1
    
    return points

