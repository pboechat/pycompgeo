import math

def add(a, b):
    return (a[0] + b[0], a[1] + b[1])

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])

def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]

def scale(f, v):
    return (f * v[0], f * v[1])

def length(v):
    return math.sqrt(dot(v, v))

def xy(v):
    return (v[0], v[1])

def xz(v):
    return (v[0], v[2])

def normalize(v):
    m = length(v)
    try:
        return scale(1.0 / m, v)
    except ValueError as e:
        return (0, 0)
    except ZeroDivisionError as e:
        return (0, 0)

def distance(a, b):
    x = a[0] - b[0]
    y = a[1] - b[1]
    return math.sqrt(x*x + y*y)

def rawAngle(a, b):
    m = (length(a) * length(b))
    try:
        return math.acos(dot(a, b) / m)
    except ValueError as e:
        return 0
    except ZeroDivisionError as e:
        return 0

def angle(a, b):
    c = rawAngle(a, b)
    if cross(a, b) >= 0:
        return c
    else:
        return (2.0 * math.pi) - c

def orientedAngle(a, b):
    c = rawAngle(a, b)
    if cross(a, b) >= 0:
        return c
    else:
        return -c

def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]

