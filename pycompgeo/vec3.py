import math

def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def scale(f, v):
    return (f * v[0], f * v[1], f * v[2])

def length(v):
    return math.sqrt(dot(v, v))

def xyz(v):
    return (v[0], v[1], v[2])

def normalize(v):
    m = length(v)
    try:
        return scale(1.0 / m, v)
    except ValueError as e:
        return (0, 0, 0)
    except ZeroDivisionError as e:
        return (0, 0, 0)

def distance(a, b):
    x = a[0] - b[0]
    y = a[1] - b[1]
    z = a[2] - b[2]
    return math.sqrt(x*x + y*y + z*z)

def angle(a, b):
    m = (length(a) * length(b))
    try: 
        return math.acos(dot(a, b) / m)
    except ValueError as e:
        return 0
    except ZeroDivisionError as e:
        return 0

def orientedAngle(a, b):
    c = angle(a, b)
    if cross(a, b)[2] >= 0:
        return c
    else:
        return -c

def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], 
            a[2] * b[0] - a[0] * b[2], 
            a[0] * b[1] - a[1] * b[0])

