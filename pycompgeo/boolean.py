import pycompgeo.utils
import pycompgeo.collision

def triangle3DTrim(triangle, line):
        t0 = (triangle[0][0], triangle[0][2])
        t0_h = triangle[0][1]
        t1 = (triangle[1][0], triangle[1][2])
        t1_h = triangle[1][1]
        t2 = (triangle[2][0], triangle[2][2])
        t2_h = triangle[2][1]

        p0 = (line[0][0], line[0][1])
        p0_n = line[0][2]
        p1 = (line[1][0], line[1][1])
        p1_n = line[1][2]

        p0_L1, p0_L2, p0_L3 = pycompgeo.utils.barycentricCoordinates(triangle, p0)
        p1_L1, p1_L2, p1_L3 = pycompgeo.utils.barycentricCoordinates(triangle, p1)

        p0_i = pycompgeo.utils.barycentricPointInTriangle(p0_L1, p0_L2, p0_L3)
        p1_i = pycompgeo.utils.barycentricPointInTriangle(p1_L1, p1_L2, p1_L3)

        if p0_i and p1_i: # both line points lay inside the triangle
            return ( 
                    (p0[0], pycompgeo.utils.barycentricInterpolation1D2(triangle, p0_L1, p0_L2, p0_L3, 1), p0[1], p0_n), 
                    (p1[0], pycompgeo.utils.barycentricInterpolation1D2(triangle, p1_L1, p1_L2, p1_L3, 1), p1[1], p1_n)
                   )
        if p0_i ^ p1_i: # only one line point lays inside the triangle, so there's only 1 possible intersection
            if p0_i:
                a = p0
                h0 = pycompgeo.utils.barycentricInterpolation1D2(triangle, p0_L1, p0_L2, p0_L3, 1)
                n0 = p0_n
                L1 = p1_L1
                L2 = p1_L2
                L3 = p1_L3
            else:
                a = p1
                h0 = pycompgeo.utils.barycentricInterpolation1D2(triangle, p1_L1, p1_L2, p1_L3, 1)
                n0 = p1_n
                L1 = p0_L1
                L2 = p0_L2
                L3 = p0_L3

            b = None

            # check intersection only against the edges to which the outside point is opposite

            if L1 < -pycompgeo.utils.EPSILON:
                b, u, v = pycompgeo.collision.lineSegmentIntersection(t1, t2, p0, p1)
                h1 = pycompgeo.utils.lerp(u, t1_h, t2_h)
                n1 = pycompgeo.utils.lerp(v, p0_n, p1_n)

            if b == None and L2 < -pycompgeo.utils.EPSILON:
                b, u, v = pycompgeo.collision.lineSegmentIntersection(t2, t0, p0, p1)
                h1 = pycompgeo.utils.lerp(u, t2_h, t0_h)
                n1 = pycompgeo.utils.lerp(v, p0_n, p1_n)
            
            if b == None and L3 < -pycompgeo.utils.EPSILON:
                b, u, v = pycompgeo.collision.lineSegmentIntersection(t0, t1, p0, p1)
                h1 = pycompgeo.utils.lerp(u, t0_h, t1_h)
                n1 = pycompgeo.utils.lerp(v, p0_n, p1_n)

            # FIXME: checking invariants
            if b == None:
                raise Exception('b == None')

            return (
                    (a[0], h0, a[1], n0), 
                    (b[0], h1, b[1], n1)
                   )

        else: # neither line points lay inside the triangle
            ab = []

            if (p0_L1 >= -pycompgeo.utils.EPSILON and p1_L1 < -pycompgeo.utils.EPSILON) or (p1_L1 >= -pycompgeo.utils.EPSILON and p0_L1 < -pycompgeo.utils.EPSILON):
                c, u, v = pycompgeo.collision.lineSegmentIntersection(t1, t2, p0, p1)
                if c != None:
                    ab.append(
                        (c[0], pycompgeo.utils.lerp(u, t1_h, t2_h), c[1], pycompgeo.utils.lerp(v, p0_n, p1_n))
                    )

            if (p0_L2 >= -pycompgeo.utils.EPSILON and p1_L2 < -pycompgeo.utils.EPSILON) or (p1_L2 >= -pycompgeo.utils.EPSILON and p0_L2 < -pycompgeo.utils.EPSILON):
                c, u, v = pycompgeo.collision.lineSegmentIntersection(t2, t0, p0, p1)
                if c != None:
                    ab.append(
                        (c[0], pycompgeo.utils.lerp(u, t2_h, t0_h), c[1], pycompgeo.utils.lerp(v, p0_n, p1_n))
                    )

            if (p0_L3 >= -pycompgeo.utils.EPSILON and p1_L3 < -pycompgeo.utils.EPSILON) or (p1_L3 >= -pycompgeo.utils.EPSILON and p0_L3 < -pycompgeo.utils.EPSILON):
                c, u, v = pycompgeo.collision.lineSegmentIntersection(t0, t1, p0, p1)
                if c != None:
                    ab.append(
                        (c[0], pycompgeo.utils.lerp(u, t0_h, t1_h), c[1], pycompgeo.utils.lerp(v, p0_n, p1_n))
                    )

            if len(ab) == 0:
                return None
            elif len(ab) == 2:
                return (ab[0], ab[1])
            else:
                # FIXME: checking invariants
                raise Exception('len(triangle) != 0 and len(triangle) != 2')
    
def triangle3DBoolean2D(triangle, p0, p1, p2, uv0 = (0, 0), uv1 = (0, 0), uv2 = (0, 0), sort = True):
        t_p = (
                (triangle[0][0], triangle[0][2]), 
                (triangle[1][0], triangle[1][2]), 
                (triangle[2][0], triangle[2][2])
             )
        p = (p0, p1, p2)
        u = (uv0, uv1, uv2)
        p_L = (
                pycompgeo.utils.barycentricCoordinates(triangle, p0), 
                pycompgeo.utils.barycentricCoordinates(triangle, p1), 
                pycompgeo.utils.barycentricCoordinates(triangle, p2)
              )

        p_i = (
                pycompgeo.utils.barycentricPointInTriangle2(p_L[0]), 
                pycompgeo.utils.barycentricPointInTriangle2(p_L[1]), 
                pycompgeo.utils.barycentricPointInTriangle2(p_L[2])
              )
        i_p = p_i[0] + p_i[1] + p_i[2]

        points = []
        uvs = []

        if i_p == 3: # all points inside
            return [
                        (p0[0], pycompgeo.utils.barycentricInterpolation1D(triangle, p_L[0], 1), p0[1]), 
                        (p1[0], pycompgeo.utils.barycentricInterpolation1D(triangle, p_L[1], 1), p1[1]),  
                        (p2[0], pycompgeo.utils.barycentricInterpolation1D(triangle, p_L[2], 1), p2[1]), 
                   ], [
                       uv0,
                       uv1,
                       uv2
                   ]
        elif i_p == 2: # 2 pts inside
            # find the index of the outside point
            i = 0
            while i < 3:
                if not p_i[i]:
                    break
                i += 1

            # indices of the inside points
            j = (i + 1) % 3
            k = (i + 2) % 3

            # add the inside points to the output polygon
            points = [
                    (p[j][0], pycompgeo.utils.barycentricInterpolation1D(triangle, p_L[j], 1), p[j][1]),
                    (p[k][0], pycompgeo.utils.barycentricInterpolation1D(triangle, p_L[k], 1), p[k][1])
                  ]
            uvs = [
                    u[j],
                    u[k]
                ]

            op_of = (
                        p_L[i][0] < -pycompgeo.utils.EPSILON, 
                        p_L[i][1] < -pycompgeo.utils.EPSILON, 
                        p_L[i][2] < -pycompgeo.utils.EPSILON
                    )

            # counting the num. of opposing edges for the outside point
            o_f = op_of[0] + op_of[1] + op_of[2]

            f = 0
            while f < 3:
                if op_of[f]:
                    d = pycompgeo.utils.approx1D(p_L[j][f], 0.0)
                    e = pycompgeo.utils.approx1D(p_L[k][f], 0.0)
                    if o_f == 1 and d and e:
                        # if the outside point has 1 opposing edge and it is the same edge where the other 2 inside points are *on*
                        # p0p1p2 and triangle are completely disjoint
                        return [], []

                    # compute the indices of the vertices of the opposing edge
                    g = (f + 1) % 3
                    h = (f + 2) % 3

                    # check intersection between the opposing edge and the lines from the outside point to the inside points
                    if not d:
                        i_0, u_0, u_1 = pycompgeo.collision.lineSegmentIntersection(t_p[g], t_p[h], p[i], p[j])
                        if i_0 != None:
                            points.append((i_0[0], pycompgeo.utils.lerp(u_0, triangle[g][1], triangle[h][1]), i_0[1]))
                            uvs.append(pycompgeo.utils.lerp2D(u_1, u[i], u[j]))

                    if not e:
                        i_1, u_0, u_1 = pycompgeo.collision.lineSegmentIntersection(t_p[g], t_p[h], p[i], p[k])
                        if i_1 != None:
                            points.append((i_1[0], pycompgeo.utils.lerp(u_0, triangle[g][1], triangle[h][1]), i_1[1]))
                            uvs.append(pycompgeo.utils.lerp2D(u_1, u[i], u[k]))

                f += 1

            # if the number of opposing edges is 2, the junction point between both opposing edges
            # may lay inside p0p1p2

            p0p1p2 = (
                        (p0[0], 0.0, p0[1]), 
                        (p1[0], 0.0, p1[1]), 
                        (p2[0], 0.0, p2[1])
                     )
            
            uv0uv1uv2 = (
                    uv0,
                    uv1,
                    uv2
                )

            if o_f == 2:
                f = 0
                r = False
                while f < 3: 
                    nf = (f + 1) % 3
                    if op_of[f] and op_of[nf]:
                        q = triangle[(f + 2) % 3]
                        q_L = pycompgeo.utils.barycentricCoordinates(p0p1p2, (q[0], q[2]))
                        if pycompgeo.utils.barycentricPointInTriangle2(q_L):
                            points.append(q)
                            uvs.append(pycompgeo.utils.barycentricInterpolation2D(uv0uv1uv2, q_L))
                        r = True
                        break
                    f += 1

                # FIXME: checking invariants
                if not r:
                    raise Exception('r')

        elif i_p == 1: # 1 pt inside
            # find the index of the inside point
            i = 0
            while i < 3:
                if p_i[i]:
                    break
                i += 1

            # compute the indices of the outside points
            o_p = [(i + 1) % 3, (i + 2) % 3]

            # add the inside point to the output polygon
            points = [(p[i][0], pycompgeo.utils.barycentricInterpolation1D(triangle, p_L[i], 1), p[i][1])]
            uvs = [u[i]]

            op_of = (
                        (p_L[o_p[0]][0] < -pycompgeo.utils.EPSILON, p_L[o_p[0]][1] < -pycompgeo.utils.EPSILON, p_L[o_p[0]][2] < -pycompgeo.utils.EPSILON),
                        (p_L[o_p[1]][0] < -pycompgeo.utils.EPSILON, p_L[o_p[1]][1] < -pycompgeo.utils.EPSILON, p_L[o_p[1]][2] < -pycompgeo.utils.EPSILON),
                    )

            # creates masks of opposing edges for the outside points
            o_f = (
                        op_of[0][0] + (op_of[0][1] << 1) + (op_of[0][2] << 2),
                        op_of[1][0] + (op_of[1][1] << 1) + (op_of[1][2] << 2)
                  )

            # both outside points have only 1 opposing edge and it's the same edge
            if (o_f[0] == 1 or o_f[0] == 2 or o_f[0] == 4) and (o_f[1] == 1 or o_f[1] == 2 or o_f[1] == 4) and o_f[0] == o_f[1]:
                f = o_f[0] >> 1

                # both outside points are opposing the edge where the inside point is on,
                # so the insersection would be a single point (the inside point)
                if pycompgeo.utils.approx1D(p_L[i][f], 0.0):
                    return [], []

                # indices of the vertices of that opposing edge
                g = (f + 1) % 3
                h = (f + 2) % 3

                # check intersection between the opposing edge and the lines from the outside point to the inside points

                i_0, u_0, u_1 = pycompgeo.collision.lineSegmentIntersection(t_p[g], t_p[h], p[i], p[o_p[0]])
                # FIXME: checking invariants
                if i_0 == None:
                    raise Exception('i_0 == None')
                points.append((i_0[0], pycompgeo.utils.lerp(u_0, triangle[g][1], triangle[h][1]), i_0[1]))
                uvs.append(pycompgeo.utils.lerp2D(u_1, u[i], u[o_p[0]]))

                i_0, u_0, u_1 = pycompgeo.collision.lineSegmentIntersection(t_p[g], t_p[h], p[i], p[o_p[1]])
                # FIXME: checking invariants
                if i_0 == None:
                    raise Exception('i_0 == None')
                points.append((i_0[0], pycompgeo.utils.lerp(u_0, triangle[g][1], triangle[h][1]), i_0[1]))
                uvs.append(pycompgeo.utils.lerp2D(u_1, u[i], u[o_p[1]]))

            # outside points have more than 1 opposing edge or their single opposing edges are different
            else:
                # iterate over the 2 outside points
                line = 0
                while line < 2:
                    m0 = o_f[line]
                    j0 = o_p[line]

                    # iterate over the 3 bits mask
                    m1 = 1
                    while m1 < 8:
                        # if current edge is an opposing edge check intersection between 
                        # current edge and line from the inside point to the current outside point
                        if m0 & m1:
                            f = m1 >> 1

                            g = (f + 1) % 3
                            h = (f + 2) % 3

                            i_0, u_0, u_1 = pycompgeo.collision.lineSegmentIntersection(t_p[g], t_p[h], p[i], p[j0])
                            if i_0 != None:
                                points.append((i_0[0], pycompgeo.utils.lerp(u_0, triangle[g][1], triangle[h][1]), i_0[1]))
                                uvs.append(pycompgeo.utils.lerp2D(u_1, u[i], u[j0]))

                        m1 = m1 << 1
                    line += 1

                # creates a mask with edges that are opposing outside point 1 and NOT opposing outside point 2 (and vice versa)
                # because they are edges where intersection *might* occur
                m0 = (o_f[0] ^ o_f[1])

                # iterate over the 3 bits mask
                m1 = 1
                while m1 < 8:
                    # if current edge is an opposing edge, check intersection between 
                    # this edge and a line from one outside point to the other
                    if m0 & m1:
                        f = m1 >> 1

                        g = (f + 1) % 3
                        h = (f + 2) % 3

                        i_0, u_0, u_1 = pycompgeo.collision.lineSegmentIntersection(t_p[g], t_p[h], p[o_p[0]], p[o_p[1]])
                        if i_0 != None:
                            points.append((i_0[0], pycompgeo.utils.lerp(u_0, triangle[g][1], triangle[h][1]), i_0[1]))
                            uvs.append(pycompgeo.utils.lerp2D(u_1, u[o_p[0]], u[o_p[1]]))

                    m1 = m1 << 1

                # FIXME: improve
                p0p1p2 = (
                          (p0[0], 0.0, p0[1]), 
                          (p1[0], 0.0, p1[1]), 
                          (p2[0], 0.0, p2[1])
                         )

                uv0uv1uv2 = (
                    uv0,
                    uv1,
                    uv2
                )

                for q in triangle:
                    q_L = pycompgeo.utils.barycentricCoordinates(p0p1p2, (q[0], q[2]))
                    if pycompgeo.utils.barycentricPointInTriangle2(q_L):
                        points.append(q)
                        uvs.append(pycompgeo.utils.barycentricInterpolation2D(uv0uv1uv2, q_L))

        else: # no inside pts
            op_of = (
                        (p_L[0][0] < -pycompgeo.utils.EPSILON, p_L[0][1] < -pycompgeo.utils.EPSILON, p_L[0][2] < -pycompgeo.utils.EPSILON),
                        (p_L[1][0] < -pycompgeo.utils.EPSILON, p_L[1][1] < -pycompgeo.utils.EPSILON, p_L[1][2] < -pycompgeo.utils.EPSILON),
                        (p_L[2][0] < -pycompgeo.utils.EPSILON, p_L[2][1] < -pycompgeo.utils.EPSILON, p_L[2][2] < -pycompgeo.utils.EPSILON),
                    )

            # creates masks of opposing edges for the outside points
            o_f = (
                        op_of[0][0] + (op_of[0][1] << 1) + (op_of[0][2] << 2),
                        op_of[1][0] + (op_of[1][1] << 1) + (op_of[1][2] << 2),
                        op_of[2][0] + (op_of[2][1] << 1) + (op_of[2][2] << 2)
                  )

            # if all points have only 1 opposing edge and it's the same for all of them,
            # triangle and p0p1p2 are completely disjoint
            if (o_f[0] == 1 or o_f[0] == 2 or o_f[0] == 4) and (o_f[1] == 1 or o_f[1] == 2 or o_f[1] == 4) and (o_f[2] == 1 or o_f[2] == 2 or o_f[2] == 4) and o_f[0] == o_f[1] == o_f[2]:
                return [], []

            # iterates over all outside points
            i = 0
            while i < 3:
                j = (i + 1) % 3

                # creates a mask with edges that are opposing i and NOT opposing j (and vice versa)
                # because they are edges where intersection might occur
                m0 = (o_f[i] ^ o_f[j])

                # iterates over the 3 bits mask
                m1 = 1
                while m1 < 8:
                    # check intersection only if current edge is opposed by i and NOT opposed by j (or vice versa)
                    if m0 & m1:
                        f = m1 >> 1

                        g = (f + 1) % 3
                        h = (f + 2) % 3

                        i_0, u_0, u_1 = pycompgeo.collision.lineSegmentIntersection(t_p[g], t_p[h], p[i], p[j])
                        if i_0 != None:
                            points.append((i_0[0], pycompgeo.utils.lerp(u_0, triangle[g][1], triangle[h][1]), i_0[1]))
                            uvs.append(pycompgeo.utils.lerp2D(u_1, u[i], u[j]))

                    m1 = m1 << 1
                i += 1

            # FIXME: improve
            p0p1p2 = (
                          (p0[0], 0.0, p0[1]), 
                          (p1[0], 0.0, p1[1]), 
                          (p2[0], 0.0, p2[1])
                     )

            uv0uv1uv2 = (
                    uv0,
                    uv1,
                    uv2
                )

            for q in triangle:
                q_L = pycompgeo.utils.barycentricCoordinates(p0p1p2, (q[0], q[2]))
                if pycompgeo.utils.barycentricPointInTriangle2(q_L):
                    points.append(q)
                    uvs.append(pycompgeo.utils.barycentricInterpolation2D(uv0uv1uv2, q_L))

        if len(points) > 0 and sort:
            order = pycompgeo.utils.orderCW(points, (0, 2))
            points = [points[i] for i in order]
            uvs = [uvs[i] for i in order]

        return points, uvs