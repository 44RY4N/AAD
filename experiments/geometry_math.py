"""Offline geometry helpers. Matrices follow Panda3D row-vector convention."""
import math


def transform_point(point, matrix):
    p = list(point) + [1.0]
    result = [sum(p[i] * matrix[i][j] for i in range(4)) for j in range(4)]
    if abs(result[3]) < 1e-12:
        raise ValueError("Point projects to infinity")
    return [v / result[3] for v in result[:3]]


def box_vertices(half_extents):
    return [[sx*half_extents[0], sy*half_extents[1], sz*half_extents[2]]
            for sx in [-1, 1] for sy in [-1, 1] for sz in [-1, 1]]


def convex_hull_xy(points):
    points = sorted(set((float(p[0]),float(p[1])) for p in points))
    if len(points) < 3:
        return [list(p) for p in points]
    def cross(o,a,b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    def half(sequence):
        result=[]
        for p in sequence:
            while len(result)>=2 and cross(result[-2],result[-1],p)<=0:
                result.pop()
            result.append(p)
        return result
    return [list(p) for p in half(points)[:-1]+half(reversed(points))[:-1]]


def polygon_clearance_for_tests(a,b):
    """Synthetic convex-polygon distance helper, not an exported safety metric."""
    def separated(axis):
        pa=[sum(x*y for x,y in zip(p,axis)) for p in a]
        pb=[sum(x*y for x,y in zip(p,axis)) for p in b]
        return max(pa)<min(pb) or max(pb)<min(pa)
    axes=[[-(q[1]-p[1]),q[0]-p[0]] for poly in [a,b] for p,q in zip(poly,poly[1:]+poly[:1])]
    if not any(separated(axis) for axis in axes):
        return 0.0
    def distance(p,v,w):
        delta=[w[i]-v[i] for i in range(2)]
        t=max(0,min(1,sum((p[i]-v[i])*delta[i] for i in range(2))/sum(x*x for x in delta)))
        return math.hypot(*(p[i]-v[i]-t*delta[i] for i in range(2)))
    return min(distance(p,v,w) for first,second in [(a,b),(b,a)] for p in first for v,w in zip(second,second[1:]+second[:1]))
