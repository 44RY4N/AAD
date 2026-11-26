"""Convex polygon separation and continuous translated SAT; union area via GEOS."""
import math
from shapely.geometry import Polygon
from shapely.ops import unary_union

EPS=1e-9  # numerical distance tolerance in metres, never a safety threshold
AREA_EPS=1e-12


def polygon(vertices):
    if not isinstance(vertices,(list,tuple)):
        raise ValueError("missing_polygon")
    points=[]
    for p in vertices:
        if not isinstance(p,(list,tuple)) or len(p)!=2 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in p):
            raise ValueError("nonfinite_or_malformed_polygon")
        p=tuple(float(v) for v in p)
        if not points or points[-1]!=p:points.append(p)
    if len(points)>1 and points[-1]==points[0]:points.pop()
    if len(points)<3 or len(set(points))!=len(points):raise ValueError("degenerate_or_repeated_nonadjacent_vertices")
    poly=Polygon(points)
    if not poly.is_valid or poly.area<=AREA_EPS:raise ValueError("nonsimple_or_zero_area_polygon")
    if not poly.equals(poly.convex_hull):raise ValueError("nonconvex_polygon")
    # Drop exactly collinear samples; no positive simplification tolerance.
    points=list(poly.convex_hull.exterior.coords)[:-1]
    if sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))<0:points.reverse()
    first=min(range(len(points)),key=lambda i:points[i]);points=points[first:]+points[:first]
    return [list(p) for p in points]


def axes(points):
    for p,q in zip(points,points[1:]+points[:1]):
        dx,dy=q[0]-p[0],q[1]-p[1];length=math.hypot(dx,dy)
        if length>0:yield [-dy/length,dx/length]


def interval(points,axis):
    values=[p[0]*axis[0]+p[1]*axis[1] for p in points]
    return min(values),max(values)


def point_segment(point,a,b):
    d=[b[i]-a[i] for i in range(2)]
    t=max(0.,min(1.,sum((point[i]-a[i])*d[i] for i in range(2))/sum(v*v for v in d)))
    q=[a[i]+t*d[i] for i in range(2)]
    return math.dist(point,q),q


def separation(a,b):
    a,b=polygon(a),polygon(b)
    candidates=[]
    for i,p in enumerate(a):
        for j,(v,w) in enumerate(zip(b,b[1:]+b[:1])):
            distance,q=point_segment(p,v,w);candidates.append((distance,p,q,[i,j]))
    for j,p in enumerate(b):
        for i,(v,w) in enumerate(zip(a,a[1:]+a[:1])):
            distance,q=point_segment(p,v,w);candidates.append((distance,q,p,[i,j]))
    boundary=min(candidates,key=lambda r:r[0])
    gap=max(max(interval(a,n)[0]-interval(b,n)[1],interval(b,n)[0]-interval(a,n)[1]) for n in list(axes(a))+list(axes(b)))
    if gap>EPS:
        return dict(distance=boundary[0],relation="separated",closest_points=[boundary[1],boundary[2]],
                    boundary_indices=boundary[3],closest_points_scope="boundary_points",penetration_depth=None)
    shared=Polygon(a).intersection(Polygon(b))
    relation="overlapping" if shared.area>AREA_EPS else "touching"
    if shared.is_empty:points=[boundary[1],boundary[2]]
    else:
        p=shared.representative_point();points=[[p.x,p.y],[p.x,p.y]]
    return dict(distance=0.,relation=relation,closest_points=points,boundary_indices=None,
                closest_points_scope="common_point_or_numerical_touch_not_penetration",penetration_depth=None)


def actor_separation(a_components,b_components):
    if not a_components or not b_components:raise ValueError("incomplete_component_coverage")
    pairs=[]
    for ai,a in a_components.items():
        for bi,b in b_components.items():
            pairs.append(dict(component_ids=[ai,bi],**separation(a,b)))
    best=min(pairs,key=lambda p:(p["distance"],{"overlapping":0,"touching":1,"separated":2}[p["relation"]],p["component_ids"]))
    return dict(best,component_pairs=pairs)


def union_region(components):
    if not components:raise ValueError("incomplete_component_coverage")
    return unary_union([Polygon(polygon(p)) for p in components.values()])


def occupancy(components,lane_polygon):
    region=union_region(components);lane=Polygon(polygon(lane_polygon))
    area=region.area;intersection=region.intersection(lane).area
    fraction=max(0.,min(1.,intersection/area))
    state="outside" if intersection<=AREA_EPS else "whole" if abs(area-intersection)<=max(AREA_EPS,area*1e-9) else "partial"
    return dict(actor_area=area,intersection_area=intersection,fraction=fraction,state=state,
                area_method="GEOS_union_of_components_intersect_convex_lane_no_convex_union_fill")


def swept_ttc(a,b,velocity_a,velocity_b,horizon):
    a,b=polygon(a),polygon(b)
    if not isinstance(horizon,(int,float)) or not math.isfinite(horizon) or horizon<=0:raise ValueError("invalid_horizon")
    for v in [velocity_a,velocity_b]:
        if not isinstance(v,(list,tuple)) or len(v)!=2 or any(not isinstance(x,(int,float)) or isinstance(x,bool) or not math.isfinite(x) for x in v):
            return dict(value=None,validity="unavailable",reason="missing_or_invalid_velocity",entry_s=None,exit_s=None)
    current=separation(a,b)["relation"]
    if current!="separated":
        return dict(value=0.,validity="valid",reason="already_"+current+"_envelopes",entry_s=0.,exit_s=None)
    relative=[velocity_b[i]-velocity_a[i] for i in range(2)]
    enter,leave=-math.inf,math.inf
    for n in list(axes(a))+list(axes(b)):
        la,ua=interval(a,n);lb,ub=interval(b,n);speed=sum(x*y for x,y in zip(relative,n))
        if abs(speed)<=1e-12:
            if ua<lb-EPS or ub<la-EPS:
                return dict(value=None,validity="undefined",reason="no_future_intersection_on_parallel_axis",entry_s=None,exit_s=None)
            continue
        low,high=sorted([(la-ub)/speed,(ua-lb)/speed]);enter=max(enter,low);leave=min(leave,high)
        if enter>leave+1e-12:
            return dict(value=None,validity="undefined",reason="no_common_future_axis_interval",entry_s=None,exit_s=None)
    if leave<0:
        return dict(value=None,validity="undefined",reason="separating_motion_intersection_only_in_past",entry_s=None,exit_s=None)
    start=max(0.,enter)
    if start>horizon:
        return dict(value=None,validity="censored",reason="no_intersection_within_prediction_horizon",entry_s=None,exit_s=None)
    return dict(value=start,validity="valid",reason="predicted_outer_envelope_intersection",entry_s=start,
                exit_s=min(horizon,leave),orientation_held_fixed=True)


def actor_ttc(a,b,va,vb,horizon):
    if not a or not b:return dict(value=None,validity="unavailable",reason="incomplete_component_coverage",component_ids=[])
    results=[dict(swept_ttc(pa,pb,va,vb,horizon),component_ids=[ai,bi]) for ai,pa in a.items() for bi,pb in b.items()]
    valid=[r for r in results if r["validity"]=="valid"]
    if valid:return min(valid,key=lambda r:(r["value"],r["component_ids"]))
    for state in ["unavailable","invalid","censored","undefined"]:
        matches=[r for r in results if r["validity"]==state]
        if matches:return matches[0]
    raise ValueError("Unexpected TTC state")
