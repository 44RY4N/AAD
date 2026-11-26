"""Explicit evidence/validity contracts for projected outer-envelope measurements."""
import math
from shapely.geometry import Polygon

from .inputs import components
from .polygons import AREA_EPS,EPS,actor_separation,actor_ttc,occupancy,polygon,union_region


UNCERTAINTY="Planar projected box outer envelopes; not exact physical surface clearance/contact or real-world risk. Numeric tolerances do not quantify measurement accuracy."


def metric(name,value,unit,actor_ids,step,time,status="valid",reason="",component_ids=None,method="geometry_v02",**extra):
    if status not in ["valid","unavailable","undefined","not_applicable","invalid","censored"]:raise ValueError("Unknown validity")
    if status!="valid":value=None
    if isinstance(value,float) and not math.isfinite(value):raise ValueError("Nonfinite metric")
    result=dict(name=name,value=value,unit=unit,validity=status,reason=reason or "computed_under_declared_geometry_model",
        method=method,actor_ids=list(actor_ids),shape_component_ids=component_ids or [],
        time_support=dict(start_step=step,end_step=step,start_s=time,end_s=time),coordinate_frame="world_xy_m",
        assumptions=["Complete supported chassis-shape coverage", "Projected outer envelopes, not nominal source defaults"],
        uncertainty_note=UNCERTAINTY)
    result.update(extra)
    return result


def prepare_lanes(road):
    result={}
    for lane in road["lanes"]:
        key=tuple(lane["lane_id"]);record=dict(raw=lane,status="unavailable",reason="curved_or_unknown_lane_not_validated")
        if lane["python_class"].endswith(".StraightLane"):
            try:
                p=polygon(lane["polygon"]["xy_m"])
                params=lane["analytic_parameters"];start,end=params["start"],params["end"]
                length=math.dist(start,end);width=float(params["width"])
                if length<=0 or width<=0:raise ValueError("invalid_lane_dimensions")
                tangent=[(end[i]-start[i])/length for i in range(2)];normal=[tangent[1],-tangent[0]]
                corners=[[point[i]+sign*width/2*normal[i] for i in range(2)] for point,sign in [(start,1),(end,1),(end,-1),(start,-1)]]
                if Polygon(p).hausdorff_distance(Polygon(corners))>1e-4:raise ValueError("straight_lane_polygon_inconsistent_with_runtime_parameters")
                record.update(status="valid",reason="validated_straight_convex_lane_representation",polygon=p,tangent=tangent)
            except (ValueError,TypeError,KeyError) as exc:record["reason"]=str(exc)
        if key in result:raise ValueError("Duplicate lane ID")
        result[key]=record
    return result


def state_reason(run,actor,comp,reason):
    row=run["rows"][actor["step"],actor["actor_id"]] if "step" in actor else None
    if reason:return "unavailable",reason
    if row and (not row["on_lane"] or not row["same_road"]):return "invalid","localization_or_road_state_invalid"
    if not comp:return "unavailable","incomplete_component_coverage"
    return "valid",""


def actor_record(run,actor,frame,lanes):
    step,time=frame["step"],frame["time_s"];identity=actor["actor_id"]
    comp,why=components(run,actor);row=run["rows"][step,identity]
    status,reason=("unavailable",why) if why else ("invalid","localization_or_road_state_invalid") if not row["on_lane"] or not row["same_road"] else ("valid","")
    ids=[[identity,i] for i in comp] if comp else []
    area=union_region(comp).area if status=="valid" else None
    area_metric=metric("projected_outer_envelope_union_area",area,"m^2",[identity],step,time,status,reason,ids,"GEOS_component_union_area")
    velocities={name:metric(name,actor["velocity_world_m_s"][i],"m/s",[identity],step,time,
        method="direct_Bullet_runtime_world_velocity",component_ids=[],
        assumptions=["Direct runtime chassis velocity, not finite-difference estimation"]) for i,name in enumerate(["vx_world","vy_world","vz_world"])}
    record=dict(run_id=run["manifest"]["run_id"],step=step,time_s=time,actor_id=identity,role=actor["role"],
        geometry_coverage=status,geometry_reason=reason,metrics=[area_metric]+list(velocities.values()),risk_assessment="unassessed")
    occupancy_data={};lane_records=[]
    relevant={key:l for key,l in lanes.items() if key[:2]==row["lane"][:2]}
    for key,lane in sorted(relevant.items()):
        st,rs=status,reason
        if st=="valid" and lane["status"]!="valid":st,rs="unavailable",lane["reason"]
        data=occupancy(comp,lane["polygon"]) if st=="valid" else None
        occupancy_data[key]=data
        lane_records.append(dict(lane_id=list(key),state=data["state"] if data else "unavailable",metrics=[
            metric("outer_envelope_lane_intersection_area",data["intersection_area"] if data else None,"m^2",[identity],step,time,st,rs,ids,"GEOS_component_union_lane_intersection"),
            metric("outer_envelope_lane_occupancy_fraction",data["fraction"] if data else None,"fraction",[identity],step,time,st,rs,ids,"GEOS_component_union_lane_intersection")]))
    occ_record=dict(run_id=run["manifest"]["run_id"],step=step,time_s=time,actor_id=identity,role=actor["role"],
        lane_scope="All exported lanes on actor's currently assigned directed road; no assigned-ID substitution for overlap",
        scope_validity="valid" if relevant else "unavailable",scope_reason="" if relevant else "no_exported_lanes_on_assigned_road",
        footprint_area=area_metric,lanes=lane_records,risk_assessment="unassessed")
    return record,occ_record,dict(components=comp,status=status,reason=reason,occupancy=occupancy_data,row=row)


def context(a,b,ga,gb):
    for g in [ga,gb]:
        if g["status"]!="valid":return g["status"],g["reason"]
    if ga["row"]["lane"][:2]!=gb["row"]["lane"][:2]:return "not_applicable","different_directed_roads_planar_context_not_established"
    za=[p[2] for c in a["footprint_components"] for p in c["world_vertices_m"]]
    zb=[p[2] for c in b["footprint_components"] for p in c["world_vertices_m"]]
    if max(za)<min(zb)-EPS or max(zb)<min(za)-EPS:return "not_applicable","vertically_disjoint_outer_envelopes"
    return "valid",""


def pair_record(run,a,b,frame,ga,gb,lanes,horizon):
    step,time=frame["step"],frame["time_s"];ids=[a["actor_id"],b["actor_id"]]
    st,reason=context(a,b,ga,gb);metrics=[]
    def add(name,value,unit,status=st,why=reason,method="geometry_v02",component_ids=None,**extra):
        item=metric(name,value,unit,ids,step,time,status,why,component_ids,method,**extra);metrics.append(item);return item
    add("reference_point_planar_separation",math.dist(a["position_world_m"][:2],b["position_world_m"][:2]),"m",status="valid",why="reference_point_only",method="reference_point_euclidean")
    detail=actor_separation(ga["components"],gb["components"]) if st=="valid" else None
    selected=[[ids[i],detail["component_ids"][i]] for i in range(2)] if detail else []
    add("outer_envelope_planar_separation",detail["distance"] if detail else None,"m",method="convex_boundary_minimum_over_all_component_pairs",component_ids=selected)
    add("outer_envelope_relation",detail["relation"] if detail else None,"category",method="convex_SAT_and_intersection_area",component_ids=selected)
    shared=[key for key in set(ga["occupancy"])&set(gb["occupancy"]) if ga["occupancy"][key] and gb["occupancy"][key]
            and ga["occupancy"][key]["intersection_area"]>AREA_EPS and gb["occupancy"][key]["intersection_area"]>AREA_EPS]
    gap_status,gap_reason=st,reason;gap=None;longitudinal=None
    if st=="valid":
        if not shared:gap_status,gap_reason="not_applicable","no_geometrically_shared_lane"
        else:
            key=sorted(shared)[0];axis=lanes[key]["tangent"]
            if any(math.cos(actor["heading_rad"])*axis[0]+math.sin(actor["heading_rad"])*axis[1]<=0 or sum(actor["velocity_world_m_s"][i]*axis[i] for i in range(2)) < -1e-9 for actor in [a,b]):
                gap_status,gap_reason="not_applicable","different_or_reverse_travel_direction"
            else:
                ref=[sum(actor["position_world_m"][i]*axis[i] for i in range(2)) for actor in [a,b]]
                if abs(ref[0]-ref[1])<=EPS:gap_status,gap_reason="undefined","ambiguous_longitudinal_actor_order"
                else:
                    following,leading=(0,1) if ref[0]<ref[1] else (1,0)
                    components=[ga["components"],gb["components"]]
                    project=lambda index:[sum(p[i]*axis[i] for i in range(2)) for poly in components[index].values() for p in poly]
                    front,rear=max(project(following)),min(project(leading));gap=rear-front
                    longitudinal=dict(following_actor_id=ids[following],leading_actor_id=ids[leading],shared_lane_ids=[list(k) for k in sorted(shared)],
                        following_front_projection_m=front,leading_rear_projection_m=rear,
                        reference_point_longitudinal_separation_m=abs(ref[1]-ref[0]),
                        longitudinal_relation="overlapping_projections" if gap< -EPS else "touching_projections" if abs(gap)<=EPS else "separated_projections")
    all_components=[[identity,i] for identity,g in zip(ids,[ga,gb]) for i in (g["components"] or {})]
    add("shared_path_signed_longitudinal_envelope_gap",gap,"m",gap_status,gap_reason,
        "min_leader_vertex_projection_minus_max_follower_vertex_projection",all_components)
    prediction=actor_ttc(ga["components"],gb["components"],a["velocity_world_m_s"][:2],b["velocity_world_m_s"][:2],horizon) if st=="valid" else dict(value=None,validity=st,reason=reason,component_ids=[])
    ttc=add("constant_velocity_fixed_orientation_envelope_ttc_2d",prediction["value"],"s",prediction["validity"],prediction["reason"],
        "continuous_separating_axis_time_intervals",[[ids[i],c] for i,c in enumerate(prediction.get("component_ids",[]))],
        prediction_horizon_s=horizon,orientation_held_fixed=True,geometry_representation="projected_box_outer_envelope_components",
        velocity_source="runtime_Bullet_chassis_getLinearVelocity",point_approximation=False,
        assumptions=["Current envelopes translated at constant world XY velocities", "All footprint orientations fixed despite recorded angular velocity",
                     "No acceleration or future elevation/path changes; common planar context assumed over horizon"],
        uncertainty_note=UNCERTAINTY+" This is a model time to envelope intersection, not actual Bullet contact or real-world prediction.")
    add("pet",None,"s","unavailable","no_defined_shared_conflict_zone_or_occupancy_times","not_implemented")
    add("exact_physical_surface_clearance",None,"m","unavailable","outer_envelope_not_exact_margin_contact_surface","not_implemented")
    return dict(run_id=run["manifest"]["run_id"],step=step,time_s=time,actor_ids=ids,actor_roles=[a["role"],b["role"]],
        metrics=metrics,closest_geometry=detail,longitudinal_geometry=longitudinal,
        prediction_details=prediction,risk_assessment="unassessed")
