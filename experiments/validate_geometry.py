"""Validate separate runtime geometry artifacts; optional normalized repeat check."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from aad_analysis.io import read_json, sha256, write_json
from experiments.geometry_math import convex_hull_xy, transform_point


TOLERANCE_M=1e-4


def require(condition,message):
    if not condition:raise ValueError(message)


def close(a,b,tolerance=TOLERANCE_M):
    if isinstance(a,list) and isinstance(b,list):
        return len(a)==len(b) and all(close(x,y,tolerance) for x,y in zip(a,b))
    return isinstance(a,(int,float)) and isinstance(b,(int,float)) and abs(a-b)<=tolerance


def finite(data):
    if isinstance(data,float):require(math.isfinite(data),"Non-finite value")
    elif isinstance(data,dict):
        for v in data.values():finite(v)
    elif isinstance(data,list):
        for v in data:finite(v)


def normalize(data,roles):
    if isinstance(data,dict):return {k:normalize(v,roles) for k,v in data.items()}
    if isinstance(data,list):return [normalize(v,roles) for v in data]
    if isinstance(data,str):return roles.get(data,data)
    return data


def validated_data(output):
    geo=output/"geometry"
    manifest=read_json(geo/"geometry_manifest.json")
    static=read_json(geo/"actor_geometry.json")
    road=read_json(geo/"road_geometry.json")
    calibration=read_json(geo/"camera_calibration.json")
    frames=[json.loads(line) for line in (geo/"geometry_frames.jsonl").read_text().splitlines()]
    finite([manifest,static,road,calibration,frames])
    require(manifest["schema_version"]=="0.2" and manifest["completed"],"Incomplete/unsupported geometry run")
    actors={a["actor_id"]:a for a in static["actors"]}
    roles={key:a["role"] for key,a in actors.items()}
    require(len(actors)==len(static["actors"])==len(set(roles.values())),"Duplicate identity/role")
    require(manifest["all_capture_guards_passed"],"Capture guard failed")
    require(manifest["provenance"]["source_files"] and manifest["provenance"]["software"],"Missing provenance")
    unsupported=0;max_face_error=0.
    for a in actors.values():
        require(all(v>0 for v in a["resolved_dimensions_m"].values()),"Invalid dimensions")
        require(a["reference"]["origin_is_chassis"] and a["reference"]["body_is_vehicle_system_chassis"],"Unvalidated reference definition")
        require(a["shapes"],"Missing chassis shape")
        for s in a["shapes"]:
            mat=s["shape_to_actor_matrix"]
            require(len(mat)==4 and all(len(r)==4 for r in mat),"Bad shape transform")
            require(close(s["centre_offset_actor_m"],transform_point([0,0,0],mat)),"Bad reference offset")
            if s["actor_local_vertices_m"] is None:
                unsupported+=1;require(s["status"]=="unavailable" and s["reason"],"Missing unsupported reason");continue
            require(len(s["shape_local_vertices_m"])==len(s["actor_local_vertices_m"])==8,"Expected eight box vertices")
            require(close(s["actor_local_vertices_m"],[transform_point(p,mat) for p in s["shape_local_vertices_m"]]),"Incorrect shape-local transform")
            require(close(s["actor_local_footprint_xy_m"],convex_hull_xy(s["actor_local_vertices_m"])),"Incorrect local footprint")
            require(all(v>0 for v in s["half_extents_with_margin_m"]),"Invalid collision extents")
        for probe in a["bullet_face_crosschecks"]:
            if probe["status"]!="unavailable":
                max_face_error=max(max_face_error,probe["max_error_m"])
                require(probe["max_error_m"]<=TOLERANCE_M,"Bullet face probe discrepancy")
    with (output/"trajectory.csv").open(newline="") as file:rows=list(csv.DictReader(file))
    indexed={}
    for r in rows:
        key=(int(r["step"]),r["vehicle_id"]);require(key not in indexed,"Duplicate trajectory row");indexed[key]=r
    expected=list(range(manifest["decision_steps"]+1))
    require([f["step"] for f in frames]==expected,"Missing/duplicate/out-of-order geometry steps")
    require(len(rows)==len(frames)*len(actors),"Trajectory sample count mismatch")
    rgb=None
    if manifest["rgb_enabled"]:
        rgb=[json.loads(line) for line in (output/"camera/frames.jsonl").read_text().splitlines()]
        require([r["step"] for r in rgb]==expected,"RGB steps mismatch")
        require(len(list((output/"camera/frames").glob("*.png")))==len(frames),"RGB PNG count mismatch")
        require(calibration["status"]!="unavailable","RGB calibration missing")
    max_transform_error=0.;max_projection_error=0.
    for f in frames:
        step=f["step"]
        require(math.isclose(f["time_s"],step*manifest["dt_s"],abs_tol=1e-9,rel_tol=0),"Geometry timestamp mismatch")
        require(f["read_only_guard_passed"],"Capture modified state")
        require(len(f["actors"])==len(actors) and {a["actor_id"] for a in f["actors"]}==set(actors),"Missing/duplicate actor")
        for a in f["actors"]:
            identity=a["actor_id"];r=indexed[step,identity];info=actors[identity]
            require(a["role"]==r["role"]==info["role"],"Role mismatch")
            require(close(a["position_world_m"][:2],[float(r["x"]),float(r["y"])]),"Geometry position disagrees with trajectory")
            require(abs(f["time_s"]-float(r["time_s"]))<1e-9,"Trajectory timestamp mismatch")
            require(abs(a["heading_rad"]-float(r["heading_rad"]))<1e-6,"Heading mismatch")
            require(abs(a["speed_kmh"]-float(r["speed_kmh"]))<1e-4,"Speed mismatch")
            require(close(a["position_world_m"],transform_point([0,0,0],a["actor_to_world_matrix"])),"Pose/reference mismatch")
            require(abs(math.hypot(*a["velocity_world_m_s"][:2])*3.6-a["speed_kmh"])<1e-4,"Velocity units mismatch")
            require(len(a["footprint_components"])==len(info["shapes"]),"Missing shape component")
            for component,s in zip(a["footprint_components"],info["shapes"]):
                require(component["index"]==s["index"],"Shape identity mismatch")
                if component["world_vertices_m"] is None:
                    require(component["reason"],"Unavailable footprint needs reason");continue
                predicted=[transform_point(p,a["actor_to_world_matrix"]) for p in s["actor_local_vertices_m"]]
                require(close(predicted,component["world_vertices_m"]),"World shape transform mismatch")
                require(close(convex_hull_xy(predicted),component["world_footprint_xy_m"]),"World footprint mismatch")
                error=component["bullet_transform_crosscheck_max_error_m"]
                require(error is not None and error<=TOLERANCE_M,"Native Bullet/Panda transform discrepancy")
                max_transform_error=max(max_transform_error,error)
        if rgb is not None:
            r=rgb[step];require(r["time_s"]==f["time_s"],"RGB timestamp mismatch")
            require({a["vehicle_id"] for a in [r["subject"]]+r["traffic"]}==set(actors),"RGB actor identities mismatch")
            c=f["camera"];require(c is not None,"Missing per-frame camera")
            require(close(c["world_position_m"],r["camera_world_position"]) and close(c["world_hpr_degrees"],r["camera_world_hpr"]),"RGB camera pose mismatch")
            for p in [[0,0,0],[1,2,3]]:
                require(close(transform_point(transform_point(p,c["camera_to_world_matrix"]),c["world_to_camera_matrix"]),p),"Camera matrices not inverse")
            for a in f["actors"]:
                world=transform_point([0,0,.7],a["actor_to_world_matrix"])
                camera_point=transform_point(world,c["world_to_camera_matrix"])
                # Runtime projection matrix supplies NDC x/y after homogeneous divide.
                ndc=transform_point(camera_point,calibration["projection_matrix"])
                uv=[(ndc[0]+1)*calibration["width"]/2,(1-ndc[1])*calibration["height"]/2]
                projection=next(p for p in r["projections"] if p["vehicle_id"]==a["actor_id"])
                if projection["in_frustum"]:
                    error=max(abs(x-y) for x,y in zip(uv,projection["centre_pixel"]))
                    require(error<.01,"Saved matrix disagrees with RGB projection")
                    max_projection_error=max(max_projection_error,error)
                intr=calibration["intrinsics"]
                if intr and camera_point[1]>1e-6:
                    pinhole=[intr["cx_px"]+intr["fx_px"]*camera_point[0]/camera_point[1],intr["cy_px"]-intr["fy_px"]*camera_point[2]/camera_point[1]]
                    require(close(pinhole,uv,.01),"Pinhole model mismatch")
        else:require(f["camera"] is None,"Unexpected camera in geometry-only run")
    require(road["lanes"] and all(l["length_m"]>0 for l in road["lanes"]),"No valid road geometry")
    report=dict(passed=True,samples=len(frames),actor_samples=len(rows),actors=len(actors),unsupported_static_shapes=unsupported,
        rgb_synchronized=rgb is not None,tolerance_m=TOLERANCE_M,max_bullet_face_error_m=max_face_error,
        max_native_transform_error_m=max_transform_error,max_rgb_projection_error_px=max_projection_error,
        footprint_scope="Verified box outer envelopes, not exact rounded-margin contact contours",
        no_physics_advance_guards_passed=True)
    return report,dict(actors=static,road=road,frames=frames,calibration=calibration),roles


def validate(output,reference=None,compare_geometry=None):
    output=Path(output)
    report,data,roles=validated_data(output)
    if reference:
        reference=Path(reference)
        def trajectory(p):
            with (p/"trajectory.csv").open(newline="") as file:rows=list(csv.DictReader(file))
            return [{k:v for k,v in r.items() if k!="vehicle_id"} for r in rows],{r["vehicle_id"]:r["role"] for r in rows}
        rows,ids=trajectory(output);old,old_ids=trajectory(reference)
        require(rows==old,"Original vehicle trajectory changed")
        require(normalize(read_json(output/"decisions.json"),ids)==normalize(read_json(reference/"decisions.json"),old_ids),"Original IDM decisions changed")
        require(read_json(output/"metadata.json")["result"]==read_json(reference/"metadata.json")["result"],"Original event/checksum changed")
        report["reference_trajectory_idm_event_exact_match"]=True
    if compare_geometry:
        _,old,old_roles=validated_data(Path(compare_geometry))
        require(normalize(data,roles)==normalize(old,old_roles),"Normalized static/dynamic/road/camera geometry changed")
        report["normalized_geometry_repeat_exact_match"]=True
    report["normalized_geometry_sha256"]=hashlib.sha256(json.dumps(normalize(data,roles),sort_keys=True,allow_nan=False).encode()).hexdigest()
    report["artifact_hashes"]={p.name:sha256(p) for p in (output/"geometry").iterdir() if p.is_file() and p.name!="validation.json"}
    write_json(output/"geometry/validation.json",report)
    print(json.dumps(report,indent=2))
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--reference",type=Path)
    parser.add_argument("--compare-geometry",type=Path)
    args=parser.parse_args();validate(args.output,args.reference,args.compare_geometry)
