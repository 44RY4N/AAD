"""Read-only sidecar admission. Never invokes a report-writing validator CLI."""
import ast
import csv
import math
from pathlib import Path

from aad_analysis.io import read_json,sha256
from aad_analysis.manifest import inventory
from experiments.validate_geometry import validated_data
from experiments.geometry_math import box_vertices,transform_point

from .polygons import polygon


def require(condition,message):
    if not condition:raise ValueError(message)


def rigid(matrix,tolerance=1e-5):
    if len(matrix)!=4 or any(len(r)!=4 for r in matrix):return False
    if any(abs(matrix[i][3])>tolerance for i in range(3)) or abs(matrix[3][3]-1)>tolerance:return False
    return all(abs(sum(matrix[i][k]*matrix[j][k] for k in range(3))-(1 if i==j else 0))<=tolerance for i in range(3) for j in range(3))


def load(source):
    source=Path(source).resolve()
    require((source/"geometry/geometry_manifest.json").is_file(),"Runtime geometry sidecar required; historical geometry is not backfilled")
    files=inventory(source)
    saved=read_json(source/"geometry/validation.json")
    require(saved.get("passed") is True,"Successful saved geometry validation required")
    required={"actor_geometry.json","geometry_frames.jsonl","geometry_manifest.json","road_geometry.json","camera_calibration.json"}
    require(required<=set(saved.get("artifact_hashes",{})),"Saved validation lacks geometry hashes")
    for name in required:
        require(sha256(source/"geometry"/name)==saved["artifact_hashes"][name],"Geometry artifact hash mismatch: "+name)
    report,data,roles=validated_data(source)  # read-only function, not validate()/CLI
    manifest=read_json(source/"geometry/geometry_manifest.json")
    metadata=read_json(source/"metadata.json")
    require(manifest["schema_version"]==data["actors"]["schema_version"]==data["road"]["schema_version"]=="0.2","Unsupported geometry schema")
    require(manifest["capture_advances_physics"] is False and manifest["initial_state_included"] is True,"Unsupported sampling contract")
    expected_units={"length":"m","time":"s","velocity":"m/s","heading":"rad","hpr":"degrees","angular_velocity":"rad/s"}
    require(manifest["units"]==expected_units,"Unsupported geometry units")
    require(manifest["matrices"].startswith("Panda3D row-major"),"Unsupported matrix convention")
    require(manifest["dt_s"]==metadata["dt_s"] and manifest["decision_steps"]==metadata["settings"]["steps"],"Main metadata timing mismatch")
    require(manifest["provenance"]["settings"]==metadata["settings"],"Scenario provenance mismatch")
    require(saved["samples"]==manifest["samples"]==report["samples"],"Saved validation/sample count mismatch")
    with (source/"trajectory.csv").open(newline="",encoding="utf-8") as file:
        raw=list(csv.DictReader(file))
    rows={}
    for r in raw:
        for key in ["on_lane","same_road"]:require(r[key] in ["True","False"],"Malformed localization flag")
        for key in ["x","y","heading_rad","speed_kmh","source_longitudinal_m","source_lateral_m"]:
            r[key]=float(r[key]);require(math.isfinite(r[key]),"Nonfinite trajectory value")
        r["lane"]=ast.literal_eval(r["lane"]);r["step"]=int(r["step"])
        r["on_lane"]=r["on_lane"]=="True";r["same_road"]=r["same_road"]=="True"
        key=(r["step"],r["vehicle_id"]);require(key not in rows,"Duplicate trajectory join")
        rows[key]=r
    static={a["actor_id"]:a for a in data["actors"]["actors"]}
    reasons={}
    for identity,a in static.items():
        indices=[s["index"] for s in a["shapes"]]
        require(len(set(indices))==len(indices),"Duplicate shape component")
        for s in a["shapes"]:
            key=(identity,s["index"]);reason=None
            if s["status"]!="reconstructed_box_outer_envelope" or not s["shape_type"].endswith(".BulletBoxShape"):
                reason="unsupported_collision_shape"
            elif any(abs(v-1)>1e-6 for v in a["origin_scale_world"]) or not rigid(s["shape_to_actor_matrix"]):
                reason="unsupported_scale_or_shape_transform"
            else:
                outer,core,margin=s["half_extents_with_margin_m"],s["half_extents_without_margin_m"],s["margin_m"]
                require(len(outer)==len(core)==3 and margin>=0 and all(v>0 for v in outer+core),"Invalid box extents/margin")
                require(all(abs(core[i]+margin-outer[i])<1e-5 for i in range(3)),"Box core/margin does not fit outer extents")
                expected=box_vertices(outer)
                require(all(abs(x-y)<1e-6 for p,q in zip(expected,s["shape_local_vertices_m"]) for x,y in zip(p,q)),"Shape vertices do not represent recorded box extents")
            reasons[key]=reason
    for f in data["frames"]:
        for a in f["actors"]:
            require(len(a["position_world_m"])==len(a["velocity_world_m_s"])==3,"Invalid pose/velocity vector length")
            require(a["velocity_source"]=="runtime_Bullet_chassis_getLinearVelocity","Direct runtime world velocity required")
            for c in a["footprint_components"]:
                if reasons[a["actor_id"],c["index"]] is None and c["world_footprint_xy_m"] is not None:
                    polygon(c["world_footprint_xy_m"])  # reject malformed/nonconvex geometry
                    require(rigid(a["actor_to_world_matrix"]),"Unsupported nonrigid actor pose")
    return dict(source=source,files=files,saved_validation=saved,read_only_checks=report,data=data,
                manifest=manifest,metadata=metadata,roles=roles,static=static,rows=rows,component_reasons=reasons)


def components(run,actor):
    result={}
    for c in actor["footprint_components"]:
        reason=run["component_reasons"][actor["actor_id"],c["index"]]
        if reason or c["world_footprint_xy_m"] is None:return None,reason or c.get("reason") or "missing_world_component"
        result[c["index"]]=polygon(c["world_footprint_xy_m"])
    if not result:return None,"missing_components"
    return result,None
