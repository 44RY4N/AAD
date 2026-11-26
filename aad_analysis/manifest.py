"""Content-addressed provenance and deliberately incomplete historical geometry."""
import ast
import hashlib
import json
import math

from . import VERSION
from .io import sha256


def inventory(root):
    result = []
    for path in sorted(root.rglob("*")):
        if path.is_file():
            if not path.resolve().is_relative_to(root):
                raise ValueError("Source inventory contains an external link")
            result.append(dict(path=path.relative_to(root).as_posix(), bytes=path.stat().st_size, sha256=sha256(path)))
    return result


def digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def build_manifest(source, repo, metadata, rows, frames, files):
    start, end = metadata["source_start"], metadata["source_end"]
    width = float(metadata["settings"]["lane_width"])
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if not all(math.isfinite(v) for v in start + end + [width]) or length <= 0 or width <= 0:
        raise ValueError("Invalid source-lane geometry")
    e = [dx / length, dy / length]
    n = [e[1], -e[0]]
    # Assert that the saved fixed-source coordinates use this straight reference.
    for r in rows:
        delta = [r["x"] - start[0], r["y"] - start[1]]
        if abs(sum(a*b for a,b in zip(delta, e)) - r["source_longitudinal_m"]) > 1e-4 or abs(sum(a*b for a,b in zip(delta, n)) - r["source_lateral_m"]) > 1e-4:
            raise ValueError("Recorded coordinates do not match a fixed straight reference")
    code_paths = sorted((repo / "aad_analysis").glob("*.py")) + [repo / p for p in [
        "experiments/run.py", "experiments/scenarios.py", "experiments/recording.py",
        "experiments/annotation.py", "experiments/lane_geometry.py",
        "metadrive/component/vehicle/vehicle_type.py", "metadrive/component/lane/straight_lane.py"]]
    code = [dict(path=p.relative_to(repo).as_posix(), sha256=sha256(p)) for p in code_paths]
    vehicle_path = repo / "metadrive/component/vehicle/vehicle_type.py"
    vehicle_class = next(x for x in ast.parse(vehicle_path.read_text()).body if isinstance(x, ast.ClassDef) and x.name == "DefaultVehicle")
    defaults = {target.id: ast.literal_eval(node.value) for node in vehicle_class.body if isinstance(node, ast.Assign)
                for target in node.targets if isinstance(target, ast.Name) and target.id in ["DEFAULT_LENGTH", "DEFAULT_WIDTH", "DEFAULT_HEIGHT"]}
    actors = {}
    for r in rows:
        actor = actors.setdefault(r["vehicle_id"], dict(actor_id=r["vehicle_id"], role=r["role"],
            recorded_actor_class=None, recorded_dimensions_m=None, recorded_footprint=None,
            source_inference=dict(status="source_inferred_not_runtime_verified", candidate_class="DefaultVehicle",
                candidate_dimensions_m={k.removeprefix("DEFAULT_").lower(): v for k,v in defaults.items()},
                evidence=["experiments/scenarios.py", "metadrive/component/vehicle/vehicle_type.py"],
                limitations="Current source is not proof of capture-time resolved class, scaling, reference offset or collision footprint"),
            first_step=r["step"], last_step=r["step"]))
        actor["first_step"] = min(actor["first_step"], r["step"])
        actor["last_step"] = max(actor["last_step"], r["step"])
    source_index = metadata["source_lane"][2]
    lanes = []
    for index in sorted({r["lane"] for r in rows if list(r["lane"][:2]) == metadata["source_lane"][:2]}):
        offset = (index[2] - source_index) * width
        centre_start = [start[k] + offset * n[k] for k in range(2)]
        centre_end = [end[k] + offset * n[k] for k in range(2)]
        polygon = [[point[k] + sign * width/2*n[k] for k in range(2)]
                   for point,sign in [(centre_start,1),(centre_end,1),(centre_end,-1),(centre_start,-1)]]
        lanes.append(dict(lane_id=list(index), status="reconstructed_parallel_strip_not_runtime_polygon_export",
                          centre_start_m=centre_start, centre_end_m=centre_end, polygon_xy_m=polygon,
                          assumptions=["straight parallel lanes", "uniform recorded width", "increasing lane index follows source lateral normal"]))
    run_hash = digest(files)
    return dict(schema_version="0.1", extractor_version=VERSION, run_id="sha256:"+run_hash,
        scenario_id="settings-sha256:"+digest(metadata["settings"]), source_directory=str(source),
        source_files=files, input_inventory_sha256=run_hash,
        code_provenance=dict(status="current_checkout_not_capture_time_attestation", files=code),
        recorded_software=dict(python=metadata["python"], metadrive=metadata["simulator_version"]),
        recorded_settings=metadata["settings"], dt_s=metadata["dt_s"],
        geometry=dict(world_units="m", heading_units="rad", source_lane=metadata["source_lane"],
            source_start_m=start, source_end_m=end, tangent=e, signed_lateral_normal=n,
            coordinate_check_tolerance_m=1e-4, reconstructed_lanes=lanes,
            resolved_actor_footprints_available=False, conflict_zones=None),
        actors=sorted(actors.values(), key=lambda a:a["role"]),
        camera=dict(recorded_config=json.loads((source/"camera/camera.json").read_text()),
                    per_frame_pose_source="camera/frames.jsonl", hpr_units="degrees",
                    resolved_intrinsics=None, actor_world_z=None,
                    projected_points="vehicle origin local (0,0,0.7), not footprint/bbox or occlusion proof"),
        event=dict(recorded_physical_verification=metadata["result"],
                   retrospective_phases_source="camera/frames.jsonl", risk_assessment="unassessed"),
        contracts=dict(derivatives="offline central differences, unfiltered, no gap/road/localization bridging",
                       point_proxies="Reference-point separation/alignment only, never physical clearance or conflict proof",
                       footprint_metrics="unavailable; inferred default dimensions are deliberately not used",
                       collision_flags="trajectory crash=vehicle contact only; no general collision/injury conclusion"),
        frame_count=len(frames))
