"""Geometry-aware offline extraction; no driving simulation or input writes."""
import argparse
from collections import Counter,defaultdict
import hashlib
import importlib.metadata
import itertools
import json
import math
from pathlib import Path

from aad_analysis.io import sha256,write_json,write_jsonl
from aad_analysis.manifest import digest,inventory
from experiments.validate_geometry import normalize
from . import VERSION
from .inputs import load
from .measurements import actor_record,metric,pair_record,prepare_lanes


FEATURE_FILES=["actor_features.jsonl","pair_features.jsonl","road_occupancy.jsonl","pair_summary.jsonl"]


def records(run,horizon):
    lanes=prepare_lanes(run["data"]["road"])
    actors=[];pairs=[];road=[];series=defaultdict(list)
    for frame in run["data"]["frames"]:
        caches={}
        ordered=sorted(frame["actors"],key=lambda a:a["role"])
        for actor in ordered:
            a,o,g=actor_record(run,actor,frame,lanes);actors.append(a);road.append(o);caches[actor["actor_id"]]=g
        for a,b in itertools.combinations(ordered,2):
            pair=pair_record(run,a,b,frame,caches[a["actor_id"]],caches[b["actor_id"]],lanes,horizon)
            pairs.append(pair);series[tuple(pair["actor_ids"])].append(pair)
    summaries=[]
    for identities,items in series.items():
        measures=[next(m for m in item["metrics"] if m["name"]=="outer_envelope_planar_separation") for item in items]
        valid=[(item,m) for item,m in zip(items,measures) if m["validity"]=="valid"]
        complete=len(valid)==len(items)
        best=min(valid,key=lambda p:(p[1]["value"],p[0]["step"])) if complete else None
        first,last=items[0],items[-1]
        sample=metric("sampled_minimum_outer_envelope_planar_separation",best[1]["value"] if best else None,"m",identities,
            first["step"],first["time_s"],"valid" if best else "unavailable",
            "minimum_at_recorded_decision_samples_only" if best else "incomplete_pair_geometry_or_context_coverage",
            best[1]["shape_component_ids"] if best else [],"minimum_over_complete_observation_samples",
            sampling_interval_s=run["manifest"]["dt_s"])
        sample["time_support"].update(end_step=last["step"],end_s=last["time_s"])
        continuous=metric("continuous_time_minimum_physical_separation",None,"m",identities,first["step"],first["time_s"],
            "unavailable","10Hz_observations_do_not_establish_continuous_motion_or_exact_contact_surface",method="not_implemented")
        continuous["time_support"].update(end_step=last["step"],end_s=last["time_s"])
        summaries.append(dict(run_id=run["manifest"]["run_id"],actor_ids=list(identities),actor_roles=first["actor_roles"],
            samples=len(items),valid_separation_samples=len(valid),minimum_step=best[0]["step"] if best else None,
            minimum_relation=best[0]["closest_geometry"]["relation"] if best else None,
            metrics=[sample,continuous],risk_assessment="unassessed"))
    return dict(zip(FEATURE_FILES,[actors,pairs,road,summaries]))


def normalized_features(data,roles):
    def clean(value):
        if isinstance(value,dict):return {k:clean(v) for k,v in value.items() if k!="run_id"}
        if isinstance(value,list):return [clean(v) for v in value]
        return value
    return normalize(clean(data),roles)


def all_metrics(data):
    for items in data.values():
        for record in items:
            yield from record.get("metrics",[])
            if "footprint_area" in record:yield record["footprint_area"]
            for lane in record.get("lanes",[]):yield from lane["metrics"]


def extract(source,output,horizon=5.):
    source,output=Path(source).resolve(),Path(output).resolve()
    if not math.isfinite(horizon) or horizon<=0:raise ValueError("Positive finite prediction horizon required")
    if output.exists():raise FileExistsError(output)
    if output.is_relative_to(source) or source.is_relative_to(output):raise ValueError("Analysis directory must be separate from source tree")
    run=load(source);data=records(run,horizon)
    counts=defaultdict(Counter);reasons=defaultdict(Counter)
    required={"name","value","unit","validity","reason","method","actor_ids","shape_component_ids","time_support","coordinate_frame","assumptions","uncertainty_note"}
    for m in all_metrics(data):
        if not required<=m.keys():raise ValueError("Incomplete metric contract")
        if m["validity"]!="valid" and m["value"] is not None:raise ValueError("Non-valid value must be null")
        counts[m["name"]][m["validity"]]+=1;reasons[m["name"]][m["reason"]]+=1
    encoded=json.dumps(data,allow_nan=False)  # includes all geometry detail values
    fingerprint=digest(normalized_features(data,run["roles"]))
    if inventory(source)!=run["files"]:raise RuntimeError("Inputs changed while extracting")
    sources={p.name:sha256(p) for p in sorted(Path(__file__).parent.glob("*.py"))}
    manifest=dict(schema_version="0.2",extractor_version=VERSION,run_id=run["manifest"]["run_id"],
        source_directory=str(source),source_files=run["files"],source_inventory_sha256=digest(run["files"]),
        runtime_geometry_schema_version=run["manifest"]["schema_version"],runtime_provenance=run["manifest"]["provenance"],
        extractor_sources_sha256=sources,dependencies={"shapely":importlib.metadata.version("shapely")},
        prediction_horizon_s=horizon,numeric_tolerances=dict(separation_m=1e-9,zero_area_m2=1e-12),
        method_version="convex-envelope-methods-0.2.0",coordinate_frame="world_xy_m",dt_s=run["manifest"]["dt_s"],
        evidence=dict(runtime_recorded=["actor classes/dimensions/shape parameters/poses/velocities", "road polygon representation and simulator provenance"],
            reconstructed=["projected box outer-envelope components", "sampled/reconstructed lane geometry"],
            derived=["envelope separation/relation", "oriented longitudinal gap", "component-union lane occupancy", "fixed-velocity/orientation envelope TTC", "10Hz sampled minima"],
            unavailable=["exact physical surface clearance/contact", "continuous-time observed minimum", "PET without shared conflict zones"]),
        enclosure_contract="For validated unit rigid box transforms, each recorded core plus spherical margin lies within the with-margin axis box; affine transforms/XY projection preserve enclosure. Numerical contact algorithms and near-boundary tolerances are not exact surface-distance validation.",
        interpretation="Projected envelope geometry only. Overlap is not Bullet contact. Separation is not exact physical clearance; fixed-model TTC is not actual collision prediction.",
        pair_scope="All unordered pairs of captured actors; common-road/current-elevation applicability checked",
        lane_scope="Exported lanes on each actor's current directed road, geometric component-union intersection",
        risk_assessment="unassessed")
    output.mkdir(parents=True,exist_ok=False)
    for name,items in data.items():write_jsonl(output/name,items)
    manifest["feature_hashes"]={name:sha256(output/name) for name in FEATURE_FILES}
    write_json(output/"manifest.json",manifest)
    report=dict(passed=True,source_unchanged=True,read_only_geometry_validation=run["read_only_checks"],
        input_geometry_report_passed=run["saved_validation"]["passed"],samples=len(run["data"]["frames"]),
        record_counts={name:len(items) for name,items in data.items()},metric_validity_counts=dict(counts),metric_reason_counts=dict(reasons),
        normalized_feature_sha256=fingerprint,risk_assessment="unassessed",prediction_horizon_s=horizon)
    write_json(output/"validation.json",report)
    print(json.dumps({k:v for k,v in report.items() if k not in ["metric_validity_counts","metric_reason_counts","read_only_geometry_validation"]},indent=2))
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument("--source",type=Path,required=True);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--horizon-s",type=float,default=5.,help="Model prediction horizon, not a safety threshold")
    args=parser.parse_args();extract(args.source,args.output,args.horizon_s)
