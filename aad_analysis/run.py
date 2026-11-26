"""Read-only offline extraction: python -m aad_analysis.run --source ... --output ..."""
import argparse
from collections import Counter, defaultdict
from pathlib import Path

from .features import actor_features, metric, pair_features
from .io import load_run, sha256, write_json, write_jsonl
from .manifest import build_manifest, digest, inventory


def extract(source, output):
    source, output = source.resolve(), output.resolve()
    if source.is_relative_to(output) or output.is_relative_to(source):
        raise ValueError("Analysis output must be separate from the source tree")
    if output.exists():
        raise FileExistsError(f"Output already exists: {output}")
    original_files = inventory(source)
    metadata, dt, rows, tracks, by_step, frames = load_run(source)
    repo = Path(__file__).resolve().parents[1]
    manifest = build_manifest(source, repo, metadata, rows, frames, original_files)
    cached = {(actor, step): actor_features(track, step, dt) for actor, track in tracks.items() for step in track}
    actor_records, pair_records = [], []
    phases = {f["step"]:f["lane_change_state"] for f in frames}
    for row in sorted(rows, key=lambda r:(r["step"],r["role"])):
        actor, step = row["vehicle_id"], row["step"]
        measurements = dict(cached[actor, step])
        candidates = [other for other in by_step[step] if other["vehicle_id"] != actor and other["lane"] == row["lane"]
                      and other["on_lane"] and other["same_road"]]
        nearest = {}
        for direction, sign in [("front",1),("rear",-1)]:
            eligible = [(sign*(other["source_longitudinal_m"]-row["source_longitudinal_m"]),other)
                        for other in candidates if sign*(other["source_longitudinal_m"]-row["source_longitudinal_m"]) > 0]
            selected = min(eligible, key=lambda v:v[0]) if eligible else None
            valid = row["on_lane"] and row["same_road"] and selected is not None
            measurements[f"nearest_assigned_lane_{direction}_reference_gap"] = metric(
                selected[0] if valid else None, "m", "assigned_lane_reference_separation_not_bumper_gap",
                "valid" if valid else "unavailable", "" if valid else "no_eligible_actor_or_invalid_localization", [step])
            nearest[direction] = dict(actor_id=selected[1]["vehicle_id"], role=selected[1]["role"]) if valid else None
        measurements["footprint_lane_occupancy"] = metric(None, "fraction", "not_implemented_without_geometry_contract", "unavailable", "resolved_footprints_not_recorded")
        measurements["conflict_zone_occupancy"] = metric(None, "boolean", "not_implemented_without_geometry_contract", "unavailable", "conflict_zones_not_recorded")
        actor_records.append(dict(run_id=manifest["run_id"], step=step, time_s=row["time_s"],
            actor_id=actor, role=row["role"], source_csv_line=row["source_csv_line"],
            recorded_lane=list(row["lane"]), subject_phase=phases[step] if row["role"] == "subject" else None,
            nearest_assigned_lane_actors=nearest, metrics=measurements, risk_assessment="unassessed"))
    for step in sorted(by_step):
        subjects = [r for r in by_step[step] if r["role"] == "subject"]
        if len(subjects) != 1:
            raise ValueError("Expected one subject per frame")
        subject = subjects[0]
        for other in sorted(by_step[step], key=lambda r:r["role"]):
            if other is subject:
                continue
            pair_records.append(dict(run_id=manifest["run_id"], step=step, time_s=subject["time_s"],
                subject_id=subject["vehicle_id"], other_id=other["vehicle_id"], other_role=other["role"],
                metrics=pair_features(subject, other, cached[subject["vehicle_id"],step], cached[other["vehicle_id"],step]),
                risk_assessment="unassessed"))
    canonical = dict(actors=[{k:v for k,v in r.items() if k not in ["run_id","actor_id","nearest_assigned_lane_actors"]} for r in actor_records],
                     pairs=[{k:v for k,v in r.items() if k not in ["run_id","subject_id","other_id"]} for r in pair_records])
    # Preserve nearest actor roles while excluding generated IDs from comparison.
    for original, normal in zip(actor_records, canonical["actors"]):
        normal["nearest_actor_roles"] = {k:v["role"] if v else None for k,v in original["nearest_assigned_lane_actors"].items()}
    counts = defaultdict(Counter)
    for record in actor_records + pair_records:
        for name, m in record["metrics"].items():
            counts[name][m["validity"]] += 1
    if original_files != inventory(source):
        raise RuntimeError("Source files changed while extracting; analysis was not written")
    output.mkdir(parents=True, exist_ok=False)
    write_jsonl(output/"actor_features.jsonl", actor_records)
    write_jsonl(output/"pair_features.jsonl", pair_records)
    manifest["analysis_artifacts"] = {name:dict(sha256=sha256(output/name)) for name in ["actor_features.jsonl","pair_features.jsonl"]}
    write_json(output/"manifest.json", manifest)
    report = dict(passed=True, source_unchanged=True, source_file_count=len(original_files),
        synchronized_frames=len(frames), actor_feature_records=len(actor_records), subject_pair_records=len(pair_records),
        feature_fingerprint_without_generated_ids=digest(canonical), validity_counts=dict(counts),
        checks=dict(csv_frame_state_exact_match=True, timestamps_checked=True, png_headers_checked=True,
                    png_full_decode_checked=False, fixed_source_coordinate_reconstruction_checked=True),
        limitations=["No runtime footprint ground truth in historical outputs", "No criticality assessment",
                     "PNG headers checked; full image decoding belongs to saved recording validation"],
        recorded_physical_event=metadata["result"])
    write_json(output/"validation.json", report)
    return report


if __name__ == "__main__":
    import json
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = extract(args.source,args.output)
    print(json.dumps({k:v for k,v in report.items() if k != "validity_counts"}, indent=2))
