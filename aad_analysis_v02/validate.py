"""Read-only feature recomputation and normalized repeat checks; fresh report only."""
import argparse
import json
from pathlib import Path

from aad_analysis.io import read_json,sha256,write_json
from aad_analysis.manifest import digest
from .inputs import load,require
from .run import FEATURE_FILES,all_metrics,normalized_features,records


def check(output):
    output=Path(output)
    manifest=read_json(output/"manifest.json");report=read_json(output/"validation.json")
    require(manifest["schema_version"]=="0.2" and report["passed"] is True,"Incomplete analysis")
    for name in FEATURE_FILES:require(sha256(output/name)==manifest["feature_hashes"][name],"Feature artifact hash mismatch: "+name)
    for name,expected in manifest["extractor_sources_sha256"].items():
        require(sha256(Path(__file__).parent/name)==expected,"Extractor code differs from saved provenance: "+name)
    run=load(manifest["source_directory"])
    require(run["files"]==manifest["source_files"],"Source inventory changed")
    require(manifest["run_id"]==run["manifest"]["run_id"],"Run identity mismatch")
    data={name:[json.loads(line) for line in (output/name).read_text().splitlines()] for name in FEATURE_FILES}
    expected=records(run,manifest["prediction_horizon_s"])
    require(data==expected,"Feature recomputation differs")
    needed={"name","value","unit","validity","reason","method","actor_ids","shape_component_ids","time_support","coordinate_frame","assumptions","uncertainty_note"}
    for m in all_metrics(data):
        require(needed<=m.keys(),"Missing metric contract field")
        require(m["validity"]=="valid" or m["value"] is None,"Invalid metric must be null")
    for rows in data.values():require(all(r["risk_assessment"]=="unassessed" for r in rows),"Unexpected risk assignment")
    fingerprint=digest(normalized_features(data,run["roles"]))
    require(fingerprint==report["normalized_feature_sha256"],"Feature fingerprint mismatch")
    return dict(passed=True,full_feature_recomputation_match=True,source_hashes_match=True,feature_hashes_match=True,
                normalized_feature_sha256=fingerprint,record_counts={k:len(v) for k,v in data.items()},
                risk_assessment="unassessed"),manifest


def validate(output,compare=None,report_path=None):
    output=Path(output);report_path=Path(report_path) if report_path else output/"feature_validation.json"
    if report_path.exists():raise FileExistsError("Choose a fresh report path: "+str(report_path))
    result,manifest=check(output)
    if compare:
        other,other_manifest=check(Path(compare))
        require(manifest["prediction_horizon_s"]==other_manifest["prediction_horizon_s"],"Cannot compare different prediction models")
        require(result["normalized_feature_sha256"]==other["normalized_feature_sha256"],"Normalized repeated-run features differ")
        result["normalized_repeat_match"]=True;result["comparison_directory"]=str(Path(compare).resolve())
    # Never allow a report inside raw source recordings.
    source=Path(manifest["source_directory"]).resolve()
    require(not report_path.resolve().is_relative_to(source),"Report must not modify raw source")
    write_json(report_path,result);print(json.dumps(result,indent=2));return result


if __name__=="__main__":
    parser=argparse.ArgumentParser(__doc__);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--compare",type=Path);parser.add_argument("--report",type=Path)
    args=parser.parse_args();validate(args.output,args.compare,args.report)
