"""Validate saved RGB frames against state samples and a reference trajectory."""
import argparse
import csv
import json
from pathlib import Path

from PIL import Image, ImageStat


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def validate(output, reference):
    metadata = json.loads((output / "metadata.json").read_text())
    reference_metadata = json.loads((reference / "metadata.json").read_text())
    rows = read_rows(output / "trajectory.csv")
    reference_rows = read_rows(reference / "trajectory.csv")
    without_ids = lambda data: [{k: v for k, v in r.items() if k != "vehicle_id"} for r in data]
    assert without_ids(rows) == without_ids(reference_rows), "Vehicle trajectories differ from reference"
    assert metadata["result"] == reference_metadata["result"], "Physical event or trajectory hash changed"
    def decisions(path, states):
        roles = {r["vehicle_id"]: r["role"] for r in states}
        data = json.loads((path / "decisions.json").read_text())
        return [{**r, "front_id": roles.get(r["front_id"], r["front_id"])} for r in data]
    assert decisions(output, rows) == decisions(reference, reference_rows), "IDM decisions changed"
    records = [json.loads(line) for line in (output / "camera" / "frames.jsonl").read_text().splitlines()]
    expected = metadata["settings"]["steps"] + 1
    assert len(records) == expected, "Wrong metadata count"
    assert [r["step"] for r in records] == list(range(expected)), "Missing or duplicated step"
    images = sorted((output / "camera" / "frames").glob("*.png"))
    assert len(images) == expected, "Frame and metadata counts differ"
    assert {r["file"] for r in records} == {p.relative_to(output / "camera").as_posix() for p in images}
    grouped = {}
    for row in rows:
        grouped.setdefault(int(row["step"]), []).append(row)
    subjects_in_view, leaders_in_view = 0, 0
    event = metadata["result"]
    for record in records:
        step = record["step"]
        assert record["time_s"] == step * metadata["dt_s"]
        frame_rows = [record["subject"]] + record["traffic"]
        assert len(frame_rows) == len(grouped[step])
        for recorded, logged in zip(frame_rows, grouped[step]):
            # Compare every recorded state field, not just positions or row counts.
            assert {k: str(v) for k, v in recorded.items()} == logged, f"State mismatch at {step}"
        phase = ("lane_following" if step < event["departure_step"] else
                 "lane_changing" if step < event["settled_step"] else
                 "target_tolerance_pending_confirmation" if step < event["confirmed_step"] else
                 "confirmed_target_lane")
        assert record["lane_change_state"] == phase
        with Image.open(output / "camera" / record["file"]) as im:
            im.load()  # Decode the entire PNG, not just its header.
            assert im.mode == "RGB" and im.size == (640, 360)
            assert max(ImageStat.Stat(im).stddev) > 1, f"Blank frame at {step}"
        projections = {p["role"]: p for p in record["projections"]}
        assert projections["subject"]["in_frustum"], f"Subject outside view at {step}"
        if event["departure_step"] <= step <= event["confirmed_step"]:
            assert projections["leader"]["in_frustum"], f"Leader outside event view at {step}"
        subjects_in_view += projections["subject"]["in_frustum"]
        leaders_in_view += projections["leader"]["in_frustum"]
    report = dict(passed=True, steps=expected - 1, frames=len(images), metadata_records=len(records),
        state_rows=len(rows), image_mode="RGB", resolution=[640, 360],
        exact_reference_trajectory_match=True, exact_event_match=True,
        exact_reference_idm_decisions_match=True, leader_in_frustum_during_event=True,
        all_frame_states_match_trajectory=True, all_images_decoded=True,
        subject_centre_in_frustum_frames=subjects_in_view, leader_centre_in_frustum_frames=leaders_in_view,
        visibility_note="Frustum projection is not an occlusion test; inspect images separately",
        reference=str(reference.resolve()), result=event)
    (output / "camera" / "validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    args = parser.parse_args()
    validate(args.output, args.reference)
