"""Strict parsing and synchronization checks for the existing output contract."""
import ast
import csv
import hashlib
import json
import math
import struct


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    def bad(value):
        raise ValueError(f"Nonstandard JSON number: {value}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=bad)


def load_run(source):
    metadata = read_json(source / "metadata.json")
    dt = float(metadata["dt_s"])
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("Expected positive finite dt_s")
    tracks, by_step, rows = {}, {}, []
    with (source / "trajectory.csv").open(newline="", encoding="utf-8") as file:
        for line, raw in enumerate(csv.DictReader(file), 2):
            r = dict(raw)
            r["step"] = int(raw["step"])
            if r["step"] < 0:
                raise ValueError("Negative step")
            for k in ["time_s", "x", "y", "heading_rad", "speed_kmh", "source_longitudinal_m", "source_lateral_m", "target_lateral_m"]:
                r[k] = float(raw[k])
                if not math.isfinite(r[k]):
                    raise ValueError(f"Non-finite {k} at CSV line {line}")
            if r["speed_kmh"] < 0:
                raise ValueError("Negative scalar speed")
            for k in ["on_lane", "white_line", "yellow_line", "sidewalk", "same_road", "crash"]:
                if raw[k] not in ("True", "False"):
                    raise ValueError(f"Invalid boolean {k}")
                r[k] = raw[k] == "True"
            lane = ast.literal_eval(raw["lane"])
            if not isinstance(lane, tuple) or len(lane) != 3 or not all(isinstance(x, str) for x in lane[:2]) or type(lane[2]) is not int:
                raise ValueError("Invalid lane tuple")
            r["lane"] = lane
            if not math.isclose(r["time_s"], r["step"] * dt, abs_tol=1e-9, rel_tol=0):
                raise ValueError("Step/time mismatch")
            track = tracks.setdefault(r["vehicle_id"], {})
            if r["step"] in track or (track and next(iter(track.values()))["role"] != r["role"]):
                raise ValueError("Duplicate sample or changing role within actor track")
            r["speed_ms"] = r["speed_kmh"] / 3.6
            r["source_csv_line"] = line
            track[r["step"]] = r
            by_step.setdefault(r["step"], []).append(r)
            rows.append(r)
    if set(by_step) != set(range(metadata["settings"]["steps"] + 1)):
        raise ValueError("Missing/extra global simulation steps")
    frames = []
    with (source / "camera" / "frames.jsonl").open(encoding="utf-8") as file:
        for line in file:
            frame = json.loads(line)
            step = frame["step"]
            if type(step) is not int or step != len(frames):
                raise ValueError("Duplicate, missing or out-of-order frame step")
            states = [frame["subject"]] + frame["traffic"]
            if step not in by_step or len(states) != len(by_step[step]):
                raise ValueError("Frame actor count mismatch")
            actual = {r["vehicle_id"]: r for r in by_step[step]}
            if len({s["vehicle_id"] for s in states}) != len(states):
                raise ValueError("Duplicate frame actor")
            if len({s["role"] for s in states}) != len(states) or frame["subject"]["role"] != "subject":
                raise ValueError("Ambiguous frame roles")
            for s in states:
                r = actual[s["vehicle_id"]]
                for k, value in s.items():
                    if k == "lane":
                        value = ast.literal_eval(value)
                    if k not in r or r[k] != value:
                        raise ValueError(f"Frame/CSV mismatch at step {step}: {k}")
                if not set(r).difference(["source_csv_line", "speed_ms"]).issubset(s):
                    raise ValueError("Missing frame state fields")
            if frame["time_s"] != by_step[step][0]["time_s"]:
                raise ValueError("Frame time mismatch")
            path = (source / "camera" / frame["file"]).resolve()
            if not path.is_relative_to((source / "camera" / "frames").resolve()):
                raise ValueError("Frame path escapes frame directory")
            with path.open("rb") as image:
                header = image.read(33)
            if header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR" or struct.unpack(">II", header[16:24]) != (640, 360) or header[24:26] != b"\x08\x02":
                raise ValueError("Expected 8-bit RGB 640x360 PNG")
            frames.append(frame)
    if len(frames) != len(by_step) or len({f["file"] for f in frames}) != len(frames):
        raise ValueError("Frame metadata count/file identity mismatch")
    if {p.resolve() for p in (source / "camera" / "frames").glob("*.png")} != {(source / "camera" / f["file"]).resolve() for f in frames}:
        raise ValueError("Unlisted/missing PNG files")
    return metadata, dt, rows, tracks, by_step, frames


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def write_jsonl(path, records):
    with path.open("w", encoding="utf-8") as file:
        for r in records:
            file.write(json.dumps(r, sort_keys=True, allow_nan=False) + "\n")
