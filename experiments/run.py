"""Run with .venv\\Scripts\\python.exe -m experiments.run."""
import argparse
import csv
import hashlib
import json
import platform
from pathlib import Path

import numpy as np
from metadrive import MetaDriveEnv
from metadrive.version import VERSION
from metadrive.component.map.pg_map import MapGenerateMethod


from experiments.scenarios import ObservedIDM, ControlledTraffic
from experiments.annotation import verify
from experiments.lane_geometry import replace_straight_lane_hulls
from experiments.localization import snapshot


def run(settings, output, render=False, box_lanes=False, record_rgb=False, record_geometry=False):
    output.mkdir(parents=True, exist_ok=False)
    config = dict(map_config={"type": MapGenerateMethod.BIG_BLOCK_SEQUENCE, "config": "S",
                  "lane_num": 3, "lane_width": settings["lane_width"],
                  "exit_length": settings["road_length"]}, traffic_density=0,
                  traffic_mode="basic", random_spawn_lane_index=False, agent_policy=ObservedIDM,
                  use_render=render, num_scenarios=1, start_seed=settings["seed"],
                  horizon=settings["steps"] + 1,
                  agent_configs={"default_agent": {"spawn_lane_index": (">>", ">>>", 1),
                                 "spawn_longitude": settings["subject_longitude"]}})
    if record_rgb:
        from experiments.recording import camera_config
        config.update(camera_config())
    env = MetaDriveEnv(config)
    rows, decisions, rays = [], [], []
    recorder, result, geometry, completed = None, None, None, False
    try:
        env.reset(seed=settings["seed"])
        replaced = replace_straight_lane_hulls(env.engine) if box_lanes else 0
        manager = ControlledTraffic()
        env.engine.register_manager("controlled_traffic", manager)
        manager.populate(settings)
        subject = env.agent
        policy = env.engine.get_policy(subject.id)
        policy.NORMAL_SPEED = settings["subject_speed_kmh"]
        source = subject.lane
        target = subject.navigation.current_ref_lanes[0]
        dt = env.config["physics_world_step_size"] * env.config["decision_repeat"]
        if record_rgb:
            from experiments.recording import RGBRecorder
            recorder = RGBRecorder(env, output)
        if record_geometry:
            from experiments.geometry_recording import GeometryRecorder
            geometry = GeometryRecorder(env, output, [("subject",subject)]+manager.vehicles,
                                        settings, dt, recorder.camera if recorder else None)
        for step in range(settings["steps"] + 1):
            rays.append(snapshot(env, source, step))
            for role, vehicle in [("subject", subject)] + manager.vehicles:
                longitudinal, lateral = source.local_coordinates(vehicle.position)
                rows.append(dict(step=step, time_s=step * dt, role=role, vehicle_id=vehicle.id,
                    x=float(vehicle.position[0]), y=float(vehicle.position[1]),
                    heading_rad=float(vehicle.heading_theta), speed_kmh=float(vehicle.speed_km_h),
                    on_lane=bool(vehicle.on_lane), white_line=bool(vehicle.on_white_continuous_line),
                    yellow_line=bool(vehicle.on_yellow_continuous_line), sidewalk=bool(vehicle.crash_sidewalk),
                    lane=str(vehicle.lane_index), source_longitudinal_m=float(longitudinal),
                    source_lateral_m=float(lateral), target_lateral_m=float(target.local_coordinates(vehicle.position)[1]),
                    same_road=vehicle.lane_index[:2] == source.index[:2], crash=bool(vehicle.crash_vehicle)))
            if recorder:
                recorder.capture(rows[-(len(manager.vehicles) + 1):], [("subject", subject)] + manager.vehicles)
            if geometry:
                geometry.capture(step)
            if step == settings["steps"]:
                break
            _, _, terminated, truncated, info = env.step([0, 0])
            decisions.append(dict(step=step + 1, **policy.decision))
            if terminated or truncated:
                rays.append(snapshot(env, source, step + 1))
                raise RuntimeError(f"Unexpected termination at {step + 1}: {info}; "
                    f"on_lane={subject.on_lane}, white={subject.on_white_continuous_line}, "
                    f"yellow={subject.on_yellow_continuous_line}, sidewalk={subject.crash_sidewalk}, "
                    f"position={subject.position}")
        completed = True
        result = verify(rows, settings["lane_width"])
        result["movement_m"] = {role: float(np.linalg.norm(
            np.array([r["x"], r["y"]]) - np.array([rows[i]["x"], rows[i]["y"]])))
            for i, (role, _) in enumerate([("subject", subject)] + manager.vehicles)
            for r in [next(r for r in reversed(rows) if r["role"] == role)]}
        numeric = [{k: v for k, v in r.items() if k != "vehicle_id"} for r in rows]
        result["trajectory_sha256_without_ids"] = hashlib.sha256(json.dumps(numeric, sort_keys=True).encode()).hexdigest()
        metadata = dict(settings=settings, simulator_version=VERSION, python=platform.python_version(),
                        simulator_config=repr(env.config), dt_s=dt, source_lane=list(source.index),
                        target_lane=list(target.index), source_start=source.start.tolist(),
                        source_end=source.end.tolist(), result=result,
                        replaced_lane_shapes=replaced,
                        all_subject_samples_localized=all(r["vehicle_on_lane"] and r["query_on_lane"] for r in rays))
        metadata["rgb_recording"] = bool(recorder)
        (output / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        (output / "decisions.json").write_text(json.dumps(decisions, indent=2), encoding="utf-8")
    finally:
        if geometry:
            geometry.finish(completed)
        if recorder:
            recorder.finish(result)
        if rays:
            with (output / "rays.csv").open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=list(rays[0]))
                writer.writeheader()
                writer.writerows(rays)
        env.close()
        if rows:
            with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    print(json.dumps(result, indent=2))
    if not result["verified"]:
        raise RuntimeError("No physically verified lane change")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("lane_change.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--box-lanes", action="store_true")
    parser.add_argument("--record-rgb", action="store_true", help="Save synchronized 640x360 RGB PNG frames")
    parser.add_argument("--record-geometry", action="store_true", help="Save separate runtime geometry/motion artifacts")
    args = parser.parse_args()
    run(json.loads(args.config.read_text()), args.output, args.render, args.box_lanes, args.record_rgb, args.record_geometry)
