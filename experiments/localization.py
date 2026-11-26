"""Read-only ray instrumentation and a minimal single-vehicle reproduction."""
import argparse
import csv
import json
from pathlib import Path

from panda3d.core import Vec3, NodePath, TransformState
from panda3d.bullet import BulletWorld, BulletRigidBodyNode, BulletConvexHullShape, BulletBoxShape
from metadrive import MetaDriveEnv
from metadrive.type import MetaDriveType
from metadrive.component.map.pg_map import MapGenerateMethod
from metadrive.utils.pg.utils import ray_localization
from metadrive.policy.idm_policy import IDMPolicy
from experiments.lane_geometry import replace_straight_lane_hulls


def isolated_probe(lane):
    """Exercise Bullet directly, with no engine, vehicle, navigation or filtering."""
    result = {}
    centre = (lane.start + lane.end) / 2
    corners = [lane.position(s, lat) for s in [0, lane.length] for lat in [-lane.width / 2, lane.width / 2]]
    points = [("dense_hull", lane.polygon, False), ("corner_hull", corners, False),
              ("centred_corner_hull", corners, True), ("box", corners, True),
              ("segmented_box", corners, True)]
    for name, polygon, centred in points:
        world = BulletWorld()
        node = BulletRigidBodyNode(name)
        if name in ["box", "segmented_box"]:
            shape = BulletBoxShape(Vec3(lane.length / 2, lane.width / 2, 0.05))
        else:
            shape = BulletConvexHullShape()
            for point in polygon:
                for z in [0, -0.1]:
                    shape.addPoint(Vec3(float(point[0] - (centre[0] if centred else 0)),
                                        float(point[1] - (centre[1] if centred else 0)),
                                        z + (0.05 if centred else 0)))
        transform = TransformState.makePos(Vec3(float(centre[0]), float(centre[1]), -0.05)) if centred else TransformState.makeIdentity()
        if name == "segmented_box":
            import math
            count = math.ceil(lane.length / 20)
            length = lane.length / count
            for i in range(count):
                start = max(0, i * length - 0.01)
                end = min(lane.length, (i + 1) * length + 0.01)
                c = lane.position((start + end) / 2, 0)
                node.addShape(BulletBoxShape(Vec3((end - start) / 2, lane.width / 2, 0.05)),
                              TransformState.makePos(Vec3(float(c[0]), float(c[1]), -0.05)))
        else:
            node.addShape(shape, transform)
        world.attachRigidBody(node)
        misses = {}
        for height in [1, 10]:
            misses[str(height)] = [x / 10 for x in range(110, 4991)
                if not world.rayTestAll(Vec3(x / 10, float(centre[1]), height),
                                        Vec3(x / 10, float(centre[1]), -height)).hasHits()]
        result[name] = dict(points_per_shape=8 if name in ["box", "segmented_box"] else len(polygon) * 2,
                            shapes=node.getNumShapes(),
                            misses=misses, margin=shape.getMargin())
        world.removeRigidBody(node)
    return result


def ray_hits(engine, position, height=1):
    x, y = position[:2]
    result = engine.physics_world.static_world.rayTestAll(Vec3(x, y, height), Vec3(x, y, -height))
    return [dict(name=h.getNode().getName(), fraction=float(h.getHitFraction()),
                 position=list(h.getHitPos()), normal=list(h.getHitNormal())) for h in result.getHits()]


def geometry(env):
    bodies = []
    for node in env.engine.physics_world.static_world.getRigidBodies():
        if MetaDriveType.is_lane(node.getName()):
            bodies.append(dict(name=node.getName(), position=list(NodePath(node).getPos(env.engine.worldNP)),
                shapes=[dict(type=type(node.getShape(i)).__name__,
                             bounds=str(node.getShape(i).getShapeBounds()),
                             margin=float(node.getShape(i).getMargin())) for i in range(node.getNumShapes())]))
    return bodies


def snapshot(env, source, step):
    v = env.agent
    hits = ray_hits(env.engine, v.position)
    lanes, on_lane = ray_localization(v.heading, v.position, env.engine, return_on_lane=True)
    s, lateral = source.local_coordinates(v.position)
    return dict(step=step, time_s=step * 0.1, x=float(v.position[0]), y=float(v.position[1]),
        longitudinal_m=s, lateral_m=lateral, speed_kmh=float(v.speed_km_h),
        vehicle_on_lane=bool(v.on_lane), query_on_lane=on_lane,
        localized_lanes=json.dumps([list(index) for _, index, _ in lanes]),
        ray_hits=json.dumps(hits), tall_ray_hits=json.dumps(ray_hits(env.engine, v.position, 10)),
        crash_vehicle=bool(v.crash_vehicle), crash_object=bool(v.crash_object),
        sidewalk=bool(v.crash_sidewalk), white_line=bool(v.on_white_continuous_line),
        yellow_line=bool(v.on_yellow_continuous_line))


def run(output, steps=300, box_lanes=False):
    output.mkdir(parents=True, exist_ok=False)
    env = MetaDriveEnv(dict(map_config={"type": MapGenerateMethod.BIG_BLOCK_SEQUENCE,
        "config": "S", "exit_length": 500, "lane_num": 3, "lane_width": 3.5},
        traffic_density=0, random_spawn_lane_index=False, agent_policy=IDMPolicy,
        horizon=steps + 1, agent_configs={"default_agent": {
            "spawn_lane_index": (">>", ">>>", 1), "spawn_longitude": 40}}))
    rows = []
    summary = {}
    try:
        env.reset(seed=0)
        (output / "isolated_shape_probe.json").write_text(json.dumps(isolated_probe(env.agent.lane), indent=2))
        replaced = replace_straight_lane_hulls(env.engine) if box_lanes else 0
        source = env.agent.lane
        env.engine.get_policy(env.agent.id).enable_lane_change = False
        (output / "geometry.json").write_text(json.dumps(geometry(env), indent=2))
        scan = [dict(x=x, y=y, hits=ray_hits(env.engine, (x, y)))
                for y in [-10, 0, 3.5, 7, 30] for x in range(11, 500)]
        (output / "spatial_scan.json").write_text(json.dumps(scan, indent=2))
        rows.append(snapshot(env, source, 0))
        for step in range(1, steps + 1):
            _, _, terminated, truncated, info = env.step([0, 0])
            rows.append(snapshot(env, source, step))
            if terminated or truncated:
                summary = dict(completed=False, stopped_step=step, info=info)
                break
        else:
            summary = dict(completed=True, stopped_step=steps)
        summary["all_localized"] = all(r["vehicle_on_lane"] and r["query_on_lane"] for r in rows)
        summary["replaced_lane_shapes"] = replaced
        summary["centre_scan_misses"] = sum(not r["hits"] for r in scan if r["y"] in [0, 3.5, 7])
        summary["outside_scan_false_hits"] = sum(bool(r["hits"]) for r in scan if r["y"] in [-10, 30])
        (output / "summary.json").write_text(json.dumps(summary, indent=2))
    finally:
        env.close()
        if rows:
            with (output / "rays.csv").open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--box-lanes", action="store_true")
    args = parser.parse_args()
    summary = run(args.output, args.steps, args.box_lanes)
    raise SystemExit(0 if summary["completed"] and summary["all_localized"] else 1)
