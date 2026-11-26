"""Opt-in runtime workaround for long straight-lane convex-hull ray misses."""
import math

from panda3d.bullet import BulletBoxShape
from panda3d.core import NodePath, TransformState, Vec3
from metadrive.component.lane.straight_lane import StraightLane
from metadrive.type import MetaDriveType
from metadrive.utils.utils import get_object_from_node


def replace_straight_lane_hulls(engine, max_segment_length=20):
    """Keep each node, tag, mask and lane; use short boxes for its prism.

    Only valid for un-clipped, untransformed straight-lane localization bodies.
    No dynamic-world vehicle, ground, sidewalk or line collision body is changed.
    """
    if max_segment_length <= 0:
        raise ValueError("Segment length must be positive")
    if engine.global_config["cull_lanes_outside_map"]:
        raise ValueError("Box workaround does not support clipped lane polygons")
    world = engine.physics_world.static_world
    count = 0
    for node in list(world.getRigidBodies()):
        if not MetaDriveType.is_lane(node.getName()):
            continue
        lane = get_object_from_node(node)
        if not isinstance(lane, StraightLane):
            continue
        if not NodePath(node).getMat(engine.worldNP).isIdentity():
            raise ValueError("Transformed lane node is unsupported")
        if node.getNumShapes() != 1 or type(node.getShape(0)).__name__ != "BulletConvexHullShape":
            raise ValueError("Expected one original convex-hull shape")
        # Original hull vertices occupy z=-0.1..0; original margin is retained.
        margin = node.getShape(0).getMargin()
        world.removeRigidBody(node)
        node.removeShape(node.getShape(0))
        segments = math.ceil(lane.length / max_segment_length)
        length = lane.length / segments
        for index in range(segments):
            start = max(0, index * length - 0.01)
            end = min(lane.length, (index + 1) * length + 0.01)
            shape = BulletBoxShape(Vec3((end - start) / 2, lane.width / 2, 0.05))
            shape.setMargin(margin)
            centre = lane.position((start + end) / 2, 0)
            transform = TransformState.makePosHpr(Vec3(float(centre[0]), float(centre[1]), -0.05),
                                                Vec3(math.degrees(lane.heading), 0, 0))
            node.addShape(shape, transform)
        world.attachRigidBody(node)
        count += 1
    return count
