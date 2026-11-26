"""Isolated shape-query experiments. No MetaDrive import or dynamics stepping."""
import argparse
import importlib.metadata
import math
from pathlib import Path

from panda3d.bullet import BulletWorld,BulletRigidBodyNode,BulletBoxShape
from panda3d.core import NodePath,Point3,TransformState,Vec3

from aad_analysis.io import write_json
from experiments.geometry_math import box_vertices,convex_hull_xy
from .polygons import separation


CASES=[
    dict(name="separated",position=[2.2,0,0]),
    dict(name="near_contact",position=[2.001,0,0]),
    dict(name="touching",position=[2,0,0]),
    dict(name="slight_overlap",position=[1.999,0,0]),
    dict(name="corner_overlap",position=[1.99,1.99,0]),
    dict(name="rotated",position=[2.3,0,0],hpr=[45,0,0]),
    dict(name="nonzero_reference_offset",position=[2.3,0,0],offset_b=[-.4,0,0]),
    dict(name="different_shape_offsets",position=[2.3,0,0],offset_a=[.3,.1,.2],offset_b=[-.1,.1,.2]),
    dict(name="elevation_disjoint",position=[1,0,3]),
    dict(name="full_orientation",position=[2.2,.1,0],hpr=[20,15,10]),
]


def query(case,margin=.04,mass=1):
    world=BulletWorld();nodes=[];paths=[];polys=[];cores=[];metadata=[]
    try:
        for index in range(2):
            shape=BulletBoxShape(Vec3(1,1,.5));shape.setMargin(margin)
            body=BulletRigidBodyNode(f"isolated_{index}");body.setMass(mass)
            offset=case.get("offset_a" if index==0 else "offset_b",[0,0,0])
            body.addShape(shape,TransformState.makePos(Vec3(*offset)))
            node=NodePath(body)
            if index:
                node.setPos(*case["position"]);node.setHpr(*case.get("hpr",[0,0,0]))
            nodes.append(body);paths.append(node);world.attachRigidBody(body)
            def corners(half):
                return [list(node.getMat().xformPoint(body.getShapeMat(0).xformPoint(Point3(*p)))) for p in box_vertices(half)]
            outer=list(shape.getHalfExtentsWithMargin());core=list(shape.getHalfExtentsWithoutMargin())
            polys.append(convex_hull_xy(corners(outer)));cores.append(convex_hull_xy(corners(core)))
            metadata.append(dict(outer_half_extents=outer,core_half_extents=core,margin_m=shape.getMargin(),offset=offset))
        result=world.contactTestPair(*nodes)
        distances=[float(c.getManifoldPoint().getDistance()) for c in result.getContacts()]
        outer_result=separation(*polys);core_result=separation(*cores)
        if mass>0 and outer_result["distance"]>1e-4 and any(d<=0 for d in distances):
            raise AssertionError("Bullet contact outside a separated projected envelope")
        return dict(case=case,margin_m=margin,body_mass_kg=mass,shape_metadata=metadata,
            outer_envelope=outer_result,core_envelope=core_result,bullet_contact_records=len(distances),
            bullet_manifold_distances_m=distances,independent_positive_surface_distance_m=None,
            status="query_observations_only_no_dynamics_advance")
    finally:
        for node in nodes:world.removeRigidBody(node)


def study():
    samples=[query(case,margin) for margin in [0.,.04,.2] for case in CASES]
    by={(r["case"]["name"],r["margin_m"]):r for r in samples}
    for margin in [0.,.04,.2]:
        for name in ["separated","near_contact","elevation_disjoint"]:
            if by[name,margin]["bullet_contact_records"]!=0:raise AssertionError("Unexpected synthetic contact")
        for name in ["touching","slight_overlap","corner_overlap","rotated","nonzero_reference_offset","different_shape_offsets"]:
            if by[name,margin]["bullet_contact_records"]==0:raise AssertionError("Expected synthetic box contact missing")
    static_control=query(dict(name="static_static_filter_control",position=[1.9,0,0]),.04,mass=0)
    return dict(passed=True,panda3d_version=importlib.metadata.version("panda3d"),driving_scenarios_executed=0,
        dynamics_steps=0,cases=samples,static_filter_control=static_control,
        findings=["Retained NodePaths and nonstatic mass=1 bodies are required for meaningful contact pair tests here; two static bodies produce zero contact records even when overlapping",
            "BoxBox query contact classification matches sharp outer-envelope geometry in same-height supported tested cases across fixed-outer-size margin changes",
            "Core extents alone exclude actual margin contacts in these tests",
            "XY overlap with vertical separation does not establish Bullet contact",
            "contactTestPair provides contact manifold observations, not trustworthy positive separation for disjoint bodies",
            "Core-plus-margin nominal support is enclosed by with-margin extents under unit rigid transforms; sampled queries are not a proof about every dispatcher/tolerance/context",
            "No exact physical surface-distance or universal collision interpretation is adopted"],
        independent_positive_distance_available=False)


if __name__=="__main__":
    parser=argparse.ArgumentParser(__doc__);parser.add_argument("--output",type=Path,required=True);args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    report=study();args.output.mkdir(parents=True,exist_ok=False);write_json(args.output/"bullet_validation.json",report)
    print(f"PASS {len(report['cases'])} isolated box-query cases; no positive-distance oracle available")
