"""Read-only runtime instrumentation, geometry schema 0.2; no physics stepping."""
import hashlib
import importlib.metadata
import math
from pathlib import Path
import uuid

from panda3d.bullet import BulletBoxShape, BulletWorld, BulletRigidBodyNode
from panda3d.core import Point2, Point3, PerspectiveLens

from aad_analysis.io import write_json, write_jsonl
from experiments.geometry_math import box_vertices, convex_hull_xy, transform_point


def matrix(value):
    return [[float(value.getCell(i,j)) for j in range(4)] for i in range(4)]


def primitive(value):
    if value is None or isinstance(value,(str,bool,int,float)):
        return value
    if isinstance(value,dict):
        return {str(k):primitive(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):
        return [primitive(v) for v in value]
    if isinstance(value,type):
        return {"python_class":value.__module__+"."+value.__qualname__}
    if hasattr(value,"tolist"):
        return primitive(value.tolist())
    return {"unserialized_type":type(value).__module__+"."+type(value).__name__, "repr":repr(value)}


def shapes_for(vehicle):
    node=vehicle.chassis.node()
    result=[]
    for index in range(node.getNumShapes()):
        shape=node.getShape(index)
        transform=matrix(node.getShapeMat(index))
        record=dict(index=index,shape_type=type(shape).__module__+"."+type(shape).__name__,
                    shape_to_actor_matrix=transform,centre_offset_actor_m=transform_point([0,0,0],transform),
                    margin_m=float(shape.getMargin()),status="unavailable", reason="unsupported_collision_shape",
                    half_extents_with_margin_m=None,half_extents_without_margin_m=None,
                    shape_local_vertices_m=None,actor_local_vertices_m=None,actor_local_footprint_xy_m=None)
        if isinstance(shape,BulletBoxShape):
            half=list(shape.getHalfExtentsWithMargin())
            vertices=box_vertices(half)
            local=[transform_point(p,transform) for p in vertices]
            record.update(status="reconstructed_box_outer_envelope",reason="Box extents/transform are runtime APIs; margin contact semantics are not replaced by an exact sharp rectangle claim",
                half_extents_with_margin_m=half,half_extents_without_margin_m=list(shape.getHalfExtentsWithoutMargin()),
                shape_local_vertices_m=vertices,actor_local_vertices_m=local,
                actor_local_footprint_xy_m=convex_hull_xy(local))
        result.append(record)
    return result


def probe_box_faces(vehicle):
    """Ray-test the actual shared shape in an isolated, never-stepped BulletWorld."""
    result=[]
    original=vehicle.chassis.node()
    for i in range(original.getNumShapes()):
        shape=original.getShape(i)
        if not isinstance(shape,BulletBoxShape):
            result.append(dict(index=i,status="unavailable",reason="unsupported_shape_face_probe"));continue
        world=BulletWorld();body=BulletRigidBodyNode("read_only_geometry_probe")
        body.addShape(shape,original.getShapeTransform(i));world.attachRigidBody(body)
        half=list(shape.getHalfExtentsWithMargin());face_errors=[]
        try:
            for axis in range(3):
                for sign in [-1,1]:
                    outer=[0,0,0];inner=[0,0,0];surface=[0,0,0]
                    outer[axis]=sign*(half[axis]+1);surface[axis]=sign*half[axis]
                    mat=original.getShapeMat(i)
                    hit=world.rayTestClosest(mat.xformPoint(Point3(*outer)),mat.xformPoint(Point3(*inner)))
                    if not hit.hasHit():raise RuntimeError("Runtime box face probe missed")
                    expected=mat.xformPoint(Point3(*surface))
                    face_errors.append(max(abs(a-b) for a,b in zip(expected,hit.getHitPos())))
            result.append(dict(index=i,status="six_face_centre_rays_checked",max_error_m=max(face_errors),
                scope="Actual shared runtime shape in isolated BulletWorld, no simulation advance; does not establish corner contact/margin contour"))
        finally:world.removeRigidBody(body)
    return result


def static_actor(role,vehicle,world):
    node=vehicle.chassis.node()
    return dict(actor_id=vehicle.id,role=role,python_class=type(vehicle).__module__+"."+type(vehicle).__qualname__,
        evidence="runtime_recorded",resolved_dimensions_m=dict(length=float(vehicle.LENGTH),width=float(vehicle.WIDTH),height=float(vehicle.HEIGHT)),
        dimension_semantics="Runtime vehicle dimension properties; actual chassis primitives recorded independently",
        configured_model=primitive(vehicle.config.get("vehicle_model")),configured_visual_scale=primitive(vehicle.config.get("scale")),
        visual_asset_descriptor=primitive(getattr(vehicle,"path",None)),origin_scale_world=list(vehicle.origin.getScale(world)),
        reference=dict(definition="vehicle.origin is chassis NodePath; trajectory.position is its world XY",
                       origin_is_chassis=vehicle.origin==vehicle.chassis,
                       body_is_vehicle_system_chassis=node==vehicle.system.getChassis(),
                       local_axes="+X right, +Y forward, +Z up; heading_rad is world +X at zero",
                       offset_to_shape_centres="per-shape centre_offset_actor_m, not assumed zero"),
        body=dict(mass_kg=float(node.getMass()),collision_mask=int(node.getIntoCollideMask().getWord()),
                  kinematic=bool(node.isKinematic()),static=bool(node.isStatic()),
                  raycast_wheel_count=int(vehicle.system.getNumWheels())),
        collision_scope="Chassis rigid-body shapes only. BulletVehicle wheels are raycast suspension, not exported rigid wheel/visual footprints",
        shapes=shapes_for(vehicle),bullet_face_crosschecks=probe_box_faces(vehicle))


def road_geometry(env):
    graph=env.current_map.road_network.graph
    blocks=[]
    owners={}
    for block in env.current_map.blocks:
        blocks.append(dict(name=block.name,block_id=block.ID,python_class=type(block).__module__+"."+type(block).__name__))
        for f,tos in block.block_network.graph.items():
            for t,lanes in tos.items():
                for i,_ in enumerate(lanes):owners.setdefault((f,t,i),[]).append(block.name)
    records=[]
    for f,tos in graph.items():
        for t,lanes in tos.items():
            for i,lane in enumerate(lanes):
                if not getattr(lane,"need_lane_localization",True):
                    continue
                length=float(lane.length)
                count=max(1,math.ceil(length))
                ss=[length*k/count for k in range(count+1)]
                points=lambda lateral: [[float(v) for v in lane.position(s,lateral(s))] for s in ss]
                polygon=lane.polygon
                analytic={k:primitive(getattr(lane,k)) for k in ["start","end","width","heading","center","radius","start_phase","end_phase","direction"] if hasattr(lane,k)}
                successors=[]
                for nt,next_lanes in graph.get(t,{}).items():
                    for ni,next_lane in enumerate(next_lanes):
                        if lane.is_previous_lane_of(next_lane):successors.append([t,nt,ni])
                records.append(dict(lane_id=[f,t,i],road_id=[f,t],block_names=owners.get((f,t,i),[]),
                    python_class=type(lane).__module__+"."+type(lane).__name__,length_m=length,
                    centreline=dict(status="sampled_runtime_lane_position_api",s_m=ss,xy_m=points(lambda s:0),max_spacing_m=length/count),
                    width_samples_m=[float(lane.width_at(s)) for s in ss],
                    boundaries=dict(status="reconstructed_from_runtime_position_and_width_apis",positive_lateral_xy_m=points(lambda s:lane.width_at(s)/2),negative_lateral_xy_m=points(lambda s:-lane.width_at(s)/2)),
                    polygon=dict(status="runtime_lane_polygon_representation",xy_m=primitive(polygon)),
                    analytic_parameters=analytic,travel_heading_samples_rad=[float(lane.heading_theta_at(s)) for s in ss],
                    successor_lane_ids=successors,connectivity_method="runtime graph adjacency filtered by lane.is_previous_lane_of"))
    return dict(schema_version="0.2",coordinate_frame="world XY metres",blocks=blocks,lanes=records,
                conflict_zones=None,limitations="Sampled geometry is not exact continuous curved-lane boundaries; no claim of curved-map localization validation")


def camera_calibration(camera):
    if camera is None:
        return dict(status="unavailable",reason="RGB sensor not enabled",intrinsics=None)
    lens=camera.lens
    result=dict(status="runtime_lens_parameters",lens_class=type(lens).__module__+"."+type(lens).__name__,
        width=camera.BUFFER_W,height=camera.BUFFER_H,fov_degrees=list(lens.getFov()),near_m=float(lens.getNear()),far_m=float(lens.getFar()),
        film_size=list(lens.getFilmSize()),film_offset=list(lens.getFilmOffset()),focal_length_lens_units=float(lens.getFocalLength()),
        coordinate_system_code=int(lens.getCoordinateSystem()),projection_matrix=matrix(lens.getProjectionMat()),intrinsics=None,
        camera_axes="Panda3D +X right,+Y forward,+Z up; CV coordinates=(X,-Z,Y)",
        distortion=dict(status="not_parameterized_by_this_PerspectiveLens_model",coefficients=None,
                        note="Not a real-world calibration or a measured distortion estimate"))
    if isinstance(lens,PerspectiveLens):
        def pixel(p):
            uv=Point2();lens.project(Point3(*p),uv)
            return [(uv.x+1)*camera.BUFFER_W/2,(1-uv.y)*camera.BUFFER_H/2]
        centre=pixel([0,1,0]);right=pixel([1,1,0]);up=pixel([0,1,1])
        fx,fy=right[0]-centre[0],centre[1]-up[1]
        error=0.
        for x,y,z in [(-.4,2,.3),(.5,3,-.2),(.1,5,.6)]:
            expected=[centre[0]+fx*x/y,centre[1]-fy*z/y]
            error=max(error,max(abs(a-b) for a,b in zip(expected,pixel([x,y,z]))))
        if error<=1e-3:
            result["intrinsics"]=dict(status="derived_and_crosschecked_against_runtime_lens_project",fx_px=fx,fy_px=fy,cx_px=centre[0],cy_px=centre[1],
                matrix_cv=[[fx,0,centre[0]],[0,fy,centre[1]],[0,0,1]],max_project_error_px=error,
                pixel_convention="continuous image edges, same formula as RGB centre_pixel; principal centre may be W/2,H/2")
    return result


class GeometryRecorder:
    def __init__(self,env,output,vehicles,settings,dt,camera=None):
        self.env,self.vehicles,self.dt,self.camera=env,vehicles,dt,camera
        initial=self.signature();initial_step=env.engine.episode_step
        self.output=output/"geometry";self.output.mkdir(exist_ok=False)
        self.records=[];self.actor_info=[static_actor(role,v,env.engine.worldNP) for role,v in vehicles]
        self.initial_shapes={v.id:shapes_for(v) for _,v in vehicles}
        repo=Path(__file__).resolve().parents[1]
        sources=["experiments/run.py","experiments/geometry_recording.py","experiments/geometry_math.py",
                 "experiments/recording.py","experiments/scenarios.py","experiments/lane_geometry.py",
                 "metadrive/component/vehicle/base_vehicle.py","metadrive/component/vehicle/vehicle_type.py",
                 "metadrive/base_class/base_object.py","metadrive/component/lane/straight_lane.py",
                 "metadrive/engine/core/image_buffer.py"]
        self.manifest=dict(schema_version="0.2",run_id="geometry-"+str(uuid.uuid4()),
            provenance=dict(evidence="runtime_recorded_for_this_capture",settings=settings,
                software={k:importlib.metadata.version(k) for k in ["metadrive-simulator","panda3d"]},
                source_files={p:hashlib.sha256((repo/p).read_bytes()).hexdigest() for p in sources}),
            dt_s=dt,decision_steps=settings["steps"],initial_state_included=True,
            units=dict(length="m",time="s",velocity="m/s",heading="rad",hpr="degrees",angular_velocity="rad/s"),
            matrices="Panda3D row-major serialization, row vectors: p_world=[x,y,z,1] * actor_to_world",
            footprint_contract="Separate per-shape XY convex hulls of all 8 transformed 3D box-envelope vertices; retain shape margins/core extents; not an exact rounded-margin contact contour or a merged multi-shape union",
            wheel_scope="Raycast wheels not separate rigid collision footprints",
            rgb_enabled=camera is not None,capture_advances_physics=False,
            unsupported_geometry_policy="Null vertices/footprints with reason; never substitute visual bounding boxes")
        write_json(self.output/"actor_geometry.json",dict(schema_version="0.2",actors=self.actor_info))
        write_json(self.output/"road_geometry.json",road_geometry(env))
        write_json(self.output/"camera_calibration.json",camera_calibration(camera))
        if initial!=self.signature() or initial_step!=env.engine.episode_step:
            raise RuntimeError("Geometry initialization changed simulation state")

    def signature(self):
        return [(v.id,matrix(v.origin.getMat(self.env.engine.worldNP)),list(v.chassis.node().getLinearVelocity()),
                 list(v.chassis.node().getAngularVelocity()),float(v.steering),float(v.throttle_brake),
                 self.env.engine.get_policy(v.id).overtake_timer) for _,v in self.vehicles]

    def capture(self,step):
        before=self.signature();counter=self.env.engine.episode_step
        actors=[];world=self.env.engine.worldNP
        for role,v in self.vehicles:
            node=v.chassis.node();pose=matrix(v.origin.getMat(world))
            if shapes_for(v)!=self.initial_shapes[v.id]:raise RuntimeError("Static actor shape changed during capture")
            shapes=[]
            scale=list(v.origin.getScale(world))
            for record in self.initial_shapes[v.id]:
                component=dict(index=record["index"],status=record["status"],reason=record["reason"],world_vertices_m=None,world_footprint_xy_m=None,bullet_transform_crosscheck_max_error_m=None)
                if record["actor_local_vertices_m"] is not None and max(abs(s-1) for s in scale)<1e-6:
                    vertices=[transform_point(p,pose) for p in record["actor_local_vertices_m"]]
                    native=[list(v.origin.getMat(world).xformPoint(node.getShapeMat(record["index"]).xformPoint(Point3(*p)))) for p in record["shape_local_vertices_m"]]
                    error=max(abs(a-b) for p,q in zip(vertices,native) for a,b in zip(p,q))
                    if error>1e-4:raise RuntimeError("World vertices disagree with Bullet/Panda3D shape transforms")
                    component.update(world_vertices_m=vertices,world_footprint_xy_m=convex_hull_xy(vertices),bullet_transform_crosscheck_max_error_m=error)
                elif record["actor_local_vertices_m"] is not None:
                    component.update(status="unavailable",reason="nonunit_origin_scaling_not_validated")
                shapes.append(component)
            q=v.origin.getQuat(world)
            actors.append(dict(actor_id=v.id,role=role,position_world_m=list(v.origin.getPos(world)),
                heading_rad=float(v.heading_theta),hpr_world_degrees=list(v.origin.getHpr(world)),
                quaternion_wxyz=[q.getR(),q.getI(),q.getJ(),q.getK()],actor_to_world_matrix=pose,
                velocity_world_m_s=list(node.getLinearVelocity()),velocity_source="runtime_Bullet_chassis_getLinearVelocity",
                angular_velocity_world_rad_s=list(node.getAngularVelocity()),speed_kmh=float(v.speed_km_h),
                speed_convention="horizontal sqrt(vx^2+vy^2)*3.6, excludes vz",footprint_components=shapes))
        camera=None
        if self.camera:
            c=self.camera.cam
            camera=dict(camera_to_world_matrix=matrix(c.getMat(world)),world_to_camera_matrix=matrix(world.getMat(c)),
                        world_position_m=list(c.getPos(world)),world_hpr_degrees=list(c.getHpr(world)))
        if before!=self.signature() or counter!=self.env.engine.episode_step:
            raise RuntimeError("Geometry capture changed simulation state")
        self.records.append(dict(step=step,time_s=step*self.dt,actors=actors,camera=camera,read_only_guard_passed=True))

    def finish(self,completed):
        write_jsonl(self.output/"geometry_frames.jsonl",self.records)
        self.manifest.update(completed=completed,samples=len(self.records),all_capture_guards_passed=all(r["read_only_guard_passed"] for r in self.records))
        write_json(self.output/"geometry_manifest.json",self.manifest)
