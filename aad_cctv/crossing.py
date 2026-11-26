"""Controlled perpendicular vehicle crossing on MetaDrive's generated X block."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from metadrive import MetaDriveEnv
from metadrive.component.map.pg_map import MapGenerateMethod
from metadrive.component.vehicle.vehicle_type import DefaultVehicle
from metadrive.policy.idm_policy import IDMPolicy, FrontBackObjects
from experiments.scenarios import ControlledTraffic as BaseTraffic
from experiments.geometry_recording import GeometryRecorder
from experiments.lane_geometry import replace_straight_lane_hulls
from .camera import FixedCCTVRecorder, sensor_config, write


class CrossingIDM(IDMPolicy):
    """Route-following steering and IDM; no broad exception-to-zero fallback."""
    def act(self, *args, **kwargs):
        if not self.move_to_next_road():
            raise RuntimeError('Crossing policy has no supported route')
        objects=self.control_object.lidar.get_surrounding_objects(self.control_object)
        nearby=FrontBackObjects.get_find_front_back_objs(objects,self.routing_target_lane,
            self.control_object.position,self.MAX_LONG_DIST,self.control_object.navigation.current_ref_lanes)
        front=nearby.front_object() if nearby.has_front_object() else None
        action=[self.steering_control(self.routing_target_lane),self.acceleration(front,nearby.front_min_distance())]
        self.decision=dict(target=list(self.routing_target_lane.index),front_id=front.id if front else None,
            front_distance=nearby.front_min_distance(),steering_command=float(action[0]),longitudinal_command=float(action[1]))
        return action


class CrossingTraffic(BaseTraffic):
    def populate(self, settings):
        vehicle=self.spawn_object(DefaultVehicle,vehicle_config={
            'spawn_lane_index':('-1X0_1_','-1X0_0_',0),'spawn_longitude':settings['target_longitude'],
            'destination':'1X2_1_','enable_reverse':False})
        self.add_policy(vehicle.id,CrossingIDM,vehicle,settings['seed'])
        policy=self.get_policy(vehicle.id)
        policy.NORMAL_SPEED=policy.target_speed=settings['target_speed_kmh'];policy.enable_lane_change=False
        self.vehicles=[('crossing_target',vehicle)]


def run(settings, cameras, output, rgb=True):
    for key in ['subject_speed_kmh','target_speed_kmh']:
        if settings[key]<=0:raise ValueError('Positive IDM desired speeds required')
    output.mkdir(parents=True,exist_ok=False)
    config=dict(map_config={'type':MapGenerateMethod.BIG_BLOCK_SEQUENCE,'config':'X',
        'lane_num':2,'lane_width':3.5,'exit_length':80},traffic_density=0,traffic_mode='basic',
        use_render=False,random_spawn_lane_index=False,num_scenarios=1,start_seed=settings['seed'],
        horizon=settings['steps']+1,agent_policy=CrossingIDM,
        agent_configs={'default_agent':{'spawn_lane_index':('>>','>>>',0),
        'spawn_longitude':settings['subject_longitude'],'destination':'1X1_1_'}})
    if rgb:config.update(sensor_config(cameras))
    env=MetaDriveEnv(config);rows=[];decisions=[];recorder=geometry=None;complete=False;termination=None
    try:
        env.reset(seed=settings['seed'])
        replaced=replace_straight_lane_hulls(env.engine)
        manager=CrossingTraffic();env.engine.register_manager('controlled_traffic',manager);manager.populate(settings)
        subject=env.agent;source=subject.lane
        policy=env.engine.get_policy(subject.id);policy.NORMAL_SPEED=policy.target_speed=settings['subject_speed_kmh']
        policy.enable_lane_change=False
        vehicles=[('subject',subject)]+manager.vehicles
        dt=env.config['physics_world_step_size']*env.config['decision_repeat']
        # Derived from the two actual straight route lanes, not an invented intersection.
        net=env.current_map.road_network
        subject_path=net.get_lane(('>>>','1X1_0_',0));target_path=net.get_lane(('-1X0_0_','1X2_0_',0))
        from shapely.geometry import Polygon
        def corridor(lane):
            return Polygon([lane.position(s,l).tolist() for s,l in
                [(0,-lane.width/2),(lane.length,-lane.width/2),(lane.length,lane.width/2),(0,lane.width/2)]])
        zone=corridor(subject_path).intersection(corridor(target_path))
        if zone.is_empty or zone.geom_type!='Polygon':raise RuntimeError('No supported path conflict zone')
        write(output/'conflict_zone.json',dict(polygon_world_xy_m=list(zone.exterior.coords)[:-1],
            derivation='Intersection of selected straight-lane corridor polygons; not a legal-priority definition',
            subject_lane=list(subject_path.index),target_lane=list(target_path.index)))
        if rgb:recorder=FixedCCTVRecorder(env,output,cameras)
        geometry=GeometryRecorder(env,output,vehicles,settings,dt,camera=None)
        for step in range(settings['steps']+1):
            frame=[]
            for role,v in vehicles:
                s,l=source.local_coordinates(v.position)
                frame.append(dict(step=step,time_s=step*dt,role=role,vehicle_id=v.id,x=float(v.position[0]),y=float(v.position[1]),
                    heading_rad=float(v.heading_theta),speed_kmh=float(v.speed_km_h),on_lane=bool(v.on_lane),
                    white_line=bool(v.on_white_continuous_line),yellow_line=bool(v.on_yellow_continuous_line),sidewalk=bool(v.crash_sidewalk),
                    lane=str(v.lane_index),source_longitudinal_m=float(s),source_lateral_m=float(l),target_lateral_m=0.,
                    same_road=v.lane_index[:2]==source.index[:2],crash=bool(v.crash_vehicle)))
            rows.extend(frame)
            if recorder:recorder.capture(frame,vehicles)
            geometry.capture(step)
            if termination:
                write(output/'termination.json',termination);break
            if step==settings['steps']:
                complete=True;break
            _,_,terminated,truncated,info=env.step([0,0])
            decisions.append(dict(step=step+1,subject=policy.decision,
                target=env.engine.get_policy(manager.vehicles[0][1].id).decision))
            if terminated or truncated:
                termination=dict(step=step+1,info=repr(info),vehicle_contact=bool(subject.crash_vehicle))
            # Background contacts also terminate this new scenario; no contact is hidden.
            if any(v.crash_vehicle or v.crash_object or v.crash_sidewalk for _,v in vehicles):
                termination=dict(step=step+1,info='contact flag on captured actor',vehicle_contact=any(v.crash_vehicle for _,v in vehicles))
            if any(not v.on_lane for _,v in vehicles):
                termination=dict(step=step+1,info='captured actor lost lane localization',vehicle_contact=any(v.crash_vehicle for _,v in vehicles))
        numeric=[{k:v for k,v in row.items() if k!='vehicle_id'} for row in rows]
        write(output/'metadata.json',dict(settings=settings,dt_s=dt,source_lane=list(source.index),
            target_lane=list(target_path.index),source_start=source.start.tolist(),source_end=source.end.tolist(),
            replaced_lane_shapes=replaced,rgb_recording=rgb,completed=complete,
            result=dict(verified=False,scope='Not a lane-change assertion',trajectory_sha256_without_ids=hashlib.sha256(json.dumps(numeric,sort_keys=True).encode()).hexdigest())))
        write(output/'decisions.json',decisions)
    finally:
        if geometry:geometry.finish(complete)
        if recorder:recorder.finish()
        if rows:
            with (output/'trajectory.csv').open('w',newline='',encoding='utf-8') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        write(output/'cctv_execution.json',dict(family='crossing',settings=settings,cameras=cameras if rgb else [],
            completed=complete,source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')}))
        env.close()
    if not complete:raise RuntimeError('Crossing ended naturally before requested horizon: '+repr(termination))


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__)
    for key in ['config','cameras','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--headless',action='store_true');a=p.parse_args()
    run(json.loads(a.config.read_text()),json.loads(a.cameras.read_text()),a.output,not a.headless)
