from experiments.aad_lane_change_v02.scenario import ObservedIDM as ParentPolicy,ControlledTraffic
from metadrive.policy.idm_policy import FrontBackObjects
from experiments.geometry_recording import shapes_for,matrix
from experiments.geometry_math import transform_point


def world_interval(vehicle):
    points=[transform_point(p,matrix(vehicle.origin.getMat(vehicle.engine.worldNP)))
            for shape in shapes_for(vehicle) for p in shape['actor_local_vertices_m']]
    return min(p[0] for p in points),max(p[0] for p in points)


class ObservedIDM(ParentPolicy):
    def lane_change_policy(self,objects):
        self.command_calls=getattr(self,'command_calls',0)+1
        settings=self.settings;subject=self.control_object
        leader=next(v for role,v in self.engine.managers['controlled_traffic'].vehicles if role=='leader')
        gap=world_interval(subject)[0]-world_interval(leader)[1]
        lanes=subject.navigation.current_ref_lanes
        if (self.command_calls>=settings['command_step'] and
                abs(lanes[0].local_coordinates(subject.position)[1])<.35 and gap>=settings['return_gap_m']):
            self.returning=True
        idx=1 if settings.get('negative_control') or getattr(self,'returning',False) or self.command_calls<settings['command_step'] else 0
        lane=lanes[idx]
        nearby=FrontBackObjects.get_find_front_back_objs(objects,lane,subject.position,self.MAX_LONG_DIST,lanes)
        front=nearby.front_object() if nearby.has_front_object() else None
        self.decision=dict(target=list(lane.index),front_id=front.id if front else None,
            front_distance=nearby.front_min_distance(),timer=int(self.overtake_timer),
            phase='return_commanded' if getattr(self,'returning',False) else 'passing_commanded' if idx==0 else 'following',
            subject_rear_minus_leader_front_m=gap)
        return front,nearby.front_min_distance(),lane
