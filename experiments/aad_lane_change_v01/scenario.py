"""Scheduled steering target with IDM longitudinal response, not lane teleportation."""
from experiments.scenarios import ControlledTraffic as BaseTraffic
from experiments.scenarios import ObservedIDM as BaseIDM
from metadrive.component.vehicle.vehicle_type import DefaultVehicle
from metadrive.policy.idm_policy import IDMPolicy,FrontBackObjects


class ObservedIDM(BaseIDM):
    settings=None

    def lane_change_policy(self,objects):
        self.command_calls=getattr(self,'command_calls',0)+1
        lane=self.control_object.navigation.current_ref_lanes[
            self.settings['target_lane'] if self.command_calls>=self.settings['command_step'] else 1]
        nearby=FrontBackObjects.get_find_front_back_objs(objects,lane,self.control_object.position,
            self.MAX_LONG_DIST,self.control_object.navigation.current_ref_lanes)
        front=nearby.front_object() if nearby.has_front_object() else None
        distance=nearby.front_min_distance()
        self.decision=dict(target=list(lane.index),front_id=front.id if front else None,
            front_distance=distance,timer=int(self.overtake_timer))
        return front,distance,lane


class ControlledTraffic(BaseTraffic):
    def populate(self,settings):
        self.vehicles=[]
        specifications=[('leader',1,settings['leader_gap'],settings['leader_speed_kmh']),
            ('background',2,100,30),
            ('background_far',settings['target_lane'],settings['target_vehicle_gap_m'],settings['target_vehicle_speed_kmh'])]
        for role,lane,gap,speed in specifications:
            vehicle=self.spawn_object(DefaultVehicle,vehicle_config={
                'spawn_lane_index':('>>','>>>',lane),'spawn_longitude':settings['subject_longitude']+gap,'enable_reverse':False})
            self.add_policy(vehicle.id,IDMPolicy,vehicle,settings['seed'])
            policy=self.get_policy(vehicle.id);policy.NORMAL_SPEED=speed;policy.target_speed=speed;policy.enable_lane_change=False
            self.vehicles.append((role,vehicle))
