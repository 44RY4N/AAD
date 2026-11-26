from experiments.aad_lane_change_v01.scenario import ObservedIDM as ScheduledIDM
from experiments.aad_lane_change_v01.scenario import ControlledTraffic as OriginalTraffic


class ObservedIDM(ScheduledIDM):
    def act(self,*args,**kwargs):
        self.TIME_WANTED=self.settings.get('idm_time_wanted_s',self.TIME_WANTED)
        self.DISTANCE_WANTED=self.settings.get('idm_distance_wanted_m',self.DISTANCE_WANTED)
        action=super().act(*args,**kwargs)
        self.decision.update(command_step=self.settings['command_step'],steering_command=float(action[0]),
            longitudinal_command=float(action[1]),subject_speed_kmh=float(self.control_object.speed_km_h),
            target_speed_kmh=float(self.target_speed),idm_time_wanted_s=float(self.TIME_WANTED),
            idm_distance_wanted_m=float(self.DISTANCE_WANTED))
        return action


class ControlledTraffic(OriginalTraffic):
    def populate(self,settings):
        super().populate(settings)
        for role,vehicle in self.vehicles:
            if role=='background_far':
                policy=self.get_policy(vehicle.id)
                policy.TIME_WANTED=settings.get('target_idm_time_wanted_s',policy.TIME_WANTED)
