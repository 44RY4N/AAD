"""Explicit proposed abort controller; trigger settings are not safety thresholds."""
from experiments.aad_overtake_v01.scenario import ObservedIDM as PassPolicy, world_interval
from metadrive.policy.idm_policy import FrontBackObjects


def abort_due(elapsed, budget, returning=False, negative_control=False):
    if type(budget) is not int or budget<1:raise ValueError('Positive attempt budget required')
    return elapsed>=budget and not returning and not negative_control


class AbortIDM(PassPolicy):
    def lane_change_policy(self, objects):
        # Preserve the normal pass policy until a configured elapsed-attempt budget
        # expires before full pass. This is controller intent, not inferred intent.
        normal = super().lane_change_policy(objects)
        subject = self.control_object
        leader = next(v for role, v in self.engine.managers['controlled_traffic'].vehicles if role == 'leader')
        elapsed = self.command_calls-self.settings['command_step']
        if (not getattr(self, 'abort_trigger', None) and abort_due(elapsed,
                self.settings['abort_attempt_budget_steps'],getattr(self,'returning',False),self.settings.get('negative_control',False))):
            self.abort_trigger = dict(step=self.command_calls, reason='configured_attempt_time_budget_expired_before_full_pass',
                elapsed_steps=elapsed, threshold_status='experimental_controller_setting_not_validated_safety_limit')
        if not getattr(self, 'abort_trigger', None):
            return normal
        gap = world_interval(leader)[0]-world_interval(subject)[1]
        if gap >= self.settings['abort_return_gap_m']:
            self.abort_returning = True
        returning = getattr(self, 'abort_returning', False)
        lane = subject.navigation.current_ref_lanes[1 if returning else 0]
        self.target_speed = self.settings['subject_speed_kmh'] if returning else self.settings['abort_wait_speed_kmh']
        self.NORMAL_SPEED = self.target_speed
        nearby = FrontBackObjects.get_find_front_back_objs(objects, lane, subject.position,
                    self.MAX_LONG_DIST, subject.navigation.current_ref_lanes)
        front = nearby.front_object() if nearby.has_front_object() else None
        self.decision.update(phase='abort_return_commanded' if returning else 'abort_slowing_in_passing_lane',
            abort_trigger=self.abort_trigger, source_following_envelope_gap_m=gap,
            target=list(lane.index), front_id=front.id if front else None,
            front_distance=nearby.front_min_distance())
        return front, nearby.front_min_distance(), lane
