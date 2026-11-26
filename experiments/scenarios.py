from metadrive.component.vehicle.vehicle_type import DefaultVehicle
from metadrive.manager.base_manager import BaseManager
from metadrive.policy.idm_policy import IDMPolicy, FrontBackObjects


class ObservedIDM(IDMPolicy):
    """Observe the unmodified IDM decision; allow errors to surface in validation."""
    def lane_change_policy(self, objects):
        result = super().lane_change_policy(objects)
        nearby = FrontBackObjects.get_find_front_back_objs(
            objects, self.routing_target_lane, self.control_object.position,
            self.MAX_LONG_DIST, self.control_object.navigation.current_ref_lanes)
        self.decision = {
            "target": list(result[2].index),
            "front_id": nearby.front_object().id if nearby.has_front_object() else None,
            "front_distance": nearby.front_min_distance(),
            "timer": int(self.overtake_timer),
        }
        return result

    def act(self, *args, **kwargs):
        # Same control sequence as 0.4.3, without its broad exception fallback.
        if not self.move_to_next_road():
            raise RuntimeError("Subject left the controlled road")
        objects = self.control_object.lidar.get_surrounding_objects(self.control_object)
        front, distance, lane = self.lane_change_policy(objects)
        return [self.steering_control(lane), self.acceleration(front, distance)]


class ControlledTraffic(BaseManager):
    """Public manager lifecycle and spawning APIs; no PGTrafficManager internals."""
    def populate(self, settings):
        self.vehicles = []
        for role, lane, longitude, speed in [
            ("leader", 1, settings["subject_longitude"] + settings["leader_gap"], settings["leader_speed_kmh"]),
            ("background", 2, settings["subject_longitude"] + 100, 30),
            ("background_far", 0, settings["subject_longitude"] + 180, 30),
        ]:
            vehicle = self.spawn_object(DefaultVehicle, vehicle_config={
                "spawn_lane_index": (">>", ">>>", lane),
                "spawn_longitude": longitude, "enable_reverse": False})
            self.add_policy(vehicle.id, IDMPolicy, vehicle, settings["seed"])
            policy = self.get_policy(vehicle.id)
            policy.NORMAL_SPEED = speed
            policy.target_speed = speed
            policy.enable_lane_change = False
            self.vehicles.append((role, vehicle))

    def before_step(self):
        for _, vehicle in self.vehicles:
            vehicle.before_step(self.get_policy(vehicle.id).act())
        return {}

    def after_step(self):
        for _, vehicle in self.vehicles:
            vehicle.after_step()
        return {}


