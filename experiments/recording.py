"""RGB capture at decision boundaries without advancing simulation time."""
import json

import numpy as np
from PIL import Image
from panda3d.core import Point2, Point3
from metadrive.component.sensors.rgb_camera import RGBCamera


def camera_config():
    # Reuse test_rgb_camera.py's sensor, resolution and observation configuration.
    return dict(image_observation=True, sensors={"rgb_camera": (RGBCamera, 640, 360)},
                vehicle_config={"image_source": "rgb_camera"})


class RGBRecorder:
    def __init__(self, env, output):
        self.env = env
        self.output = output / "camera"
        self.frames = self.output / "frames"
        self.frames.mkdir(parents=True, exist_ok=False)
        self.camera = env.engine.get_sensor("rgb_camera")
        # Vehicle model local +y is forward. Sensor settings are unchanged;
        # this observational mount includes the subject rather than its bonnet.
        self.camera.track(env.agent.origin, (0, -22, 16), (0, 0, 0))
        self.camera.cam.lookAt(env.agent.origin, Point3(0, 15, 0))
        self.records = []

    def capture(self, state_rows, vehicles):
        before = [(tuple(v.position), v.heading_theta, v.speed_km_h, v.lane_index) for _, v in vehicles]
        step = state_rows[0]["step"]
        # Flush render buffers at the current transforms, with no taskMgr.step,
        # env.step, policy invocation, or physics warm-up step.
        self.env.engine.graphicsEngine.renderFrame()
        self.env.engine.graphicsEngine.renderFrame()
        self.env.engine.graphicsEngine.syncFrame()
        bgr = np.asarray(self.camera.perceive(to_float=False))
        if bgr.shape != (360, 640, 3) or bgr.dtype != np.uint8:
            raise RuntimeError(f"Unexpected camera image: {bgr.shape}, {bgr.dtype}")
        rgb = bgr[..., ::-1].copy()
        # Confirm channel order against Panda3D's explicitly requested RGB bytes.
        texture = self.camera.buffer.getDisplayRegion(1).getScreenshot()
        explicit_rgb = np.frombuffer(texture.getRamImageAs("RGB"), dtype=np.uint8).reshape(360, 640, 3)[::-1]
        if not np.array_equal(rgb, explicit_rgb):
            raise RuntimeError("Sensor channel order does not match explicit RGB readback")
        after = [(tuple(v.position), v.heading_theta, v.speed_km_h, v.lane_index) for _, v in vehicles]
        if before != after:
            raise RuntimeError("Rendering changed vehicle state")
        filename = f"frames/{step:06d}.png"
        Image.fromarray(rgb).save(self.output / filename)
        projections = []
        for role, vehicle in vehicles:
            point = self.camera.cam.getRelativePoint(vehicle.origin, Point3(0, 0, 0.7))
            uv = Point2()
            in_view = bool(self.camera.lens.project(point, uv))
            projections.append(dict(role=role, vehicle_id=vehicle.id, in_frustum=in_view,
                centre_pixel=[float((uv.x + 1) * 320), float((1 - uv.y) * 180)] if in_view else None))
        self.records.append(dict(step=step, time_s=state_rows[0]["time_s"], file=filename,
            subject=dict(state_rows[0]), traffic=[dict(r) for r in state_rows[1:]],
            camera_world_position=list(self.camera.cam.getPos(self.env.engine.worldNP)),
            camera_world_hpr=list(self.camera.cam.getHpr(self.env.engine.worldNP)),
            projections=projections))

    def finish(self, event=None):
        for record in self.records:
            step = record["step"]
            state = "unverified"
            if event and event.get("verified"):
                if step < event["departure_step"]:
                    state = "lane_following"
                elif step < event["settled_step"]:
                    state = "lane_changing"
                elif step < event["confirmed_step"]:
                    state = "target_tolerance_pending_confirmation"
                else:
                    state = "confirmed_target_lane"
            record["lane_change_state"] = state
        with (self.output / "frames.jsonl").open("w", encoding="utf-8") as file:
            for record in self.records:
                file.write(json.dumps(record) + "\n")
        (self.output / "camera.json").write_text(json.dumps(dict(
            width=640, height=360, format="RGB PNG", fps_simulation=10,
            initial_state_included=True, render_frames_per_sample=2,
            mount_position=[0, -22, 16], look_at_local=[0, 15, 0],
            labels="Retrospective scenario phases from physical event verification; not criticality labels",
            frames=len(self.records), channel_order_checked=True,
            capture_state_unchanged_checked=True), indent=2), encoding="utf-8")
