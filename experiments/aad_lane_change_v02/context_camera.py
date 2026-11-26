"""Explicit opt-in camera pose variant; same sensor and capture/guard implementation."""
import json
from panda3d.core import Point3
from experiments.recording import RGBRecorder


class ContextRGBRecorder(RGBRecorder):
    def __init__(self,env,output,settings):
        super().__init__(env,output)
        self.mount=settings['camera_mount'];self.look=settings['camera_look_at']
        self.camera.track(env.agent.origin,tuple(self.mount),(0,0,0))
        self.camera.cam.lookAt(env.agent.origin,Point3(*self.look))

    def finish(self,event=None):
        super().finish(event)
        path=self.output/'camera.json'
        data=json.loads(path.read_text());data.update(mount_position=self.mount,look_at_local=self.look,
            view_variant='explicit_rear_context_pose_same_RGB_sensor',baseline_view_unchanged=False)
        path.write_text(json.dumps(data,indent=2),encoding='utf-8')
