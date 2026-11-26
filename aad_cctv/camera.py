"""World-parented RGB cameras. No actor-following transforms or physics steps."""
import hashlib
import json
import math

import numpy as np


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def validate_cameras(configs, dt):
    if not configs or len({c['id'] for c in configs}) != len(configs):
        raise ValueError('Nonempty unique camera IDs required')
    for c in configs:
        if set(c)!={'id','position_world_m','look_at_world_m','width','height','horizontal_fov_deg','fps'}:
            raise ValueError('Camera settings must specify fixed world coordinates only')
        if not c['id'].replace('_', '').isalnum():
            raise ValueError('Camera ID must be alphanumeric/underscore')
        for key in ['position_world_m', 'look_at_world_m']:
            if len(c[key]) != 3 or not all(math.isfinite(x) for x in c[key]):
                raise ValueError('Finite world-space camera vectors required')
        if c['position_world_m'] == c['look_at_world_m']:
            raise ValueError('Camera and aim point must differ')
        if c['position_world_m'][2] <= 0:
            raise ValueError('Elevated external camera required')
        if not 1 < c['horizontal_fov_deg'] < 170:
            raise ValueError('Unsupported field of view')
        if any(type(c[k]) is not int or c[k] < 32 for k in ['width', 'height']):
            raise ValueError('Positive image dimensions required')
        fps = c['fps']
        if not math.isfinite(fps) or fps <= 0 or fps > 1/dt + 1e-8:
            raise ValueError('FPS must be positive and no greater than decision frequency')
        stride = 1/(fps*dt)
        if abs(stride-round(stride)) > 1e-8:
            raise ValueError('FPS must divide the simulation decision frequency exactly')


def sensor_config(configs):
    from metadrive.component.sensors.rgb_camera import RGBCamera

    return dict(image_observation=True,
                sensors={c['id']: (RGBCamera, c['width'], c['height']) for c in configs},
                vehicle_config={'image_source': configs[0]['id']})


class FixedCCTVRecorder:
    def __init__(self, env, output, configs):
        from panda3d.core import Point3
        from experiments.geometry_recording import camera_calibration, matrix

        self.env, self.output, self.configs = env, output, configs
        self.dt = env.config['physics_world_step_size']*env.config['decision_repeat']
        validate_cameras(configs, self.dt)
        self.views = []
        for i, config in enumerate(configs):
            folder = output/'camera' if i == 0 else output/'cameras'/config['id']
            (folder/'frames').mkdir(parents=True, exist_ok=False)
            sensor = env.engine.get_sensor(config['id'])
            # Intentionally parent to static world, never a vehicle or actor origin.
            sensor.cam.reparentTo(env.engine.worldNP)
            sensor.cam.setPos(*config['position_world_m'])
            sensor.cam.lookAt(Point3(*config['look_at_world_m']))
            sensor.lens.setAspectRatio(config['width']/config['height'])
            sensor.lens.setFov(config['horizontal_fov_deg'])
            pose = matrix(sensor.cam.getMat(env.engine.worldNP))
            calibration = camera_calibration(sensor)
            calibration.update(camera_id=config['id'], viewpoint='fixed_external_CCTV',
                camera_to_world_matrix=pose,
                world_to_camera_matrix=matrix(env.engine.worldNP.getMat(sensor.cam)),
                ground_plane_world_z_m=0.0,
                calibration_scope='Actual simulated lens and world transform; not physical CCTV calibration')
            write(folder/'calibration.json', calibration)
            self.views.append(dict(config=config, sensor=sensor, folder=folder, pose=pose,
                stride=round(1/(config['fps']*self.dt)), records=[], observations=[]))
        self.camera = self.views[0]['sensor']

    def capture(self, state_rows, vehicles):
        from PIL import Image
        from panda3d.core import Point2, Point3
        from experiments.geometry_recording import matrix

        step, time = state_rows[0]['step'], state_rows[0]['time_s']
        signature = lambda: [(v.id, matrix(v.origin.getMat(self.env.engine.worldNP)),
            list(v.chassis.node().getLinearVelocity()), v.lane_index,
            self.env.engine.get_policy(v.id).overtake_timer) for _, v in vehicles]
        before, counter = signature(), self.env.engine.episode_step
        for view in self.views:
            sensor = view['sensor']
            if sensor.cam.getParent() != self.env.engine.worldNP or matrix(sensor.cam.getMat(self.env.engine.worldNP)) != view['pose']:
                raise RuntimeError('Fixed camera transform/parent changed')
        due = [v for v in self.views if step % v['stride'] == 0]
        if not due:
            return
        self.env.engine.graphicsEngine.renderFrame()
        self.env.engine.graphicsEngine.renderFrame()
        self.env.engine.graphicsEngine.syncFrame()
        for view in due:
            c, sensor, folder = view['config'], view['sensor'], view['folder']
            bgr = np.asarray(sensor.perceive(to_float=False))
            expected = (c['height'], c['width'], 3)
            if bgr.shape != expected or bgr.dtype != np.uint8:
                raise RuntimeError('Unexpected sensor image')
            rgb = bgr[..., ::-1].copy()
            texture = sensor.buffer.getDisplayRegion(1).getScreenshot()
            explicit = np.frombuffer(texture.getRamImageAs('RGB'), dtype=np.uint8).reshape(expected)[::-1]
            if not np.array_equal(rgb, explicit):
                raise RuntimeError('RGB channel-order mismatch')
            name = f'frames/{step:06d}.png'
            Image.fromarray(rgb).save(folder/name)
            observation = dict(camera_id=c['id'], step=step, time_s=time, file=name,
                               image_sha256=hashlib.sha256((folder/name).read_bytes()).hexdigest())
            # This is the sole image-only inference input manifest. No actors/state.
            view['observations'].append(observation)
            projections = []
            for role, vehicle in vehicles:
                point = sensor.cam.getRelativePoint(vehicle.origin, Point3(0, 0, .7))
                uv = Point2(); visible = bool(sensor.lens.project(point, uv))
                projections.append(dict(role=role, vehicle_id=vehicle.id, in_frustum=visible,
                    centre_pixel=[(float(uv.x)+1)*c['width']/2, (1-float(uv.y))*c['height']/2] if visible else None))
            # Evaluation-only synchronized state, deliberately in a different file.
            view['records'].append(dict(**observation, subject=dict(state_rows[0]),
                traffic=[dict(r) for r in state_rows[1:]], projections=projections,
                camera_world_position=list(sensor.cam.getPos(self.env.engine.worldNP)),
                camera_world_hpr=list(sensor.cam.getHpr(self.env.engine.worldNP)),
                camera_to_world_matrix=view['pose'], reference_only=True))
        if before != signature() or counter != self.env.engine.episode_step:
            raise RuntimeError('Rendering modified physical state/controller/step')

    def finish(self, event=None):
        index = []
        for view in self.views:
            for name, key in [('observations.jsonl', 'observations'), ('frames.jsonl', 'records')]:
                with (view['folder']/name).open('w', encoding='utf-8') as file:
                    for row in view[key]:
                        file.write(json.dumps(row, allow_nan=False)+'\n')
            config = dict(view['config'], viewpoint='fixed_external_CCTV', parent='engine.worldNP',
                dt_s=self.dt, capture_stride_steps=view['stride'], initial_state_included=True,
                frames=len(view['records']), format='RGB PNG', channel_order_checked=True,
                capture_state_unchanged_checked=True, fixed_transform_checked=True,
                visibility_scope='Frustum projection is not occlusion/continuous-trackability evidence')
            write(view['folder']/'camera.json', config)
            index.append(dict(camera_id=view['config']['id'], directory=view['folder'].relative_to(self.output).as_posix()))
        write(self.output/'cctv_cameras.json', index)


def ground_point(pixel, calibration):
    """Pixel ray / z=0 intersection. Bbox bottom centre is only an estimated contact."""
    if len(pixel)!=2 or not all(math.isfinite(v) for v in pixel):
        raise ValueError('Finite image point required')
    intr = calibration.get('intrinsics')
    if not intr:
        return None, 'intrinsics_unavailable'
    direction = np.array([(pixel[0]-intr['cx_px'])/intr['fx_px'], 1.,
                          -(pixel[1]-intr['cy_px'])/intr['fy_px']])
    transform = np.asarray(calibration['camera_to_world_matrix'])
    ray = direction @ transform[:3, :3]; origin = transform[3, :3]
    if abs(ray[2]) < 1e-9:
        return None, 'parallel_to_ground'
    distance = (calibration['ground_plane_world_z_m']-origin[2])/ray[2]
    if distance <= 0:
        return None, 'ground_intersection_behind_camera'
    return (origin+distance*ray).tolist(), 'assumed_flat_ground_plane'
