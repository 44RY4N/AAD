"""Execute preserved A/B/C controllers with a fixed-world recording adapter."""
import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
from unittest.mock import patch


def run(family, settings, cameras, output, rgb=True, box_lanes=True):
    from aad_system.__main__ import validate_config
    validate_config(dict(settings,target_vehicle_speed_kmh=30) if family=='baseline_lane_change' else settings)
    from experiments import recording, geometry_recording
    from experiments.aad_lane_change_v02 import context_camera
    from .camera import FixedCCTVRecorder, sensor_config, write
    from .capture_geometry import IndependentGeometryRecorder
    if family == 'baseline_lane_change':
        from experiments import run as runner
    elif family == 'lane_change':
        from experiments.aad_lane_change_v02 import run as runner
    else:
        from experiments.aad_overtake_v01 import run as runner
    settings = dict(settings)
    settings.setdefault('scenario_id','original_extended_lane_change')
    settings.pop('camera_mount', None); settings.pop('camera_look_at', None)
    settings['camera_architecture'] = 'fixed_external_CCTV' if rgb else 'headless_control'
    failure = None
    with ExitStack() as stack:
        if settings.get('subject_proxy'):
            from .proxy import vehicle_config
            original_env=runner.MetaDriveEnv
            def proxy_env(config):
                config['agent_configs']['default_agent'].update(vehicle_config(settings['subject_proxy']))
                return original_env(config)
            stack.enter_context(patch.object(runner,'MetaDriveEnv',proxy_env))
        stack.enter_context(patch.object(recording, 'camera_config', lambda: sensor_config(cameras)))
        stack.enter_context(patch.object(recording, 'RGBRecorder', lambda env, dest: FixedCCTVRecorder(env, dest, cameras)))
        stack.enter_context(patch.object(geometry_recording, 'GeometryRecorder', IndependentGeometryRecorder))
        if family == 'braking':
            from aad_system.braking_run import BrakingTraffic
            if settings['leader_after_kmh'] <= 0:
                raise ValueError('Post-braking desired speed must be positive')
            settings['negative_control'] = True
            stack.enter_context(patch.object(runner, 'ControlledTraffic', BrakingTraffic))
        if family == 'abort':
            from .controllers import AbortIDM
            stack.enter_context(patch.object(runner, 'ObservedIDM', AbortIDM))
        try:
            runner.run(settings, output, box_lanes=box_lanes, record_rgb=rgb, record_geometry=True)
        except Exception as exc:
            failure = repr(exc)
            raise
        finally:
            if output.exists():
                write(output/'cctv_execution.json', dict(family=family, settings=settings,
                    cameras=cameras if rgb else [], failure=failure,
                    source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in Path(__file__).parent.glob('*.py')},
                    subject_convention='role=subject is a passenger-car research proxy; never a validated ambulance',
                    inference_scope='Only camera/observations.jsonl and images; actor reference files excluded'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--family', choices=['baseline_lane_change', 'lane_change', 'overtaking', 'braking', 'abort'], required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--cameras', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--proxy-config', type=Path)
    args = parser.parse_args()
    settings=json.loads(args.config.read_text())
    if args.proxy_config:settings['subject_proxy']=json.loads(args.proxy_config.read_text())
    run(args.family, settings, json.loads(args.cameras.read_text()), args.output, not args.headless)
