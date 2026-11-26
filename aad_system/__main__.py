"""Stage-isolated command line. Simulation dependencies load only for simulation."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


REGISTRY={
 'lane_change':dict(module='experiments.aad_lane_change_v02.run',status='demonstrated'),
 'overtaking':dict(module='experiments.aad_overtake_v01.run',status='pilot demonstrated'),
 'braking':dict(module='aad_system.braking_run',status='experimental'),
 'right_of_way':dict(module=None,status='not implemented; road/priority/VRU feasibility unvalidated')}


def validate_config(config: dict) -> None:
    import math
    for field in ['subject_speed_kmh','leader_speed_kmh','target_vehicle_speed_kmh']:
        if type(config.get(field)) not in (int,float) or not math.isfinite(config[field]) or config[field]<=0:
            raise ValueError(field+' must be finite and positive; stationary IDM is unsupported')
    if type(config.get('steps')) is not int or config['steps']<1:raise ValueError('Positive integer steps required')


def main() -> None:
    p=argparse.ArgumentParser(__doc__);p.add_argument('stage',choices=['registry','simulate','validate','vision','temporal','extract','review'])
    p.add_argument('--family',choices=list(REGISTRY));p.add_argument('--config',type=Path)
    p.add_argument('--source',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--weights',type=Path,default=Path('models/yolox_opencv_2022nov/model.onnx'))
    p.add_argument('--rgb',action='store_true');a=p.parse_args()
    if a.stage=='registry':print(json.dumps(REGISTRY,indent=2));return
    if a.stage=='simulate':
        if not a.config or not a.family or not a.output:p.error('simulate requires --family --config --output')
        config=json.loads(a.config.read_text());validate_config(config)
        module=REGISTRY[a.family]['module']
        if module is None:raise ValueError(REGISTRY[a.family]['status'])
        cmd=[sys.executable,'-m',module,'--config',str(a.config),'--output',str(a.output),'--box-lanes','--record-geometry']
        if a.rgb:cmd.append('--record-rgb')
        subprocess.run(cmd,check=True);return
    if not a.source:p.error('--source required')
    if a.stage=='review':print((a.source/'review.html').resolve());return
    if not a.output:p.error('--output required')
    if a.stage=='validate':
        from .validate import check
        if a.output.exists():raise FileExistsError(a.output)
        a.output.write_text(json.dumps(check(a.source),indent=2))
    elif a.stage=='vision':
        from .vision import run
        print(json.dumps(run(a.source,a.output,a.weights),indent=2))
    elif a.stage=='extract':
        from aad_analysis_v02.run import extract
        extract(a.source,a.output)
    elif a.stage=='temporal':
        from .temporal import run
        run(a.source,a.output)


if __name__=='__main__':main()
