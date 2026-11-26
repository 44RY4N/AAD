"""Validate new recordings and reuse frozen geometry feature implementations."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from PIL import Image
from experiments.geometry_math import transform_point
from .camera import ground_point, write
from .events import lines, extract


def check_cameras(source):
    if not (source/'cctv_cameras.json').exists():return []
    geometry={f['step']:f for f in lines(source/'geometry/geometry_frames.jsonl')}
    reports=[]
    for entry in json.loads((source/'cctv_cameras.json').read_text()):
        folder=source/entry['directory'];config=json.loads((folder/'camera.json').read_text())
        calibration=json.loads((folder/'calibration.json').read_text());obs=lines(folder/'observations.jsonl');refs=lines(folder/'frames.jsonl')
        if len(obs)!=len(refs) or len(obs)!=len(list((folder/'frames').glob('*.png'))):raise ValueError('Frame/metadata mismatch')
        expected=list(range(0,max(geometry)+1,config['capture_stride_steps']))
        if [o['step'] for o in obs]!=expected:raise ValueError('Capture schedule mismatch')
        max_error=0.;max_ground_error=0.
        for o,r in zip(obs,refs):
            if set(o)!={'camera_id','step','time_s','file','image_sha256'}:raise ValueError('State leakage into inference manifest')
            if o['time_s']!=geometry[o['step']]['time_s'] or r['step']!=o['step']:raise ValueError('Synchronization mismatch')
            if r['camera_to_world_matrix']!=calibration['camera_to_world_matrix']:raise ValueError('Camera moved')
            path=folder/o['file']
            import hashlib
            if hashlib.sha256(path.read_bytes()).hexdigest()!=o['image_sha256']:raise ValueError('Image hash mismatch')
            with Image.open(path) as image:
                image.load()
                if image.mode!='RGB' or image.size!=(config['width'],config['height']):raise ValueError('Invalid RGB image')
            for actor in geometry[o['step']]['actors']:
                world=transform_point([0,0,.7],actor['actor_to_world_matrix'])
                camera=transform_point(world,calibration['world_to_camera_matrix'])
                if camera[1]<=calibration['near_m']:continue
                ndc=transform_point(camera,calibration['projection_matrix'])
                uv=[(ndc[0]+1)*config['width']/2,(1-ndc[1])*config['height']/2]
                p=next(p for p in r['projections'] if p['vehicle_id']==actor['actor_id'])
                if p['in_frustum']:max_error=max(max_error,max(abs(x-y) for x,y in zip(uv,p['centre_pixel'])))
                ground=[*actor['position_world_m'][:2],0.]
                c=transform_point(ground,calibration['world_to_camera_matrix'])
                if c[1]>calibration['near_m']:
                    ndc=transform_point(c,calibration['projection_matrix'])
                    pixel=[(ndc[0]+1)*config['width']/2,(1-ndc[1])*config['height']/2]
                    recovered,reason=ground_point(pixel,calibration)
                    if recovered is not None:max_ground_error=max(max_ground_error,float(np.linalg.norm(np.array(recovered)-ground)))
        if max_error>.01 or max_ground_error>.001:raise ValueError('Calibration algebra crosscheck failed')
        reports.append(dict(camera_id=entry['camera_id'],frames=len(obs),passed=True,
            pose_invariant=True,rgb_valid=True,synchronized=True,max_projection_error_px=max_error,
            known_ground_point_roundtrip_error_m=max_ground_error,
            limitation='Roundtrip validates simulated projection algebra only, not estimated detector contact points or occlusion'))
    return reports


def compare(source,reference):
    from experiments.validate_geometry import normalize
    def load(p):
        rows=list(csv.DictReader((p/'trajectory.csv').open()));ids={r['vehicle_id']:r['role'] for r in rows}
        return [{k:v for k,v in r.items() if k!='vehicle_id'} for r in rows],normalize(json.loads((p/'decisions.json').read_text()),ids)
    a,b=load(source),load(reference)
    return dict(reference=str(reference.resolve()),trajectory_exact_match=a[0]==b[0],controller_decisions_exact_match=a[1]==b[1])


def run(source,output,reference=None):
    output.mkdir(parents=True,exist_ok=False)
    from experiments.validate_geometry import validate
    if not (source/'geometry/validation.json').exists():validate(source)
    from aad_analysis.run import extract as v1
    from aad_analysis_v02.run import extract as v2
    camera_path=source/'camera/camera.json'
    config=json.loads(camera_path.read_text()) if camera_path.exists() else {}
    compatible=config.get('width')==640 and config.get('height')==360 and config.get('capture_stride_steps')==1
    if compatible:v1(source,output/'v01')
    write(output/'v01_admission.json',dict(executed=compatible,
        reason='Original v0.1 requires 640x360 RGB at every decision sample; unchanged. Geometry v0.2 supports these recordings.'))
    v2(source,output/'v02')
    event=extract(source,output/'v02');write(output/'event.json',event)
    checks=dict(cameras=check_cameras(source),event_schema_valid=True,
        reproducibility=compare(source,reference) if reference else None)
    write(output/'validation.json',checks)
    print(json.dumps(dict(event_window=[event['start_step'],event['end_step']],checks=checks),indent=2))
    return checks


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__)
    for key in ['source','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--reference',type=Path);a=p.parse_args();run(a.source,a.output,a.reference)
