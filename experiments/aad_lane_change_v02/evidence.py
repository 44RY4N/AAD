"""Same-step target occupancy plus positive finite model TTC during actual manoeuvre."""
import json
from pathlib import Path
from shapely.geometry import Polygon
from aad_analysis.io import read_json
from .screen import summarize


def evidence(raw,features):
    raw,features=Path(raw),Path(features);result=summarize(raw,features)
    pairs={r['step']:r for r in [json.loads(s) for s in (features/'pair_features.jsonl').read_text().splitlines()]
           if set(r['actor_roles'])=={'subject','background_far'}}
    road=read_json(raw/'geometry/road_geometry.json')
    lane=next(Polygon(l['polygon']['xy_m']) for l in road['lanes'] if l['lane_id']==['>>','>>>',0])
    hits=[]
    for line in (raw/'geometry/geometry_frames.jsonl').read_text().splitlines():
        f=json.loads(line)
        if not result['event_window_steps'][0]<=f['step']<=result['event_window_steps'][1]:continue
        a,b=[next(x for x in f['actors'] if x['role']==r) for r in ['subject','background_far']]
        polygons=[Polygon(x['footprint_components'][0]['world_footprint_xy_m']) for x in [a,b]]
        if not all(p.intersection(lane).area>1e-12 for p in polygons):continue
        ttc=next(m for m in pairs[f['step']]['metrics'] if m['name']=='constant_velocity_fixed_orientation_envelope_ttc_2d')
        if ttc['validity']=='valid' and ttc['value']>0:hits.append(dict(step=f['step'],time_s=f['time_s'],model_ttc_s=ttc['value'],
            target_reference_dx_m=b['position_world_m'][0]-a['position_world_m'][0],
            target_relative_vx_m_s=b['velocity_world_m_s'][0]-a['velocity_world_m_s'][0],
            target_lane_occupancy_fractions=[p.intersection(lane).area/p.area for p in polygons]))
    result['concurrent_model_interaction_samples']=hits
    result['model_interaction_definition']='At same recorded verified-window step BOTH current envelopes occupy target lane AND constant-velocity fixed-orientation model predicts future envelope intersection within recorded horizon. This is model interaction evidence, not current contact or calibrated criticality.'
    result['strict_projection_concurrence_note']='concurrent_steps from screen.py requires current longitudinal projection overlap, a stronger distinct diagnostic, not required by model interaction definition'
    return result
