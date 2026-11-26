"""Evaluate actual verified-window concurrence; no criticality threshold."""
import argparse
from collections import Counter
import contextlib
import csv
import io
import json
from pathlib import Path
from shapely.geometry import Polygon
from aad_analysis.io import read_json,write_json
from aad_analysis_v02.run import extract
from experiments.validate_geometry import validate


def summarize(source,analysis):
    meta=read_json(source/'metadata.json');event=meta['result']
    lo,hi=event['departure_step'],event['confirmed_step']
    frames=[json.loads(s) for s in (source/'geometry/geometry_frames.jsonl').read_text().splitlines()]
    pairs=[json.loads(s) for s in (analysis/'pair_features.jsonl').read_text().splitlines()]
    pairs=[r for r in pairs if set(r['actor_roles'])=={'subject','background_far'}]
    window=[r for r in pairs if lo<=r['step']<=hi]
    measure=lambda r,n:next(m for m in r['metrics'] if m['name']==n)
    road=read_json(source/'geometry/road_geometry.json')
    lane=next(Polygon(l['polygon']['xy_m']) for l in road['lanes'] if l['lane_id']==['>>','>>>',0])
    concurrence=[];motion=[]
    for f in frames:
        a,b=[next(a for a in f['actors'] if a['role']==role) for role in ['subject','background_far']]
        pa,pb=[Polygon(actor['footprint_components'][0]['world_footprint_xy_m']) for actor in [a,b]]
        common=pa.intersection(lane).area>1e-12 and pb.intersection(lane).area>1e-12
        longitudinal=min(pa.bounds[2],pb.bounds[2])-max(pa.bounds[0],pb.bounds[0])
        if lo<=f['step']<=hi and common and longitudinal>=0:concurrence.append(f['step'])
        motion.append(dict(step=f['step'],relative_x_m=b['position_world_m'][0]-a['position_world_m'][0],
            relative_vx_m_s=b['velocity_world_m_s'][0]-a['velocity_world_m_s'][0],
            longitudinal_projection_overlap_m=longitudinal,common_target_lane_occupancy=common))
    sep=lambda r:measure(r,'outer_envelope_planar_separation')['value']
    best=min(window,key=sep);ttc=[measure(r,'constant_velocity_fixed_orientation_envelope_ttc_2d') for r in window]
    with (source/'trajectory.csv').open(newline='') as file:rows=list(csv.DictReader(file))
    sub=next(r for r in rows if r['role']=='subject' and int(r['step'])==lo)
    return dict(config=meta['settings'],event=event,event_window_steps=[lo,hi],
        concurrence_definition='Longitudinal envelope projections overlap AND both envelopes have positive area in target lane, during inclusive verified departure..confirmation; geometric context, not risk',
        concurrent_steps=concurrence,concurrent_samples=len(concurrence),
        event_minimum_envelope_separation_m=sep(best),event_minimum_step=best['step'],
        full_minimum_envelope_separation_m=min(map(sep,pairs)),
        model_ttc_validity_counts=dict(Counter(m['validity'] for m in ttc)),
        minimum_valid_model_ttc_s=min([m['value'] for m in ttc if m['validity']=='valid'],default=None),
        subject_departure_speed_kmh=float(sub['speed_kmh']),
        command_to_departure_steps=lo-meta['settings']['command_step'],
        vehicle_contact_samples=sum(r['crash']=='True' for r in rows),
        all_actors_localized=all(r['on_lane']=='True' and r['same_road']=='True' for r in rows),
        relative_motion=motion,risk_assessment='unassessed')


def run(source,output):
    source,output=Path(source),Path(output);output.mkdir(parents=True,exist_ok=False)
    trials=read_json(source/'execution.json');summaries=[]
    for trial in trials:
        raw=Path(trial['output']);name=trial['config']['scenario_id']
        if trial['exit_code']:
            item=dict(config=trial['config'],exit_code=trial['exit_code'],outcome='failed_or_terminated_see_raw_log',
                log=str(source/(name+'.log')),risk_assessment='unassessed')
        else:
            with contextlib.redirect_stdout(io.StringIO()):
                validate(raw);extract(raw,output/name)
            item=summarize(raw,output/name);item.update(exit_code=0,outcome='verified_merge')
        write_json(output/(name+'_summary.json'),item)
        summaries.append({k:v for k,v in item.items() if k!='relative_motion'})
        print(name,item['outcome'],item.get('concurrent_samples'),flush=True)
    write_json(output/'comparison.json',summaries)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.source,a.output)
