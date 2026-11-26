"""Reuse unchanged v0.1 derivative functions for camera-free successful sweep trials."""
import argparse
import ast
import csv
from pathlib import Path
from aad_analysis.features import actor_features
from aad_analysis.io import read_json,sha256,write_json,write_jsonl


def run(screenings,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);counts={}
    for screening in screenings:
        for item in read_json(Path(screening)/'comparison.json'):
            if item['exit_code']:continue
            name=item['config']['scenario_id'];source=Path(read_json(Path(screening)/name/'manifest.json')['source_directory'])
            dt=read_json(source/'metadata.json')['dt_s']
            with (source/'trajectory.csv').open(newline='') as file:rows=list(csv.DictReader(file))
            tracks={}
            for r in rows:
                r['step']=int(r['step']);r['lane']=ast.literal_eval(r['lane'])
                for k in ['time_s','x','y','heading_rad','speed_kmh','source_longitudinal_m','source_lateral_m']:r[k]=float(r[k])
                for k in ['on_lane','same_road']:r[k]=r[k]=='True'
                r['speed_ms']=r['speed_kmh']/3.6;tracks.setdefault(r['vehicle_id'],{})[r['step']]=r
            role_ids={r['role']:r['vehicle_id'] for r in rows};a,b=role_ids['subject'],role_ids['background_far']
            actors=[];pairs=[]
            cache={}
            for r in rows:
                m=actor_features(tracks[r['vehicle_id']],r['step'],dt);cache[r['step'],r['vehicle_id']]=m
                actors.append(dict(step=r['step'],time_s=r['time_s'],actor_id=r['vehicle_id'],role=r['role'],metrics=m,risk_assessment='unassessed'))
            for step in sorted(tracks[a]):
                metrics={}
                for field in ['vx_world','vy_world','ax_world','ay_world','v_longitudinal','v_lateral','a_longitudinal','a_lateral']:
                    ma,mb=cache[step,a][field],cache[step,b][field];valid=ma['validity']==mb['validity']=='valid'
                    metrics['relative_'+field]=dict(value=mb['value']-ma['value'] if valid else None,unit=ma['unit'],
                        validity='valid' if valid else 'unavailable',reason='other_minus_subject' if valid else 'invalid_or_missing_derivative_support',
                        method='unchanged_v01_actor_features_then_other_minus_subject',support_steps=ma['support_steps'])
                pairs.append(dict(step=step,time_s=step*dt,actor_ids=[a,b],metrics=metrics,risk_assessment='unassessed'))
            trial=output/name;trial.mkdir();write_jsonl(trial/'actor_motion.jsonl',actors);write_jsonl(trial/'relative_motion.jsonl',pairs)
            write_json(trial/'provenance.json',dict(trajectory_sha256=sha256(source/'trajectory.csv'),dt_s=dt,
                actor_feature_method='aad_analysis.features.actor_features unchanged',scope='Headless API reuse, not the RGB-requiring v0.1 CLI output schema'))
            counts[name]=dict(actor_samples=len(actors),pair_samples=len(pairs))
    write_json(output/'counts.json',counts);print('Processed',len(counts),'camera-free successful sweep trials')


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--screenings',nargs='+',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.screenings,a.output)
