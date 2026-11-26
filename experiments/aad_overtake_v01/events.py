"""Retrospective pass-and-return verification from recorded state and envelopes."""
import argparse
import json
from pathlib import Path
from aad_analysis.io import write_json
from experiments.validate_geometry import finite


def reconstruct(source: Path) -> dict:
    manifest=json.loads((source/'geometry/geometry_manifest.json').read_text())
    if not manifest.get('completed'):
        raise ValueError('Incomplete simulation; no completed overtaking assertion')
    frames=[json.loads(s) for s in (source/'geometry/geometry_frames.jsonl').read_text().splitlines()]
    finite(frames)
    if not frames or [f['step'] for f in frames]!=list(range(len(frames))):
        raise ValueError('Missing or unordered observation steps')
    phases=[];seen_pass=False;completed=None;settled=[]
    for f in frames:
        a,b=[next(x for x in f['actors'] if x['role']==r) for r in ['subject','leader']]
        ax=[p[0] for c in a['footprint_components'] for p in c['world_vertices_m']]
        bx=[p[0] for c in b['footprint_components'] for p in c['world_vertices_m']]
        clear=min(ax)>max(bx)
        if clear and a['position_world_m'][1]>5.25:seen_pass=True
        returned=seen_pass and clear and abs(a['position_world_m'][1]-3.5)<.35
        settled=settled+[f['step']] if returned else []
        if len(settled)>=11 and completed is None:completed=f['step']
        phases.append(dict(step=f['step'],time_s=f['time_s'],subject_id=a['actor_id'],leader_id=b['actor_id'],
            subject_rear_minus_leader_front_m=min(ax)-max(bx),
            phase='returned_and_ahead' if returned else 'fully_ahead_in_passing_path' if seen_pass and a['position_world_m'][1]>5.25 else
                'alongside' if min(max(ax),max(bx))>=max(min(ax),min(bx)) else 'approach_or_transition'))
    return dict(event_type='overtaking',event_subtype='same_direction_out_and_return',
        observed_full_pass=seen_pass,return_confirmation_step=completed,completed_under_definition=completed is not None,
        definition='World-x outer-envelope full leader clearance, passing path y>5.25, return within .35m of source centre for 11 samples; straight road only',
        risk_assessment='unassessed',phases=phases)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    write_json(a.output,reconstruct(a.source))
