"""Audit frozen video-only tags against independent simulator records."""
import argparse
from collections import Counter
import csv
import json
from math import dist, isclose
from pathlib import Path
from statistics import median

from aad_system.vision import sha
from .camera import write


def run(protocol, workspace, raw, output):
    design=json.loads(protocol.read_text())
    results=[]
    for case in design['cases']:
        name=case['name']
        recording=raw/case.get('recording',f'aad_unseen_{name}_01')
        tag_root=workspace/name
        report=json.loads((tag_root/'report.json').read_text())
        checks=json.loads((workspace/f'{name}_reference/validation.json').read_text())
        reference=json.loads((workspace/f'{name}_reference/event.json').read_text())
        camera=recording/'camera'
        if (report['rules']!=design['rules'] or report['code_sha256']!=design['tagger_sha256']
                or report['video_sha256']!=sha(workspace/f'{name}.mp4')
                or report['calibration_sha256']!=sha(camera/'calibration.json')):
            raise ValueError('Frozen inference lineage does not match')
        if design.get('detector_sha256')!=report.get('derived_from_code_sha256'):
            raise ValueError('Frozen source detector lineage does not match')
        if not isclose(report['fps']*reference['dt_s'],1,abs_tol=1e-6) or report['frames']!=checks['cameras'][0]['frames']:
            raise ValueError('Decoded video and simulator capture schedule differ')
        with (recording/'trajectory.csv').open(newline='',encoding='utf-8') as f:
            states={}
            for row in csv.DictReader(f):
                states.setdefault(int(row['step']),{})[row['role']]=[float(row['x']),float(row['y'])]
        tracks=json.loads((tag_root/'tracks.json').read_text())

        def match(identity,start,end):
            roles=Counter()
            offsets=[]
            for step in range(start,end+1):
                estimate=next((r['xy_m'] for r in tracks[step] if r['track_id']==identity),None)
                if estimate is None:
                    continue
                role,offset=min(( (role,dist(estimate,xy)) for role,xy in states[step].items()),key=lambda row:row[1])
                if offset<=2:
                    roles[role]+=1
                    offsets.append(offset)
            return dict(nearest_roles_within_2m=dict(roles),median_offset_m=median(offsets) if offsets else None)

        tags=report['tags']
        match_results=[]
        for tag in tags:
            start,end=tag['start_frame'],tag['end_frame']
            overlap=max(0,min(end,reference['end_step'])-max(start,reference['start_step'])+1)
            match_results.append(dict(tag=tag['tag'],start_frame=start,end_frame=end,reference_overlap_frames=overlap,
                                      subject_match=match(tag['track_id'],start,end),
                                      counterpart_matches=[match(p['track_id'],start,end) for p in tag['interaction_evidence']]))
        matched=any(r['reference_overlap_frames']>0 and r['subject_match']['nearest_roles_within_2m'].get('subject',0)>=3
                    for r in match_results)
        passed=(len(tags)==1 and tags[0]['tag']==case['expected'] and matched
                and reference['physical_behaviour']=='detected' and not reference['censored'])
        separation=reference['interaction_evidence']['event_window_minima']['outer_envelope_planar_separation']['value']
        results.append(dict(case=name,expected=case['expected'],observed_tags=[r['tag'] for r in match_results],
                            verified=passed,frames=report['frames'],reference_window=[reference['start_step'],reference['end_step']],
                            reference_min_envelope_separation_m=separation, matches=match_results))
    write(output,dict(cases=results,passed=all(r['verified'] for r in results),
                      scope='Small held-out scripted scenario check. Candidate expectation is target-lane context, not an independent abnormality label.'))
    print(json.dumps(results,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ('protocol','workspace','raw','output'):
        parser.add_argument('--'+arg,type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    run(args.protocol,args.workspace,args.raw,args.output)
