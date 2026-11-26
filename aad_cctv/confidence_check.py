"""Compare a single lower-confidence setting with the matched archived baseline."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

from .camera import write
from .evaluate import run as evaluate
from .events import lines
from .temporal import temporal


def longest_match(matches, start, end, role):
    longest=length=0
    previous=None
    for row in matches:
        if row['role']!=role or not start<=row['step']<=end:
            continue
        length=length+1 if previous and previous['step']+1==row['step'] and previous['track_id']==row['track_id'] else 1
        longest=max(longest,length)
        previous=row
    return longest


def run(vision_root, output):
    output.mkdir(parents=True,exist_ok=False)
    raw=Path('test_output')
    analysis=Path('analysis_output')
    cases=[('clear','clear'),('concurrent','roadside')]
    cases.extend((name,name) for name in ('overtake','braking','crossing_interaction')
                 if (vision_root/name/'vision/report.json').is_file())
    for case,archive in cases:
        source=raw/f'cctv_{case}_20261119_final01'
        camera=source/'camera'
        lower=vision_root/case/'vision'
        baseline=analysis/f'cctv_vision_{archive}_20261119_01'
        work=output/case
        event=analysis/source.name/'event.json'
        ref=evaluate(source,camera,lower,work/'projected',event)
        layout=(dict(zone=[95.25,1.75,98.75,5.25]) if case=='crossing_interaction'
                else dict(lane_divider_y=5.25,extended=True))
        temporal(camera,lower,work/'temporal',**layout)
        after=lines(lower/'camera_observations.jsonl')
        before=lines(baseline/'camera_observations.jsonl')[:len(after)]
        if not after or [r['image_sha256'] for r in before]!=[r['image_sha256'] for r in after]:
            raise ValueError('Confidence comparison requires identical images')
        low_report=json.loads((lower/'report.json').read_text())
        old_report=json.loads((baseline/'report.json').read_text())
        if low_report['model_sha256']!=old_report['model_sha256'] or low_report['preprocessing']!=old_report['preprocessing']:
            raise ValueError('Confidence must be the only detector change')
        old_name='concurrent_roadside' if case=='concurrent' else case
        old_ref=analysis/'cctv_end_semester_20261125_03/replay'/old_name/'reference'
        old_matches=json.loads((old_ref/'reference_associations.json').read_text())
        new_matches=json.loads((work/'projected/reference_associations.json').read_text())
        hypotheses=json.loads((work/'temporal/evidence.json').read_text())['events']
        start,end=ref['event_window']
        if before[0]['step']>start or before[-1]['step']<end:
            raise ValueError('The identical prefix must contain the whole reference event')
        with (source/'trajectory.csv').open(newline='',encoding='utf-8') as f:
            reference_rows=list(csv.DictReader(f))
        inspection=[]
        for candidate in hypotheses:
            lo,hi=candidate['start_step'],candidate['end_step']
            roles=Counter(r['role'] for r in new_matches if lo<=r['step']<=hi and r['track_id']==candidate['track_id'])
            item=dict(camera_track=candidate['track_id'],family=candidate['family'],roles_by_envelope_association=dict(roles))
            if len(roles)==1:
                role=next(iter(roles))
                states={int(r['step']):r for r in reference_rows if r['role']==role}
                speed=[float(states[step]['speed_kmh'])/3.6 for step in (lo,hi)]
                item.update(recorded_role=role,reference_speed_m_s=speed,reference_speed_drop_m_s=speed[0]-speed[1])
            inspection.append(item)
        write(work/'comparison.json',dict(frames=len(after), confidence=[old_report['confidence'],low_report['confidence']],
              frames_with_predictions=[sum(bool(r['detections']) for r in data) for data in (before,after)],
              associations={role:[sum(r['role']==role and start<=r['step']<=end for r in matches)
                                  for matches in (old_matches,new_matches)] for role in ref['event_projected_samples']},
              longest_consecutive_same_label={role:[longest_match(matches,start,end,role) for matches in (old_matches,new_matches)]
                                               for role in ref['event_projected_samples']},
              event_projected_support=ref['event_projected_samples'], camera_hypotheses=hypotheses,
              diagnostic_hypothesis_roles=inspection,
              scope='Conditional projected associations. Labels are not certified identities.'))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(args.vision,args.output)
