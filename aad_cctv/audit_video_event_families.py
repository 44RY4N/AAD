"""Check video-only family tags against a separately opened simulator reference."""
import argparse
from collections import Counter
import csv
import json
from math import dist
from pathlib import Path

from aad_system.vision import sha
from .camera import write
from .video_event_families import METHOD


def audit(workspace, jobs, output):
    rows=[]
    for entry in json.loads(jobs.read_text()):
        name=entry['name']
        raw=Path(entry['output'])
        prediction_folder=workspace/entry.get('prediction',name)
        report_path=prediction_folder/'report.json'
        reference_path=workspace/f'{name}_reference/event.json'
        if not report_path.is_file() or not reference_path.is_file():
            rows.append(dict(clip=name,status='capture_or_processing_incomplete'))
            continue
        report=json.loads(report_path.read_text())
        reference=json.loads(reference_path.read_text())
        reference_checks=json.loads((reference_path.parent/'validation.json').read_text())
        if (report['method']!=METHOD or report['video_sha256']!=sha(workspace/f'{name}.mp4')
                or report['calibration_sha256']!=sha(raw/'camera/calibration.json')
                or report['frames']!=reference_checks['cameras'][0]['frames']):
            raise ValueError(f'{name}: video lineage or capture schedule mismatch')
        tracks=json.loads((prediction_folder/'tracks.json').read_text())
        samples={}
        with (raw/'trajectory.csv').open(newline='') as f:
            for r in csv.DictReader(f):
                samples.setdefault(int(r['step']),{})[r['role']]=(
                    [float(r['x']),float(r['y'])],float(r['speed_kmh'])/3.6)

        def associate(identity,start,end):
            roles=Counter()
            for step in range(start,end+1):
                xy=next((t['xy_m'] for t in tracks[step] if t['track_id']==identity),None)
                if xy is None:
                    continue
                role,offset=min(((role,dist(xy,point)) for role,(point,_) in samples[step].items()),
                                key=lambda pair:pair[1])
                if offset<=2:
                    roles[role]+=1
            return dict(roles)

        checked=[]
        for event in report['tags']:
            if event['family']!=reference['family']:
                continue
            start,end=event['start_frame'],event['end_frame']
            roles=associate(event['track_id'],start,end)
            evidence=event['interaction_evidence']
            partner=evidence.get('counterpart_track') if isinstance(evidence,dict) else None
            other=associate(partner,start,end) if partner is not None else {}
            ref_start,ref_end=reference['start_step'],reference['end_step']
            overlap=max(0,min(end,ref_end)-max(start,ref_start)+1) if ref_start is not None else 0
            subject_support=roles.get('subject',0)
            if event['family']=='D':
                subject_support=max(subject_support,other.get('subject',0))
                corroborated=(roles.get('subject',0)>=2 and other.get('crossing_target',0)>=2 or
                              roles.get('crossing_target',0)>=2 and other.get('subject',0)>=2)
            elif event['family']=='B':
                corroborated=subject_support>=3 and other.get('leader',0)>=3
                corroborated=corroborated and event['physical_behaviour']==(
                    'return_behind_after_excursion' if entry['family']=='abort' else 'pass_and_return')
            else:
                start_speed=samples[start]['subject'][1]
                end_speed=samples[end]['subject'][1]
                corroborated=subject_support>=3 and start_speed-end_speed>=1.5
            checked.append(dict(tag=event['tag'],behavior=event['physical_behaviour'],interval_s=[event['start_s'],event['end_s']],
                                actor_associations=roles,counterpart_associations=other,reference_overlap_frames=overlap,
                                consistent=bool(corroborated and overlap),evidence=evidence))
        rows.append(dict(clip=name,family=reference['family'],frames=report['frames'],
                         status='consistent_tag' if any(v['consistent'] for v in checked) else 'no_consistent_tag',
                         video_report=report_path.as_posix(), reference_event=reference_path.as_posix(),
                         video_tags=checked,reference_window=[reference['start_s'],reference['end_s']],
                         reference_details=reference['interaction_evidence']['details'],
                         clip_contacts=reference['outcome']['vehicle_contact_samples']))
    write(output,dict(clips=rows,scope='Scenario and actor alignment; independent abnormality labels are unavailable.'))
    for row in rows:
        print(row['clip'],row['status'],[(e['behavior'],e['tag'],e['consistent']) for e in row.get('video_tags',[])])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('workspace','jobs','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    audit(args.workspace,args.jobs,args.output)
