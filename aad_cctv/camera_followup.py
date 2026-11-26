"""Evaluate camera-only hypotheses and a paired crop ablation on retained views."""
import argparse
import csv
import json
from pathlib import Path

from .camera import write
from .evaluate import run as evaluate
from .events import lines
from .replay_evaluation import ANALYSIS, vision_jobs
from .temporal import temporal


def run(output, tiles, near=None, high=None):
    output.mkdir(parents=True, exist_ok=False)
    cases=[]
    for job in vision_jobs():
        if job['returncode']:
            continue
        name=job['name']
        camera=Path(job['source'])
        vision=Path(job['output'])
        intersection=name.startswith('crossing_')
        layout=dict(zone=[95.25,1.75,98.75,5.25]) if intersection else dict(lane_divider_y=5.25,extended=True)
        folder=output/name
        temporal(camera,vision,folder/'full_frame',**layout)
        cases.append((name,'full_frame',folder/'full_frame/evidence.json'))
        if name not in {'clear','concurrent_roadside'}:
            continue
        variant=tiles/('clear' if name=='clear' else 'concurrent')/'vision'
        temporal(camera,variant,folder/'tiles',**layout)
        cases.append((name,'tiles_first_100',folder/'tiles/evidence.json'))
        source=camera.parent
        event=ANALYSIS/source.name/'event.json'
        diagnostic=evaluate(source,camera,variant,folder/'tiles_reference',event)
        before=lines(vision/'camera_observations.jsonl')[:diagnostic['evaluated_frames']]
        after=lines(variant/'camera_observations.jsonl')
        if [r['image_sha256'] for r in before]!=[r['image_sha256'] for r in after]:
            raise ValueError('The ablation requires identical frames')
        baseline_root=ANALYSIS/'cctv_end_semester_20261125_03/replay'/name/'reference'
        baseline=json.loads((baseline_root/'report.json').read_text())
        before_temporal=json.loads((folder/'full_frame/evidence.json').read_text())
        after_temporal=json.loads((folder/'tiles/evidence.json').read_text())
        end=after[-1]['step']
        def event_counts(evidence):
            selected=[x for x in evidence['events'] if x['end_step']<=end]
            return dict(hypotheses=len(selected),review_flags=sum(e['review_status']=='candidate_for_review' for e in selected))
        write(folder/'ablation.json',dict(same_image_count=len(before),last_step=end,
              frames_with_detections=dict(full=sum(bool(r['detections']) for r in before),
                                           tiles=sum(bool(r['detections']) for r in after)),
              event_associations=dict(full=baseline['event_matched_projected_samples'],
                  tiles=diagnostic['event_matched_projected_samples'], projected_support=diagnostic['event_projected_samples']),
              camera_outputs=dict(full=event_counts(before_temporal),tiles=event_counts(after_temporal)),
              evidence_scope='Frame output counts and conditional reference overlap; object visibility and identity unknown'))
    for placement, pair in [('near',near),('high',high)]:
        if pair is None:
            continue
        source,predictions=pair
        folder=output/f'concurrent_{placement}'
        temporal(source/'camera',predictions,folder/'temporal',lane_divider_y=5.25,extended=True)
        cases.append((folder.name,'full_frame_first_100',folder/'temporal/evidence.json'))
        event=ANALYSIS/source.name/'event.json'
        reference=evaluate(source,source/'camera',predictions,folder/'reference',event)
        vision=json.loads((predictions/'report.json').read_text())
        write(folder/'pilot.json',dict(frames=vision['frames'],frames_with_output=vision['frames_with_detections'],
              support=reference['event_projected_samples'],associations=reference['event_matched_projected_samples'],
              source=source.as_posix(),vision=(predictions/'report.json').as_posix(),
              reference=(folder/'reference/report.json').as_posix()))
    with (output/'hypotheses.csv').open('x',newline='',encoding='utf-8') as f:
        writer=csv.writer(f)
        writer.writerow(['view','preprocessing','family','frames','hypotheses','review_flags','evidence'])
        for name,method,path in cases:
            evidence=json.loads(path.read_text())
            for family in evidence['evaluated_hypothesis_families']:
                events=[e for e in evidence['events'] if e['family']==family]
                writer.writerow([name,method,family,evidence['frames'],len(events),
                                 sum(e['review_status']=='candidate_for_review' for e in events),path.as_posix()])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--tiles',type=Path,required=True)
    parser.add_argument('--near',type=Path,nargs=2,metavar=('RECORDING','VISION'))
    parser.add_argument('--high',type=Path,nargs=2,metavar=('RECORDING','VISION'))
    args=parser.parse_args()
    run(args.output,args.tiles,args.near,args.high)
