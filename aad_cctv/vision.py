"""Image-only YOLOX/IoU inference, with simulated fixed-camera calibration optional."""
import argparse
from collections import Counter,defaultdict
import json
from itertools import islice
from pathlib import Path
import cv2
from aad_system.vision import Detector,Tracker,iou,sha
from .camera import ground_point,write

ALLOWED={'camera_id','step','time_s','file','image_sha256'}


def observations(folder):
    records=(json.loads(line) for line in (folder/'observations.jsonl').read_text(encoding='utf-8').splitlines())
    previous_step=-1;previous_time=-1.
    for r in records:
        if set(r)!=ALLOWED:raise ValueError('Inference manifest must contain observations only, without actor state')
        if r['step']<=previous_step or r['time_s']<=previous_time:raise ValueError('Observations must be ordered in time')
        path=(folder/r['file']).resolve()
        if not path.is_relative_to((folder/'frames').resolve()):raise ValueError('Image path escapes frame directory')
        if sha(path)!=r['image_sha256']:raise ValueError('Image hash mismatch')
        previous_step,previous_time=r['step'],r['time_s']
        yield r,path


def tiled_detections(detector,image):
    width=image.shape[1]
    crop_width=round(.6*width)
    candidates=[]
    for left in (0,width-crop_width):
        for result in detector.infer(image[:,left:left+crop_width]):
            x1,y1,x2,y2=result['box_xyxy']
            if (left and x1<=0) or (not left and x2>=crop_width):
                continue
            candidates.append(dict(result,box_xyxy=[x1+left,y1,x2+left,y2]))
    detections=[]
    for result in sorted(candidates,key=lambda d:-d['confidence']):
        if any(d['class_id']==result['class_id'] and iou(d['box_xyxy'],result['box_xyxy'])>.5
               for d in detections):
            continue
        detections.append(result)
    return detections


def run(folder,output,weights,*,tiles=False,capture_limit=None,confidence=.35):
    if capture_limit is not None and capture_limit<1:
        raise ValueError('Capture limit must be positive')
    if not 0<confidence<=1:
        raise ValueError('Confidence must be in (0, 1]')
    output.mkdir(parents=True,exist_ok=False)
    detector=Detector(weights,confidence=confidence);tracker=Tracker();tracks_by_id=defaultdict(list);counts=Counter();samples=[]
    calibration=json.loads((folder/'calibration.json').read_text())
    with (output/'camera_observations.jsonl').open('w',encoding='utf-8') as f:
        for record,path in islice(observations(folder),capture_limit):
            image=cv2.imread(str(path))
            if image is None:raise ValueError('Cannot decode '+str(path))
            detections=tiled_detections(detector,image) if tiles else detector.infer(image)
            tracks=tracker.update(detections,record['step'],record['time_s'])
            for t in tracks:
                box=t['box_xyxy'];pixel=[(box[0]+box[2])/2,box[3]]
                estimate,reason=ground_point(pixel,calibration)
                t['estimated_ground_contact_world_m']=estimate
                t['ground_estimate_status']=reason
                t['ground_estimate_assumption']='Detected box bottom-centre approximates road contact on z=0; not actor centroid or validated metric trajectory'
                tracks_by_id[t['track_id']].append(dict(step=record['step'],time_s=record['time_s'],box_xyxy=box,
                    image_centre=t['centre'],estimated_ground_contact_world_m=estimate))
                counts[t['class_name']]+=1
            f.write(json.dumps(dict(step=record['step'],time_s=record['time_s'],image_sha256=record['image_sha256'],
                detections=detections,tracks=tracks,source='RGB_and_static_simulated_camera_calibration_only'),allow_nan=False)+'\n')
            samples.append(dict(step=record['step'],detections=len(detections)))
            if record['step']%10==0:
                for t in tracks:
                    x1,y1,x2,y2=map(int,t['box_xyxy']);cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),2)
                    cv2.putText(image,f"{t['track_id']} {t['class_name']} {t['confidence']:.2f}",(x1,max(15,y1)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
                if not cv2.imwrite(str(output/f"overlay_{record['step']:06d}.jpg"),image):raise OSError('Overlay write failed')
    temporal=[]
    for identity,series in tracks_by_id.items():
        temporal.append(dict(track_id=identity,samples=series,observations=len(series),
            duration_s=series[-1]['time_s']-series[0]['time_s'],
            image_displacement_px=[series[-1]['image_centre'][i]-series[0]['image_centre'][i] for i in range(2)],
            event_family=None,abnormality_assessment='unassessed',subject_identity='unknown_without_independent_visual_identification',
            continuous_identity_validated=False))
    write(output/'track_trajectories.json',temporal)
    report=dict(frames=len(samples),frames_with_detections=sum(s['detections']>0 for s in samples),detections_by_class=dict(counts),
        created_tracks=len(temporal),tracks_with_at_least_10_observations=sum(t['observations']>=10 for t in temporal),
        model_sha256=sha(weights),confidence=confidence,nms_iou=.5,inference_input_size=[640,640],
        preprocessing=('Two overlapping 60%-width crops, then class-wise cross-crop IoU merge'
                       if tiles else 'Unchanged original Detector: raw RGB top-left letterbox, 114 fill'),
        capture_limit=capture_limit,
        tracker='Unchanged class-aware greedy IoU gate .2, max_age 3 simulation steps',
        source_manifest_sha256=sha(folder/'observations.jsonl'),calibration_sha256=sha(folder/'calibration.json'),
        inference_inputs='Images, timestamps and fixed camera calibration only; no simulator actors/trajectories',
        precision=None,recall=None,AP=None,IDF1=None,HOTA=None,
        metrics_unavailable_reason='No independently reviewed visible-object and identity labels for these new camera views',
        ground_reconstruction_validation='Projection algebra crosschecked separately; bbox contact estimates remain unvalidated')
    write(output/'report.json',report);write(output/'frame_counts.json',samples)
    (output/'review.html').write_text('<meta charset="utf-8"><h1>Fixed CCTV detection/tracking</h1><p>Green boxes are model predictions; no ground-truth actor IDs enter inference.</p>'+''.join(f'<h3>{p.stem}</h3><img width="960" src="{p.name}">' for p in sorted(output.glob('overlay*.jpg'))),encoding='utf-8')
    print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__)
    for key in ['source','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--weights',type=Path,default=Path('models/yolox_opencv_2022nov/model.onnx'))
    p.add_argument('--tiles',action='store_true')
    p.add_argument('--capture-limit',type=int)
    p.add_argument('--confidence',type=float,default=.35)
    a=p.parse_args();run(a.source,a.output,a.weights,tiles=a.tiles,capture_limit=a.capture_limit,confidence=a.confidence)
