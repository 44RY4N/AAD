"""CPU YOLOX, independent image tracker, and separate projected-envelope audit."""
import hashlib
import json
import math
from pathlib import Path
import cv2
import numpy as np
from experiments.geometry_math import transform_point


def sha(path: Path) -> str:return hashlib.sha256(path.read_bytes()).hexdigest()


def iou(a: list,b: list) -> float:
    intersection=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union>0 else 0.


class Tracker:
    """Greedy class-aware IoU baseline; no simulator information accepted."""
    def __init__(self,max_age: int=3,gate: float=.2):
        self.max_age=max_age;self.gate=gate;self.active={};self.next_id=1
    def update(self,detections: list,step: int,time_s: float) -> list:
        if not math.isfinite(time_s) or step<=getattr(self,'last_input_step',-1):raise ValueError('Finite time and increasing steps required')
        self.last_input_step=step
        for d in detections:
            b=d['box_xyxy']
            if len(b)!=4 or not all(math.isfinite(v) for v in b) or b[2]<=b[0] or b[3]<=b[1] or not 0<=d['confidence']<=1:
                raise ValueError('Invalid detection box/confidence')
        self.active={k:v for k,v in self.active.items() if step-v['last_step']<=self.max_age}
        candidates=sorted([(iou(d['box_xyxy'],t['box_xyxy']),j,k) for j,d in enumerate(detections)
            for k,t in self.active.items() if d['class_id']==t['class_id']],reverse=True)
        matches={};used=set()
        for score,j,k in candidates:
            if score>=self.gate and j not in matches and k not in used:matches[j]=k;used.add(k)
        result=[]
        for j,d in enumerate(detections):
            k=matches.get(j)
            if k is None:k=self.next_id;self.next_id+=1
            old=self.active.get(k);centre=[(d['box_xyxy'][i]+d['box_xyxy'][i+2])/2 for i in range(2)]
            dt=time_s-old['time_s'] if old else None
            v=[(centre[i]-old['centre'][i])/dt for i in range(2)] if dt and dt>0 else None
            t=dict(d,track_id=k,first_step=old['first_step'] if old else step,last_step=step,time_s=time_s,
                centre=centre,image_velocity_px_s=v,state='matched' if old else 'new')
            self.active[k]=t;result.append(t)
        return result


class Detector:
    def __init__(self,weights: Path,confidence: float=.35):
        if not weights.is_file():raise FileNotFoundError('Provide YOLOX-S ONNX weights; see models provenance')
        self.net=cv2.dnn.readNet(str(weights));self.confidence=confidence
        grids=[];strides=[]
        for stride in [8,16,32]:
            x,y=np.meshgrid(np.arange(640//stride),np.arange(640//stride))
            grids.append(np.stack([x,y],axis=-1).reshape(-1,2));strides.append(np.full((x.size,1),stride))
        self.grid=np.concatenate(grids);self.strides=np.concatenate(strides)
    def infer(self,bgr: np.ndarray) -> list:
        h,w=bgr.shape[:2];scale=min(640/w,640/h)
        image=np.full((640,640,3),114,dtype=np.uint8)
        image[:int(h*scale),:int(w*scale)]=cv2.resize(bgr,(int(w*scale),int(h*scale)))
        # Zoo demo uses raw RGB values, top-left letterbox, no normalization.
        blob=cv2.dnn.blobFromImage(image,1.,(640,640),swapRB=True)
        self.net.setInput(blob);out=self.net.forward()[0]
        xy=(out[:,:2]+self.grid)*self.strides;wh=np.exp(out[:,2:4])*self.strides
        classes=np.argmax(out[:,5:],axis=1);scores=out[:,4]*out[np.arange(len(out)),classes+5]
        boxes=np.concatenate([xy-wh/2,wh],axis=1)/scale
        ids=cv2.dnn.NMSBoxesBatched(boxes.tolist(),scores.tolist(),classes.tolist(),self.confidence,.5)
        names={0:'person',1:'bicycle',2:'car',3:'motorcycle',5:'bus',7:'truck'};result=[]
        for i in np.asarray(ids).flatten():
            c=int(classes[i])
            if c not in names:continue
            x,y,bw,bh=map(float,boxes[i]);box=[max(0.,x),max(0.,y),min(float(w),x+bw),min(float(h),y+bh)]
            if box[2]>box[0] and box[3]>box[1]:result.append(dict(box_xyxy=box,class_id=c,class_name=names[c],confidence=float(scores[i])))
        return result


def candidates(frame: dict,calibration: dict) -> list:
    """Geometry evaluation only. Never passed into Detector or Tracker."""
    result=[];near=calibration['near_m']
    for actor in frame['actors']:
        pixels=[];unresolved=False
        for component in actor['footprint_components']:
            points=component['world_vertices_m']
            if points is None:unresolved=True;continue
            camera=[transform_point(p,frame['camera']['world_to_camera_matrix']) for p in points]
            # Preserve explicit unavailable status for near-plane-straddling boxes;
            # do not project behind-camera vertices into fabricated image boxes.
            if any(p[1]<=near for p in camera):unresolved=True;continue
            for p in camera:
                ndc=transform_point(p,calibration['projection_matrix']);pixels.append([(ndc[0]+1)*320,(1-ndc[1])*180])
        box=None
        if pixels and not unresolved:
            box=[max(0.,min(p[0] for p in pixels)),max(0.,min(p[1] for p in pixels)),min(640.,max(p[0] for p in pixels)),min(360.,max(p[1] for p in pixels))]
            if box[2]<=box[0] or box[3]<=box[1]:box=None
        result.append(dict(actor_id=actor['actor_id'],role=actor['role'],box_xyxy=box,
            status='projected_candidate_not_visible_ground_truth' if box else 'unavailable_or_outside',
            reason='near_plane_or_unsupported' if unresolved else 'unbuffered_envelope_projection_clipped_to_image'))
    return result


def run(source: Path,output: Path,weights: Path,stride: int=1) -> dict:
    if stride<1:raise ValueError('Positive integer sampling stride required')
    detector=Detector(weights);tracker=Tracker();output.mkdir(parents=True,exist_ok=False)
    images=sorted((source/'camera/frames').glob('*.png'))[::stride]
    if not images:raise ValueError('No PNG frames')
    geometry_path=source/'geometry/geometry_frames.jsonl'
    geometry={f['step']:f for f in [json.loads(s) for s in geometry_path.read_text().splitlines()]} if geometry_path.exists() else {}
    calibration=json.loads((source/'geometry/camera_calibration.json').read_text()) if geometry else None
    dt=json.loads((source/'geometry/geometry_manifest.json').read_text())['dt_s'] if geometry else .1
    associations=[];total=0;tracked=0;index=[]
    with (output/'camera_observations.jsonl').open('w') as observations,(output/'projection_candidates.jsonl').open('w') as projected:
        for path in images:
            step=int(path.stem);time=step*dt;image=cv2.imread(str(path))
            if image is None:raise ValueError('Cannot decode '+str(path))
            detections=detector.infer(image);tracks=tracker.update(detections,step,time)
            total+=len(detections);tracked=max(tracked,tracker.next_id-1)
            observations.write(json.dumps(dict(step=step,time_s=time,image_sha256=sha(path),detections=detections,tracks=tracks,source='RGB_only'))+'\n')
            labels=candidates(geometry[step],calibration) if geometry else []
            projected.write(json.dumps(dict(step=step,actors=labels))+'\n')
            # Greedy one-to-one proxy association, evaluation only.
            pairs=sorted([(iou(t['box_xyxy'],a['box_xyxy']),t['track_id'],a['actor_id']) for t in tracks for a in labels if a['box_xyxy']],reverse=True)
            used_t=set();used_a=set()
            for score,tid,aid in pairs:
                if score>=.5 and tid not in used_t and aid not in used_a:
                    associations.append(dict(step=step,track_id=tid,actor_id=aid,iou=score));used_t.add(tid);used_a.add(aid)
            if step%20==0:
                for t in tracks:
                    x1,y1,x2,y2=map(int,t['box_xyxy']);cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),1)
                    cv2.putText(image,str(t['track_id'])+' '+t['class_name'],(x1,max(12,y1)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
                cv2.imwrite(str(output/f'overlay_{step:06d}.jpg'),image)
            index.append(dict(step=step,image=str(path.resolve())))
    switches=sum(a['track_id']!=b['track_id'] for aid in {a['actor_id'] for a in associations}
                 for a,b in zip([a for a in associations if a['actor_id']==aid],[a for a in associations if a['actor_id']==aid][1:]))
    report=dict(frames=len(images),detections=total,created_track_ids=tracked,matched_projected_candidate_samples=len(associations),
        proxy_actor_track_changes=switches,association_basis='IoU>=.5 projected envelope candidates; not visible-object GT or certified ID switches',
        detection_AP=None,tracking_IDF1=None,event_F1=None,
        unavailable_reason='No independently verified visible-object or camera-event annotation dataset',
        model='OpenCV Zoo YOLOX-S 2022nov',weights_sha256=sha(weights),opencv=cv2.__version__,confidence=.35,nms_iou=.5,
        source=str(source.resolve()),inference_inputs='RGB only; no simulator IDs/state used in detection/tracking',risk_assessment='unassessed')
    (output/'report.json').write_text(json.dumps(report,indent=2));(output/'associations.json').write_text(json.dumps(associations))
    (output/'review.html').write_text('<meta charset="utf-8"><h1>RGB baseline inspection</h1><p>Green boxes: detector/tracker. No human confirmation implied.</p>'+''.join(f'<h3>{p.stem}</h3><img src="{p.name}">' for p in output.glob('overlay*.jpg')))
    return report
