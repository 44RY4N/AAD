"""Tag lane changes and nearby receiving-lane context in simulated CCTV MP4s."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import cv2
import numpy as np

from aad_system.vision import Tracker, sha
from .camera import ground_point, write
from .vision import observations


RULES = dict(road_divider_y_m=5.25, lane_deadband_m=.40, stable_frames=3,
             nearby_longitudinal_m=9.0, context_frames=3, proxy_surface_z_m=.65,
             red_min=58, red_channel_margin=23, min_component_px=36)


def writer(path, fps, size):
    stream=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),fps,size)
    if not stream.isOpened():
        raise OSError(f'Cannot encode {path}')
    return stream


def export(camera, output):
    """Write an MP4 without reference data. Capture timestamps are uniformly spaced."""
    setup=json.loads((camera/'camera.json').read_text())
    stream=writer(output,setup['fps'],(setup['width'],setup['height']))
    try:
        for _,path in observations(camera):
            frame=cv2.imread(str(path))
            if frame is None:
                raise OSError(f'Cannot decode {path}')
            stream.write(frame)
    finally:
        stream.release()


def red_proxies(frame):
    b,g,r=frame.astype(np.int16).transpose(2,0,1)
    mask=(r>=RULES['red_min']) & (r-g>=RULES['red_channel_margin']) & (r-b>=RULES['red_channel_margin'])
    mask=cv2.morphologyEx(mask.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    _,_,stats,centres=cv2.connectedComponentsWithStats(mask,8)
    for (x,y,w,h,pixels),centre in zip(stats[1:],centres[1:]):
        if (pixels>=RULES['min_component_px'] and w>=5 and h>=4 and
                x>0 and y>0 and x+w<frame.shape[1] and y+h<frame.shape[0]):
            yield dict(class_id=2,class_name='red_proxy',confidence=1., # tracker weight only
                       box_xyxy=[int(x),int(y),int(x+w),int(y+h)], roof_pixel=centre.tolist())


def tag_tracks(frames):
    series=defaultdict(list)
    for n,tracks in enumerate(frames):
        for track in tracks:
            if track['xy_m'] is not None:
                series[track['track_id']].append((n,*track['xy_m']))
    divider=RULES['road_divider_y_m']
    dead=RULES['lane_deadband_m']
    support=RULES['stable_frames']
    events=[]
    for identity,points in series.items():
        stable_side=run=0
        prior=None
        origin=None
        transition=None
        for i,(n,x,y) in enumerate(points):
            if prior is None or n!=prior+1:
                stable_side=run=0
                origin=transition=None
            prior=n
            side=-1 if y<divider-dead else 1 if y>divider+dead else 0
            if side==0:
                stable_side=run=0
                if origin is not None and transition is None:
                    transition=n
                continue
            run=run+1 if side==stable_side else 1
            stable_side=side
            if origin is None and run>=support:
                origin=side
            if side==origin:
                transition=None
            if origin is None or side==origin:
                continue
            if transition is None:
                transition=n
            if run<support:
                continue
            lo=transition
            evidence=defaultdict(list)
            previous_tracks={}
            for frame in range(lo,n+1):
                current=next(t for t in frames[frame] if t['track_id']==identity)
                ax,ay=current['xy_m']
                for other in frames[frame]:
                    tid=other['track_id']
                    point=other['xy_m']
                    if tid==identity or point is None or side*(point[1]-divider)<=dead:
                        continue
                    distance=abs(point[0]-ax)
                    if distance<=RULES['nearby_longitudinal_m']:
                        if previous_tracks.get(tid)!=frame-1:
                            evidence[tid]=[]
                        evidence[tid].append((frame,distance))
                        previous_tracks[tid]=frame
            pairs=[dict(track_id=tid,consecutive_frames=len(values),
                        minimum_longitudinal_proxy_m=min(v for _,v in values))
                   for tid,values in evidence.items() if len(values)>=RULES['context_frames']]
            events.append(dict(track_id=identity,family='A',start_frame=lo,end_frame=n,
                               physical_behaviour='lane_change',attention_context='nearby_receiving_lane_vehicle' if pairs else 'insufficient_context_for_flag',
                               interaction_evidence=pairs,evasive_response='unassessed',observed_outcome='unknown',
                               tag='candidate_AAD_review' if pairs else 'lane_change_only',abnormality_assessment='unassessed'))
            origin=side
            transition=None
    return sorted(events,key=lambda row:(row['start_frame'],row['track_id']))


def infer(video, calibration_path, output, overlay=True):
    calibration=json.loads(calibration_path.read_text())
    calibration['ground_plane_world_z_m']=RULES['proxy_surface_z_m']
    capture=cv2.VideoCapture(str(video))
    fps=capture.get(cv2.CAP_PROP_FPS)
    if not capture.isOpened() or fps<=0:
        capture.release()
        raise OSError(f'Cannot decode {video}')
    output.mkdir(parents=True,exist_ok=False)
    tracker=Tracker(gate=.15)
    frames=[]
    try:
        while True:
            ok,frame=capture.read()
            if not ok:
                break
            if (frame.shape[1],frame.shape[0])!=(calibration['width'],calibration['height']):
                raise ValueError('Video size differs from calibration')
            tracks=tracker.update(list(red_proxies(frame)),len(frames),len(frames)/fps)
            for track in tracks:
                position,_=ground_point(track['roof_pixel'],calibration)
                track['xy_m']=position[:2] if position is not None else None
            frames.append(tracks)
    finally:
        capture.release()
    if not frames:
        raise ValueError('No decoded frames')
    tags=tag_tracks(frames)
    for tag in tags:
        tag['start_s']=tag['start_frame']/fps
        tag['end_s']=tag['end_frame']/fps
    report=dict(frames=len(frames),fps=fps,video_sha256=sha(video),calibration_sha256=sha(calibration_path),
                code_sha256=sha(Path(__file__)),rules=RULES,tags=tags,
                scope='Simulated red ordinary vehicles, static road and fixed calibrated CCTV. Review tag is provisional.')
    write(output/'report.json',report)
    write(output/'tracks.json',frames)
    if overlay:
        annotate(video,output/'tagged.mp4',frames,tags,fps,(calibration['width'],calibration['height']))
    print(json.dumps(dict(frames=len(frames),tags=tags),indent=2))


def annotate(video, path, frames, tags, fps, size):
    capture=cv2.VideoCapture(str(video))
    stream=writer(path,fps,size)
    try:
        for i,tracks in enumerate(frames):
            ok,frame=capture.read()
            if not ok:
                raise OSError('Input changed during overlay replay')
            active={tag['track_id']:tag for tag in tags if tag['start_frame']<=i<=tag['end_frame']+round(fps*2)}
            for track in tracks:
                identity=track['track_id']
                tag=active.get(identity)
                flagged=tag is not None and tag['tag']=='candidate_AAD_review'
                color=(0,165,255) if flagged else (125,220,85)
                x1,y1,x2,y2=map(int,track['box_xyxy'])
                cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)
                label=f'ID {identity}'
                if tag:
                    label+=' | Candidate AAD: lane change' if flagged else ' | Lane change'
                cv2.putText(frame,label,(x1,max(15,y1-7)),cv2.FONT_HERSHEY_SIMPLEX,.6,color,2)
            cv2.putText(frame,f'Fixed CCTV | {i/fps:.1f} s',(18,29),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
            stream.write(frame)
    finally:
        stream.release()
        capture.release()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    tasks=parser.add_subparsers(dest='action',required=True)
    exp=tasks.add_parser('export')
    exp.add_argument('--camera',type=Path,required=True)
    exp.add_argument('--output',type=Path,required=True)
    run=tasks.add_parser('infer')
    for option in ('video','calibration','output'):
        run.add_argument('--'+option,type=Path,required=True)
    args=parser.parse_args()
    if args.action=='export':
        if args.output.exists():
            raise FileExistsError(args.output)
        export(args.camera,args.output)
    else:
        infer(args.video,args.calibration,args.output)
