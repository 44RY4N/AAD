"""Video-only manoeuvre and review tags for a calibrated straight road or junction."""
import argparse
from collections import defaultdict
from itertools import combinations
import json
from pathlib import Path
from statistics import median

import cv2

from aad_system.vision import sha
from .camera import write
from .roadside_video_tags import in_road_strip
from .video_tags import RULES, tag_tracks, writer


ZONE=[95.25,1.75,98.75,5.25]
METHOD=dict(road_strip_y_m=[1.75,8.75],intersection_zone_world_m=ZONE,
            deceleration_drop_m_s=3,braking_lead_window_m=30,
            crossing_review_gap_s=1,pass_order_margin_m=3)


def hypothesis(family,identity,start,end,behavior,evidence,review=False):
    return dict(family=family,track_id=identity,start_frame=start,end_frame=end,
                physical_behaviour=behavior,attention_context='review_context_observed' if review else 'unassessed',
                interaction_evidence=evidence,
                tag='candidate_AAD_review' if review else behavior,
                evasive_response='unassessed',observed_outcome='unknown',abnormality_assessment='unassessed')


def tracks_by_id(frames):
    series=defaultdict(dict)
    for i,tracks in enumerate(frames):
        for t in tracks:
            if t['xy_m'] is not None:
                series[t['track_id']][i]=t['xy_m']
    return series


def overtaking(lane_tags,series):
    changes=defaultdict(list)
    for event in lane_tags:
        changes[event['track_id']].append(event)
    divider=RULES['road_divider_y_m']
    for identity,events in changes.items():
        motion=series[identity]
        for first,last in zip(events,events[1:]):
            start=max(min(motion),first['start_frame']-4)
            end=last['end_frame']
            if any(i not in motion for i in range(start,end+1)):
                continue
            side=lambda y:y<divider
            origin=side(motion[start][1])
            if side(motion[end][1])!=origin or motion[end][0]<=motion[start][0]:
                continue
            observations=[]
            for other,track in series.items():
                if other==identity or start not in track or end not in track:
                    continue
                if side(track[start][1])!=origin or side(track[end][1])!=origin or track[end][0]<=track[start][0]:
                    continue
                shared=[i for i in range(start,end+1) if i in track and side(track[i][1])==origin]
                if len(shared)<.8*(end-start+1):
                    continue
                distances=[track[i][0]-motion[i][0] for i in shared]
                if distances[0]>METHOD['pass_order_margin_m']:
                    observations.append((distances[0],other,distances))
            if not observations:
                continue
            before,other,deltas=min(observations,key=lambda obs:obs[0])
            after=deltas[-1]
            if after < -METHOD['pass_order_margin_m']:
                label='pass_and_return'
            elif min(deltas)>0 and min(deltas)<before-3 and after>METHOD['pass_order_margin_m']:
                label='return_behind_after_excursion'
            else:
                continue
            yield hypothesis('B',identity,start,end,label,dict(counterpart_track=other,
                initial_longitudinal_order_m=before,final_longitudinal_order_m=after,
                minimum_absolute_longitudinal_proxy_m=min(map(abs,deltas))))


def braking(series,fps,window=12):
    for identity,motion in series.items():
        previous_end=-1
        for end in motion:
            start=end-window
            if start<=previous_end or any(i not in motion for i in range(start,end+1)):
                continue
            points=[motion[i] for i in range(start,end+1)]
            if max(y for _,y in points)-min(y for _,y in points)>.8:
                continue
            if window==12:
                before=fps*median(points[i+1][0]-points[i][0] for i in range(4))
                after=fps*median(points[i+1][0]-points[i][0] for i in range(8,12))
            else:
                span=window//3
                before=fps*(points[span][0]-points[0][0])/span
                after=fps*(points[-1][0]-points[-span-1][0])/span
            if after<0 or before-after<METHOD['deceleration_drop_m_s']:
                continue
            lead=[]
            for other,track in series.items():
                if other==identity or any(i not in track for i in range(start,end+1)):
                    continue
                gaps=[]
                for i in range(start,end+1):
                    x,y=motion[i]
                    ax,ay=track[i]
                    if abs(y-ay)>1.2 or ax<=x:
                        break
                    gaps.append(ax-x)
                if (len(gaps)==window+1 and gaps[0]-gaps[-1]>=1.5 and
                        sum(g<=METHOD['braking_lead_window_m'] for g in gaps[-3:])==3):
                    lead.append((gaps[-1],other,gaps[0]))
            context=dict(initial_speed_proxy_m_s=before,final_speed_proxy_m_s=after)
            if lead:
                final,other,initial=min(lead)
                context.update(counterpart_track=other,initial_longitudinal_proxy_m=initial,final_longitudinal_proxy_m=final)
            previous_end=end
            yield hypothesis('C',identity,start,end,'deceleration',context,review=bool(lead))


def crossing(series,fps):
    xmin,ymin,xmax,ymax=ZONE
    visits=[]
    for identity,motion in series.items():
        inside=[i for i,(x,y) in motion.items() if xmin<x<xmax and ymin<y<ymax]
        if len(inside)<2 or len(inside)!=inside[-1]-inside[0]+1:
            continue
        start,end=inside[0],inside[-1]
        if any(i not in motion for i in range(start-2,end+3)):
            continue
        x,y=motion[start-2]
        ax,ay=motion[end+2]
        dx,dy=abs(ax-x),abs(ay-y)
        if max(dx,dy)<2 or min(dx,dy)*2>max(dx,dy):
            continue
        visits.append((start,end,identity,dx>dy))
    for first,last in combinations(sorted(visits),2):
        if first[3]==last[3]:
            continue
        gap=max(0,last[0]-first[1])/fps
        yield hypothesis('D',last[2],first[0],max(first[1],last[1]),'crossing_interaction',
                         dict(counterpart_track=first[2],sampled_zone_gap_proxy_s=gap),
                         review=gap<=METHOD['crossing_review_gap_s'])


def tag(frames,fps,layout,braking_window=12):
    if layout=='intersection':
        return list(crossing(tracks_by_id(frames),fps)),frames
    frames=[[t for t in frame if in_road_strip(t)] for frame in frames]
    series=tracks_by_id(frames)
    lanes=tag_tracks(frames)
    tags=lanes+list(overtaking(lanes,series))+list(braking(series,fps,braking_window))
    return sorted(tags,key=lambda item:(item['start_frame'],item['family'])),frames


def render_tags(video,destination,frames,tags,fps,size):
    capture=cv2.VideoCapture(str(video))
    stream=writer(destination,fps,size)
    labels={'A':'Lane change','B':'Overtaking sequence','C':'Deceleration','D':'Crossing interaction'}
    try:
        for i,tracks in enumerate(frames):
            ok,image=capture.read()
            if not ok:
                raise OSError('Input video ended during overlay')
            active=[e for e in tags if e['start_frame']<=i<=e['end_frame']+round(fps)]
            marked={e['track_id'] for e in active}
            for track in tracks:
                x,y,u,v=track['box_xyxy']
                color=(0,165,255) if track['track_id'] in marked else (105,215,95)
                cv2.rectangle(image,(x,y),(u,v),color,2)
                cv2.putText(image,f"ID {track['track_id']}",(x,max(y-5,14)),cv2.FONT_HERSHEY_SIMPLEX,.5,color,2)
            cv2.putText(image,f'Fixed CCTV | {i/fps:.1f} s',(18,24),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
            display=sorted({(e['family'],e['tag']=='candidate_AAD_review') for e in active})
            for line,(family,review) in enumerate(display):
                description=labels[family]+(' | Candidate AAD review' if review else '')
                cv2.putText(image,description,(18,53+line*24),cv2.FONT_HERSHEY_SIMPLEX,.63,(0,175,255),2)
            stream.write(image)
    finally:
        stream.release()
        capture.release()


def run(video,calibration,source,output,layout,braking_window=12):
    report=json.loads((source/'report.json').read_text())
    if report['video_sha256']!=sha(video) or report['calibration_sha256']!=sha(calibration):
        raise ValueError('Video/calibration lineage does not match')
    frames=json.loads((source/'tracks.json').read_text())
    if len(frames)!=report['frames']:
        raise ValueError('Video/track frame count differs')
    tags,frames=tag(frames,report['fps'],layout,braking_window)
    for event in tags:
        event['start_s']=event['start_frame']/report['fps']
        event['end_s']=event['end_frame']/report['fps']
    output.mkdir(parents=True,exist_ok=False)
    write(output/'report.json',dict(report,tags=tags,static_layout=layout,method=METHOD,
                                 braking_window_frames=braking_window,
                                 tagger_sha256=sha(Path(__file__)),derived_from_code_sha256=report['code_sha256']))
    write(output/'tracks.json',frames)
    static=json.loads(calibration.read_text())
    render_tags(video,output/'tagged.mp4',frames,tags,report['fps'],(static['width'],static['height']))
    print(json.dumps(tags,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('video','calibration','source','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--layout',choices=['straight','intersection'],required=True)
    parser.add_argument('--braking-window',type=int,choices=[12,30],default=12)
    args=parser.parse_args()
    run(args.video,args.calibration,args.source,args.output,args.layout,args.braking_window)
