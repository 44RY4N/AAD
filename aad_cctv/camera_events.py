"""Provisional motion hypotheses from calibrated contact-point tracks."""
from collections import defaultdict
from itertools import combinations


def hypothesis(family, track, start, end, behaviour, evidence, review=False):
    return dict(family=family, track_id=track, start_step=start, end_step=end,
                physical_behaviour=behaviour, attention_context='unassessed',
                interaction_evidence=evidence, evasive_response='unassessed',
                observed_outcome='unknown', abnormality_assessment='unassessed',
                review_status='candidate_for_review' if review else 'insufficient_review_evidence')


def decelerations(series, pairs):
    """One sustained longitudinal decrease per overlapping set of valid windows."""
    for track, samples in series.items():
        last_end=-1
        for i in range(10,len(samples)):
            window=samples[i-10:i+1]
            if window[0]['capture_index']<=last_end or window[-1]['capture_index']-window[0]['capture_index']!=10:
                continue
            if any(s['xy_m'] is None for s in window):
                continue
            # Five two-capture velocities reduce single-frame box jitter.
            speeds=[(b['xy_m'][0]-a['xy_m'][0])/(b['time_s']-a['time_s'])
                    for a,b in zip(window[::2],window[2::2])]
            if not (all(v>0 for v in speeds) or all(v<0 for v in speeds)):
                continue
            speeds=list(map(abs,speeds))
            if speeds[0]-speeds[-1]<2 or not all(a>b for a,b in zip(speeds,speeds[1:])):
                continue
            start,end=window[0]['step'],window[-1]['step']
            last_end=window[-1]['capture_index']
            closing=[p for p in pairs if track in p['tracks'] and start<p['step']<=end and
                     p['closing_proxy_m_s'] is not None and p['closing_proxy_m_s']>0]
            yield hypothesis('C',track,start,end,'sustained_longitudinal_speed_drop_proxy',
                             dict(longitudinal_speed_proxies_m_s=speeds,
                                  closing_pair_samples=len(closing)))


def passes(events, series, divider):
    transitions=defaultdict(list)
    points={track:{s['step']:s for s in samples if s['xy_m'] is not None}
            for track,samples in series.items()}
    for event in events:
        transitions[event['track_id']].append(event)
    for track, changes in transitions.items():
        for first,second in zip(changes,changes[1:]):
            start,end=first['start_step'],second['end_step']
            subject=points[track]
            motion=[s for step,s in subject.items() if start<=step<=end]
            if motion[-1]['capture_index']-motion[0]['capture_index']+1!=len(motion):
                continue
            sx,sy=subject[start]['xy_m']
            ex,ey=subject[end]['xy_m']
            if (sy-divider)*(ey-divider)<=0 or abs(ex-sx)<1:
                continue
            direction=1 if ex>sx else -1
            for other, locations in points.items():
                if other==track or any(s['step'] not in locations for s in motion):
                    continue
                ax,ay=locations[start]['xy_m']
                bx,by=locations[end]['xy_m']
                if (ay-divider)*(sy-divider)<=0 or (by-divider)*(ey-divider)<=0:
                    continue
                before,after=direction*(ax-sx),direction*(bx-ex)
                if direction*(bx-ax)>0 and before>1 and after<-1:
                    yield hypothesis('B',track,start,end,'pass_and_return_contact_point_hypothesis',
                                     dict(counterpart_track=other, signed_order_before_m=before,
                                          signed_order_after_m=after))


def crossings(series, bounds):
    xmin,ymin,xmax,ymax=bounds
    inside=defaultdict(list)
    for track,samples in series.items():
        for s in samples:
            if s['xy_m'] is not None:
                x,y=s['xy_m']
                if xmin<x<xmax and ymin<y<ymax:
                    inside[s['capture_index']].append((track,s['step']))
    last={}
    for index, entries in sorted(inside.items()):
        for (track,step),(other,_) in combinations(sorted(entries),2):
            pair=(track,other)
            previous=last.get(pair)
            if previous and previous[0]==index-1 and not previous[2]:
                yield hypothesis('D',track,previous[1],step,'simultaneous_zone_contact_point_proxy',
                                 dict(counterpart_track=other, zone_world_bounds_m=bounds),review=True)
                last[pair]=(index,step,True)
            else:
                last[pair]=(index,step,bool(previous and previous[0]==index-1 and previous[2]))
