"""Offline camera-only temporal evidence. Inputs contain no simulator actor state."""
import argparse
import json
from collections import defaultdict
from itertools import combinations
from math import dist, isfinite
from pathlib import Path

from aad_system.vision import sha
from .camera import write
from .camera_events import crossings, decelerations, passes
from .events import lines


def temporal(camera, vision, output, lane_divider_y=None, *, extended=False, zone=None):
    if lane_divider_y is not None and not isfinite(lane_divider_y):
        raise ValueError('Finite surveyed lane divider required')
    if zone is not None and (len(zone)!=4 or not all(map(isfinite,zone)) or zone[0]>=zone[2] or zone[1]>=zone[3]):
        raise ValueError('Zone requires xmin ymin xmax ymax in road-plane coordinates')
    report = json.loads((vision/'report.json').read_text())
    source = camera/'observations.jsonl'
    if report['source_manifest_sha256'] != sha(source) or report['calibration_sha256'] != sha(camera/'calibration.json'):
        raise ValueError('Camera lineage mismatch')
    manifest = lines(source)[:report.get('capture_limit')]
    captured = lines(vision/'camera_observations.jsonl')
    if [(r['step'],r['time_s'],r['image_sha256']) for r in manifest] != [
            (r['step'],r['time_s'],r['image_sha256']) for r in captured]:
        raise ValueError('Camera-only observations are missing or out of sync')
    if any(b['step'] <= a['step'] or b['time_s'] <= a['time_s'] for a,b in zip(captured,captured[1:])):
        raise ValueError('Capture steps and times must increase')
    cal = json.loads((camera/'calibration.json').read_text())
    output.mkdir(parents=True, exist_ok=False)
    series = defaultdict(list)
    motor_tracks = set()
    split = defaultdict(list)
    pair_history = []
    last_pairs = {}
    previous = {}
    for index, frame in enumerate(captured):
        current = {}
        for row in frame['tracks']:
            track = row['track_id']
            if track in current:
                raise ValueError('Repeated track in one capture')
            x1,y1,x2,y2 = row['box_xyxy']
            point = row['estimated_ground_contact_world_m']
            if row['class_name'] in {'car','bus','truck'}:
                motor_tracks.add(track)
            valid = point is not None and x1>0 and y1>0 and x2<cal['width'] and y2<cal['height']
            old = previous.get(track)
            velocity = None
            if valid and old and old['xy_m'] is not None:
                dt = frame['time_s'] - old['time_s']
                velocity = [(a-b)/dt for a,b in zip(point[:2],old['xy_m'])]
            sample = dict(step=frame['step'], time_s=frame['time_s'], xy_m=point[:2] if valid else None,
                          velocity_xy_m_s=velocity, capture_index=index,
                          position_status='box_contact_proxy' if valid else 'projection_missing_or_boundary_box',
                          velocity_status='adjacent_capture_contact_difference' if velocity is not None else 'adjacent_valid_capture_missing')
            series[track].append(sample)
            current[track] = sample
            if lane_divider_y is not None and row['class_name'] in {'car','bus','truck'} and valid:
                # A survey-supplied divider on a straight, x-aligned road.
                lateral = point[1]-lane_divider_y
                lane = -1 if lateral < -.5 else 1 if lateral > .5 else 0
                split[track].append((index, lane))
        pairs = {}
        for a,b in combinations(sorted(current),2):
            x,y = (current[t]['xy_m'] for t in (a,b))
            if x is None or y is None:
                continue
            key = (a,b)
            previous_pair = last_pairs.get(key)
            separation = dist(x,y)
            closing = None
            if previous_pair:
                closing = (previous_pair['separation_proxy_m']-separation)/(frame['time_s']-previous_pair['time_s'])
            pairs[key] = dict(tracks=key, step=frame['step'], time_s=frame['time_s'], capture_index=index,
                              separation_proxy_m=separation, closing_proxy_m_s=closing,
                              closing_status='adjacent_pair_observed' if closing is not None else 'adjacent_pair_missing')
        pair_history.extend(pairs.values())
        previous,last_pairs = current,pairs
    events = []
    for track, lanes in split.items():
        for i in range(3,len(lanes)-2):
            left = lanes[i-3:i]
            if left[0][1] == 0 or len({v for _,v in left}) != 1:
                continue
            j = i
            while j < min(i+10,len(lanes)) and lanes[j][1] == 0:
                j += 1
            right = lanes[j:j+3]
            if len(right) != 3 or right[0][1] != -left[0][1]:
                continue
            if len({v for _,v in right}) != 1:
                continue
            lo,hi = left[0][0],right[-1][0]
            if hi-lo != j-i+5:
                continue
            contexts = defaultdict(list)
            for pair in pair_history:
                if track in pair['tracks'] and lo < pair['capture_index'] <= hi:
                    contexts[pair['tracks']].append(pair)
            closing_context = []
            for key, samples in contexts.items():
                run = 0
                for pair in samples:
                    if pair['closing_proxy_m_s'] is not None and pair['closing_proxy_m_s'] > 0:
                        run += 1
                    else:
                        run = 0
                    if run == 3:
                        closing_context.append(key)
                        break
            events.append(dict(track_id=track, family='A', start_step=manifest[lo]['step'], end_step=manifest[hi]['step'],
                               physical_behaviour='provisional_lane_side_transition', attention_context='unassessed',
                               interaction_evidence=dict(closing_pair_tracks=closing_context,
                                                         interpretation='contact-point separation trend'),
                               evasive_response='unassessed', observed_outcome='unknown',
                               review_status='candidate_for_review' if closing_context else 'insufficient_review_evidence',
                               abnormality_assessment='unassessed'))
    evaluated=['A'] if lane_divider_y is not None else []
    motor_series={k:v for k,v in series.items() if k in motor_tracks}
    if extended and lane_divider_y is not None:
        events.extend(passes(events.copy(),motor_series,lane_divider_y))
        events.extend(decelerations(motor_series,pair_history))
        evaluated.extend(['B','C'])
    if zone is not None:
        events.extend(crossings(motor_series,zone))
        evaluated.append('D')
    tracks=[]
    for track,samples in series.items():
        indices=[row['capture_index'] for row in samples]
        tracks.append(dict(track_id=track,samples=samples, observations=len(samples),
                           missing_captures_between_first_and_last=indices[-1]-indices[0]+1-len(indices),
                           identity_status='unverified'))
    write(output/'evidence.json', dict(tracks=tracks,pair_samples=pair_history,events=events,frames=len(captured),
          source_sha256=dict(observations=sha(vision/'camera_observations.jsonl'),manifest=sha(source),
                             calibration=report['calibration_sha256']), lane_divider_world_y_m=lane_divider_y,
          method='Planar box-contact proxies, adjacent observations only. A review trigger needs a divider transition and three successive observed closing increments.',
          event_status='experimental_camera_hypotheses' if evaluated else 'road_layout_unavailable',
          evaluated_hypothesis_families=evaluated, zone_world_bounds_m=zone,
          unscored_families=[family for family in 'ABCD' if family not in evaluated],
          subject_identity='unknown', outcome='unknown'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('camera','vision','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--lane-divider-y',type=float)
    parser.add_argument('--extended',action='store_true')
    parser.add_argument('--zone',type=float,nargs=4)
    args=parser.parse_args()
    temporal(args.camera,args.vision,args.output,args.lane_divider_y,extended=args.extended,zone=args.zone)
