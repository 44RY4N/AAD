"""Separate simulator-reference diagnostics. Projected envelopes are not visible GT."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import math
from aad_system.vision import iou, sha
from experiments.geometry_math import transform_point
from .camera import write
from .events import lines


def project_actor(actor, calibration):
    pixels=[]
    for component in actor['footprint_components']:
        vertices=component['world_vertices_m']
        if vertices is None:return dict(box_xyxy=None,reason='unsupported_geometry')
        camera=[transform_point(p,calibration['world_to_camera_matrix']) for p in vertices]
        if any(p[1]<=calibration['near_m'] for p in camera):
            return dict(box_xyxy=None,reason='near_plane_straddling_or_behind')
        for point in camera:
            ndc=transform_point(point,calibration['projection_matrix'])
            pixels.append([(ndc[0]+1)*calibration['width']/2,(1-ndc[1])*calibration['height']/2])
    raw=[min(p[0] for p in pixels),min(p[1] for p in pixels),max(p[0] for p in pixels),max(p[1] for p in pixels)]
    clipped=[max(0.,raw[0]),max(0.,raw[1]),min(float(calibration['width']),raw[2]),min(float(calibration['height']),raw[3])]
    if clipped[2]<=clipped[0] or clipped[3]<=clipped[1]:return dict(box_xyxy=None,reason='outside_image')
    area=(clipped[2]-clipped[0])*(clipped[3]-clipped[1])
    return dict(box_xyxy=clipped,unclipped_box_xyxy=raw,truncated=raw!=clipped,area_px2=area,
        size_bin='small' if area<32**2 else 'medium' if area<96**2 else 'large',
        reason='projected_collision_envelope_not_visible_silhouette',occlusion='unknown')


def run(source,camera_folder,vision,output,event_path=None):
    output.mkdir(parents=True,exist_ok=False)
    calibration=json.loads((camera_folder/'calibration.json').read_text())
    geometry={f['step']:f for f in lines(source/'geometry/geometry_frames.jsonl')}
    observations=lines(vision/'camera_observations.jsonl')
    manifest={r['step']:r for r in lines(camera_folder/'observations.jsonl')}
    event=json.loads(event_path.read_text()) if event_path else None
    records=[];matches=[];history=defaultdict(list);eligible=defaultdict(int);event_eligible=defaultdict(int);event_matches=defaultdict(int)
    previous={}
    previous_step=-1
    capture_index={step:i for i,step in enumerate(manifest)}
    for obs in observations:
        frame=geometry[obs['step']];projections=[]
        record=manifest[obs['step']]
        if (obs['step'] <= previous_step or frame['time_s'] != obs['time_s']
                or record['time_s'] != obs['time_s'] or record['image_sha256'] != obs['image_sha256']):
            raise ValueError('Observations do not align with the camera and reference manifests')
        previous_step=obs['step']
        in_event=event is not None and event['start_step'] is not None and event['start_step']<=obs['step']<=event['end_step']
        for a in frame['actors']:
            p=dict(actor_id=a['actor_id'],role=a['role'],**project_actor(a,calibration));projections.append(p)
            if p['box_xyxy']:
                eligible[a['role']]+=1
                if in_event:event_eligible[a['role']]+=1
        pairs=sorted([(iou(t['box_xyxy'],p['box_xyxy']),i,j) for i,t in enumerate(obs['tracks']) for j,p in enumerate(projections) if p['box_xyxy'] and t['class_name'] in {'car','truck','bus'}],reverse=True)
        used_t=set();used_a=set()
        for overlap,i,j in pairs:
            if overlap<.5 or i in used_t or j in used_a:continue
            used_t.add(i);used_a.add(j);t=obs['tracks'][i];p=projections[j]
            a=next(a for a in frame['actors'] if a['actor_id']==p['actor_id'])
            estimate=t.get('estimated_ground_contact_world_m')
            box=t['box_xyxy']
            if box[0]<=0 or box[1]<=0 or box[2]>=calibration['width'] or box[3]>=calibration['height']:
                estimate=None
                position_status='box_touches_image_boundary'
            else:
                position_status='different_reference_points' if estimate is not None else 'ground_contact_estimate_unavailable'
            reference=a['position_world_m'][:2]
            offset=math.dist(estimate[:2],reference) if estimate is not None else None
            match=dict(step=obs['step'],role=p['role'],reference_actor_id=p['actor_id'],track_id=t['track_id'],
                projected_envelope_iou=overlap,ground_contact_vs_actor_reference_offset_m=offset,
                time_s=obs['time_s'],estimated_contact_xy_m=estimate[:2] if estimate is not None else None,
                reference_xy_m=reference,position_status=position_status,
                offset_scope='Different physical reference points; diagnostic discrepancy, not independent trajectory accuracy')
            # Require two successive captures associated to the same actor and track.
            key=(p['actor_id'],t['track_id'])
            old=previous.get(key)
            match.update(speed_discrepancy_m_s=None,estimated_speed_m_s=None,reference_chord_speed_m_s=None,
                         speed_status='no_adjacent_valid_same_track_association')
            if (old and capture_index[obs['step']]-capture_index[old['step']]==1 and old['estimated_contact_xy_m'] is not None
                    and estimate is not None):
                dt=obs['time_s']-old['time_s']
                if dt <= 0:
                    raise ValueError('Non-increasing capture time')
                speed=math.dist(estimate[:2],old['estimated_contact_xy_m'])/dt
                reference_speed=math.dist(reference,old['reference_xy_m'])/dt
                match.update(estimated_speed_m_s=speed,reference_chord_speed_m_s=reference_speed,
                             speed_discrepancy_m_s=abs(speed-reference_speed),
                             speed_status='different_reference_points')
            previous[key]=match
            matches.append(match);history[p['role']].append(match)
            if in_event:event_matches[p['role']]+=1
        overlaps=[dict(roles=[a['role'],b['role']],projected_box_iou=iou(a['box_xyxy'],b['box_xyxy']))
            for i,a in enumerate(projections) for b in projections[i+1:] if a['box_xyxy'] and b['box_xyxy'] and iou(a['box_xyxy'],b['box_xyxy'])>0]
        records.append(dict(step=obs['step'],time_s=obs['time_s'],reference_projections=projections,
            projected_box_overlaps=overlaps,note='Overlap indicates possible occlusion, not established pixel visibility'))
    with (output/'reference_projections.jsonl').open('w',encoding='utf-8') as f:
        for r in records:f.write(json.dumps(r,allow_nan=False)+'\n')
    write(output/'reference_associations.json',matches)
    offsets=[m['ground_contact_vs_actor_reference_offset_m'] for m in matches if m['ground_contact_vs_actor_reference_offset_m'] is not None]
    speeds=[m['speed_discrepancy_m_s'] for m in matches if m['speed_discrepancy_m_s'] is not None]
    report=dict(inference_records_read_only=True,association='Greedy one-to-one IoU >= .5 against collision-envelope projections; car, bus and truck output classes; diagnostic',
        position_pairs=len(offsets),speed_pairs=len(speeds),
        position_discrepancy_rmse_m=math.sqrt(sum(x*x for x in offsets)/len(offsets)) if offsets else None,
        speed_discrepancy_mae_m_s=sum(speeds)/len(speeds) if speeds else None,
        camera_frames=len(manifest),evaluated_frames=len(observations),
        input_sha256=dict(camera_manifest=sha(camera_folder/'observations.jsonl'),
            observations=sha(vision/'camera_observations.jsonl'),calibration=sha(camera_folder/'calibration.json'),
            reference=sha(source/'geometry/geometry_frames.jsonl')),
        projected_in_image_samples=dict(eligible),matched_projected_samples={r:len(h) for r,h in history.items()},
        event_projected_samples=dict(event_eligible),event_matched_projected_samples=dict(event_matches),
        proxy_track_changes={r:sum(a['track_id']!=b['track_id'] for a,b in zip(h,h[1:])) for r,h in history.items()},
        event_window=[event['start_step'],event['end_step']] if event else None,
        certified_id_switches=None,precision=None,recall=None,IDF1=None,
        limitation='Geometry does not establish rendered visibility, independently correct class, identity or bbox localization; no detector performance denominators certified')
    write(output/'report.json',report);print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__)
    for key in ['source','camera','vision','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--event',type=Path);a=p.parse_args();run(a.source,a.camera,a.vision,a.output,a.event)
