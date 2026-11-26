"""Bounded simulator-reference events; never a camera-only semantic classifier."""
import csv
import hashlib
import json
from pathlib import Path
from shapely.geometry import Polygon
from shapely.ops import unary_union
from .event_contract import FAMILIES, validate


def lines(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def confirm(flags, count=11):
    consecutive=0
    for step, flag in flags:
        consecutive=consecutive+1 if flag else 0
        if consecutive>=count:return step
    return None


def braking_window(samples, command_step):
    baseline=next((s['speed_kmh'] for s in samples if s['step']==command_step-1),None)
    if baseline is None:return None,None,False
    start=None
    for i in range(2,len(samples)):
        group=samples[i-2:i+1]
        if group[0]['step']>=command_step and all(s['speed_kmh']<baseline-.5 for s in group) and all(b['speed_kmh']<a['speed_kmh'] for a,b in zip(group,group[1:])):
            start=group[0]['step'];break
    if start is None:return None,None,False
    for i in range(10,len(samples)):
        group=samples[i-10:i+1]
        if group[0]['step']>start and max(s['speed_kmh'] for s in group)-min(s['speed_kmh'] for s in group)<.1:
            return start,group[-1]['step'],False
    return start,samples[-1]['step'],True


def bounds(actor):
    xs=[p[0] for c in actor['footprint_components'] for p in c['world_vertices_m']]
    return min(xs),max(xs)


def extract(source, feature_output):
    meta=json.loads((source/'metadata.json').read_text());settings=meta['settings'];dt=meta['dt_s']
    execution=json.loads((source/'cctv_execution.json').read_text());kind=execution['family']
    frames=lines(source/'geometry/geometry_frames.jsonl')
    frames_by_step={f['step']:f for f in frames}
    actors={a['role']:a['actor_id'] for a in frames[0]['actors']}
    family={'baseline_lane_change':'A','lane_change':'A','overtaking':'B','abort':'B','braking':'C','crossing':'D'}[kind]
    target='background_far' if kind=='lane_change' else 'crossing_target' if kind=='crossing' else 'leader'
    start=end=None;censored=False;detail={};response=dict(status='unassessed',evasive_intent='unassessed')
    subject=[dict(step=f['step'],**next(a for a in f['actors'] if a['role']=='subject')) for f in frames]
    departure=next((s['step'] for s in subject if abs(s['position_world_m'][1]-subject[0]['position_world_m'][1])>.35),None)
    if family=='A':
        result=meta['result'];start=result.get('departure_step',departure);end=result.get('confirmed_step')
        if start is not None and end is None:end=frames[-1]['step'];censored=True
        detail=dict(physical_verifier=result,boundary_definition=FAMILIES['A']['boundaries'])
    elif family=='B':
        start=departure
        if kind=='abort':
            decisions=json.loads((source/'decisions.json').read_text());trigger=next((r['abort_trigger'] for r in decisions if 'abort_trigger' in r),None)
            flags=[]
            for f in frames:
                a,b=[next(x for x in f['actors'] if x['role']==r) for r in ['subject','leader']]
                flags.append((f['step'],bool(trigger and f['step']>=trigger['step'] and abs(a['position_world_m'][1]-3.5)<.35 and bounds(a)[1]<bounds(b)[0])))
            end=confirm(flags)
            response=dict(status='controller_intent_recorded' if trigger else 'not_observed',abort_trigger=trigger,
                return_confirmed_step=end,evasive_intent='not_a_human_intent_claim')
            detail=dict(abort_completed=end is not None,definition='Explicit abort trigger plus 11 source-lane-tolerance samples while behind leader')
        else:
            from experiments.aad_overtake_v01.events import reconstruct
            overtake=reconstruct(source);end=overtake['return_confirmation_step']
            detail={k:v for k,v in overtake.items() if k!='phases'}
        if start is not None and end is None:end=frames[-1]['step'];censored=True
    elif family=='C':
        start,end,censored=braking_window(subject,settings['brake_step'])
        response=dict(status='actual_speed_response_observed' if start is not None else 'not_observed',
            command_step=settings['brake_step'],command_target_kmh=settings['leader_after_kmh'],
            subject_final_speed_kmh=subject[-1]['speed_kmh'],evasive_intent='unassessed')
        detail=dict(definition=FAMILIES['C']['boundaries'],scope='Subject response; leader speed command recorded separately')
    else:
        zone=json.loads((source/'conflict_zone.json').read_text());region=Polygon(zone['polygon_world_xy_m']);occupancies={r:[] for r in actors}
        for f in frames:
            for a in f['actors']:
                footprint=unary_union([Polygon(c['world_footprint_xy_m']) for c in a['footprint_components']])
                if footprint.intersection(region).area>1e-9:occupancies[a['role']].append(f['step'])
        combined=sorted(set(occupancies['subject']+occupancies[target]))
        if occupancies['subject']:
            start=combined[0];last=combined[-1];end=min(last+1,frames[-1]['step']);censored=last==frames[-1]['step']
        intervals={r:dict(first_occupied_step=s[0] if s else None,last_occupied_step=s[-1] if s else None) for r,s in occupancies.items()}
        overlap=sorted(set(occupancies['subject'])&set(occupancies[target]))
        temporal=None
        if occupancies['subject'] and occupancies[target] and not overlap:
            first,second=sorted([occupancies['subject'],occupancies[target]],key=lambda s:s[0])
            temporal=max(0,second[0]-(first[-1]+1))*dt
        detail=dict(zone=zone,occupancy_intervals=intervals,simultaneous_occupancy_steps=overlap,
            sampled_zone_clearance_s=temporal,clearance_note='Decision-sampled exit-to-entry interval; resolution .1 s, not continuous physical PET or risk')
    pairs=lines(feature_output/'pair_features.jsonl');window=[]
    if start is not None:
        window=[p for p in pairs if start<=p['step']<=end and set(p['actor_roles'])=={'subject',target}]
    minima={}
    for name in ['outer_envelope_planar_separation','shared_path_signed_longitudinal_envelope_gap','constant_velocity_fixed_orientation_envelope_ttc_2d']:
        valid=[(p,m) for p in window for m in p['metrics'] if m['name']==name and m['validity']=='valid' and m['value'] is not None]
        best=min(valid,key=lambda pair:pair[1]['value']) if valid else None
        minima[name]=dict(value=best[1]['value'] if best else None,step=best[0]['step'] if best else None,
            valid_samples=len(valid),window_samples=len(window),reason='minimum_over_valid_samples_only' if best else 'no_valid_event_window_samples')
    camera=[]
    if (source/'cctv_cameras.json').exists():
        for c in json.loads((source/'cctv_cameras.json').read_text()):
            folder=source/c['directory'];records=lines(folder/'frames.jsonl')
            samples=[r for r in records if start is not None and start<=r['step']<=end]
            camera.append(dict(camera_id=c['camera_id'],manifest=str((folder/'observations.jsonl').resolve()),
                first_step=samples[0]['step'] if samples else None,last_step=samples[-1]['step'] if samples else None,
                event_frames=len(samples),frustum_counts={role:sum(next(p for p in r['projections'] if p['role']==role)['in_frustum'] for r in samples) for role in ['subject',target]},
                occlusion_status='not_established_by_projection',continuous_trackability='unassessed'))
    raw=list(csv.DictReader((source/'trajectory.csv').open()))
    observed=start is not None
    event=dict(schema_version='aad-event-0.2-proposed',event_id=settings['scenario_id']+'_event_01',scenario_id=settings['scenario_id'],
        family=family,subject_role='subject',target_role=target,surrounding_roles=[r for r in actors if r not in ['subject',target]],
        actor_scope='simulator_reference_roles_only',start_step=start,end_step=end,dt_s=dt,start_s=start*dt if observed else None,end_s=end*dt if observed else None,
        censored=censored,physical_behaviour='detected' if observed else 'not_observed',
        attention_context=dict(status='unassessed',configured_family=FAMILIES[family]['name']),
        interaction_evidence=dict(status='measured_reference_geometry' if window else 'unavailable',event_window_minima=minima,details=detail),
        response=response,outcome=dict(vehicle_contact_samples=sum(r['crash']=='True' for r in raw),
            off_lane_actor_samples=sum(r['on_lane']=='False' for r in raw),completed_under_definition=observed and not censored),
        pcds_candidate='unassessed',abnormality_assessment='unassessed',initial_conditions=settings,camera_evidence=camera,
        geometry_evidence=dict(features=str(feature_output.resolve()),source=str((source/'geometry').resolve())),
        evidence_quality=dict(reference='recorded_simulator_box_envelopes',physical_ambulance_validation=False,
            camera_semantic_validation=False,boundary_scope=FAMILIES[family]['boundaries']),
        provenance=dict(actor_ids_by_role=actors,source_sha256={name:hashlib.sha256((source/name).read_bytes()).hexdigest()
            for name in ['trajectory.csv','metadata.json','decisions.json','geometry/geometry_frames.jsonl']}),
        approval='proposed_pending_supervisor_review')
    validate(event,list(actors))
    return event
