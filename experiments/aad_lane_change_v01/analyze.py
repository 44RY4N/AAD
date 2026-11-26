"""Validate new paired captures and emit schema-0.1 observations without risk labels."""
import argparse
from collections import Counter
import copy
import csv
import json
import math
from pathlib import Path
import re

from aad_analysis.io import read_json,sha256,write_json,write_jsonl
from aad_analysis.run import extract as extract_v1
from aad_analysis_v02.run import extract as extract_v2
from experiments.validate_geometry import validate as validate_geometry
from experiments.validate_recording import validate as validate_rgb

ROOT=Path(__file__).resolve().parents[2]
SCHEMA=ROOT/'research/aad_v0.1/AAD_ANNOTATION_SCHEMA_v0.1.json'


def schema_check(value,schema,root=None):
    """Validate constructs used by the frozen local schema; not a general schema engine."""
    root=root or schema
    if '$ref' in schema:
        target=root
        for key in schema['$ref'][2:].split('/'):target=target[key]
        schema_check(value,target,root);return
    kinds=dict(object=lambda v:isinstance(v,dict),array=lambda v:isinstance(v,list),
        string=lambda v:isinstance(v,str),number=lambda v:type(v) in (int,float) and math.isfinite(v),
        integer=lambda v:type(v) is int,boolean=lambda v:type(v) is bool,null=lambda v:v is None)
    if 'type' in schema:
        types=schema['type'] if isinstance(schema['type'],list) else [schema['type']]
        assert any(kinds[t](value) for t in types),('type',value,types)
    if 'enum' in schema:assert value in schema['enum'],('enum',value)
    if 'const' in schema:assert value==schema['const']
    if isinstance(value,dict):
        assert set(schema.get('required',[]))<=value.keys(),('required',schema.get('required'))
        props=schema.get('properties',{})
        if schema.get('additionalProperties') is False:assert value.keys()<=props.keys(),('extra',value.keys()-props.keys())
        for k,v in value.items():
            if k in props:schema_check(v,props[k],root)
    if isinstance(value,list):
        assert len(value)>=schema.get('minItems',0)
        if schema.get('uniqueItems'):assert len({json.dumps(v,sort_keys=True) for v in value})==len(value)
        for v in value:
            if 'items' in schema:schema_check(v,schema['items'],root)
    if isinstance(value,str):
        assert len(value)>=schema.get('minLength',0)
        if 'pattern' in schema:assert re.search(schema['pattern'],value)
    if type(value) in (int,float):
        if 'minimum' in schema:assert value>=schema['minimum']
        if 'exclusiveMinimum' in schema:assert value>schema['exclusiveMinimum']
    for rule in schema.get('allOf',[]):schema_check(value,rule,root)
    if 'if' in schema:
        try:schema_check(value,schema['if'],root);matched=True
        except AssertionError:matched=False
        branch='then' if matched else 'else'
        if branch in schema:schema_check(value,schema[branch],root)


def read_lines(path):return [json.loads(s) for s in path.read_text().splitlines()]


def analyze(source,reference,output):
    source,reference,output=map(Path,[source,reference,output])
    if output.exists():raise FileExistsError(output)
    # These validators write only to these newly generated, authorized captures.
    validate_rgb(source,reference);validate_geometry(source,reference=reference)
    output.mkdir(parents=True,exist_ok=False)
    source_files={str(p.relative_to(ROOT)):sha256(p) for p in (ROOT/'experiments/aad_lane_change_v01').glob('*.py')}
    write_json(output/'scenario_provenance.json',dict(source_files=source_files,
        config=read_json(source/'metadata.json')['settings'],policy='Scheduled steering target; IDM longitudinal acceleration; at-rest spawning',
        note='Supplement to unchanged geometry-0.2 exporter source list, which names the original runner'))
    extract_v1(source,output/'v01');extract_v2(source,output/'v02')
    meta=read_json(source/'metadata.json');dt=meta['dt_s'];event=meta['result'];start,end=event['departure_step'],event['confirmed_step']
    with (source/'trajectory.csv').open(newline='') as f:rows=list(csv.DictReader(f))
    roles={r['role']:r['vehicle_id'] for r in rows};ids=[roles['subject'],roles['background_far']]
    pairs=[r for r in read_lines(output/'v02/pair_features.jsonl') if set(r['actor_ids'])==set(ids)]
    window=[r for r in pairs if start<=r['step']<=end]
    def get(r,name):return next(m for m in r['metrics'] if m['name']==name)
    best=min(window,key=lambda r:get(r,'outer_envelope_planar_separation')['value'])
    ttc=[get(r,'constant_velocity_fixed_orientation_envelope_ttc_2d') for r in window]
    valid_ttc=[m['value'] for m in ttc if m['validity']=='valid']
    actorfeatures={(r['step'],r['actor_id']):r for r in read_lines(output/'v01/actor_features.jsonl')}
    by={(int(r['step']),r['vehicle_id']):r for r in rows}
    motion=[]
    for r in pairs:
        step=r['step'];a,b=[by[step,i] for i in ids];derived={}
        for name in ['vx_world','vy_world','ax_world','ay_world']:
            ma,mb=[actorfeatures[step,i]['metrics'][name] for i in ids]
            ok=ma['validity']==mb['validity']=='valid'
            derived['relative_'+name]=dict(value=mb['value']-ma['value'] if ok else None,unit='m/s^2' if name.startswith('a') else 'm/s',
                validity='valid' if ok else 'unavailable',reason='other_minus_subject_offline_central_difference' if ok else 'missing_or_invalid_derivative_support',
                support_steps=ma['support_steps'],method='unchanged_v01_derivatives_other_minus_subject')
        for name,field in [('relative_longitudinal_position','source_longitudinal_m'),('relative_lateral_position','source_lateral_m')]:
            derived[name]=dict(value=float(b[field])-float(a[field]),unit='m',validity='valid',reason='fixed_straight_source_lane_coordinates',method='other_minus_subject')
        motion.append(dict(step=step,time_s=r['time_s'],actor_ids=ids,coordinate_frame='fixed_source_lane_and_world_axes_as_named',metrics=derived,risk_assessment='unassessed'))
    write_jsonl(output/'relative_motion.jsonl',motion)
    frames=read_lines(source/'camera/frames.jsonl')
    view=sum(next(p for p in r['projections'] if p['role']=='background_far')['in_frustum'] for r in frames if start<=r['step']<=end)
    summary=dict(scenario_id=meta['settings']['scenario_id'],intent=meta['settings']['condition_intent'],
        observed_event=event,event_start_step=start,event_end_step=end,
        target_pair_minimum_event_envelope_separation_m=get(best,'outer_envelope_planar_separation')['value'],minimum_step=best['step'],
        target_pair_minimum_full_interval_envelope_separation_m=min(get(r,'outer_envelope_planar_separation')['value'] for r in pairs),
        event_ttc_validity_counts=dict(Counter(m['validity'] for m in ttc)),minimum_valid_event_model_ttc_s=min(valid_ttc) if valid_ttc else None,
        target_vehicle_in_frustum_event_samples=view,event_samples=len(window),
        vehicle_contact_samples=sum(r['crash']=='True' for r in rows),all_actors_localized=all(r['on_lane']=='True' and r['same_road']=='True' for r in rows),
        target_vehicle_speed_kmh_at_departure=float(by[start,ids[1]]['speed_kmh']),subject_speed_kmh_at_departure=float(by[start,ids[0]]['speed_kmh']),
        risk_assessment='unassessed')
    write_json(output/'summary.json',summary)
    schema=read_json(SCHEMA);ann=copy.deepcopy(schema['examples'][0]);ann.update(record_kind='observation',scenario_id=summary['scenario_id'],
        run_id=read_json(source/'geometry/geometry_manifest.json')['run_id'],event_id=summary['scenario_id']+'_event_001',
        actors=[dict(actor_id=i,role=role,actor_type='DefaultVehicle') for role,i in roles.items()],primary_actor_id=ids[0],
        event_subtype='scheduled_single_lane_change',event_phase='confirmed_target_lane',
        time_interval=dict(start_s=start*dt,end_s=end*dt),step_range=dict(start=start,end=end),
        frame_range=dict(source_manifest=str((source/'camera/frames.jsonl').resolve()),first_step=start,last_step=end,available=True),
        sampling=dict(dt_s=dt,time_origin_s=0,step_origin=0,boundary_convention='Inclusive sampled departure to confirmation, retrospective existing verifier'),
        context=[dict(kind='target_lane_vehicle',value=True,evidence_status='observed',source_ids=['trajectory'],notes='Same target lane contains the configured comparison actor; proximity is measured separately')],
        conflict_evidence=dict(status='indeterminate',metric_ids=['target_envelope_min'],description='Measured interaction geometry; no calibrated conflict assessment protocol',assessment_protocol_id=None),
        evasive_response=dict(status='not_assessed',actions=[],causal_link_status='not_assessed',notes='IDM speed adaptation is recorded; evasive intent is not inferred'),
        metrics=[],ground_truth_sources=[],notes='Actual recorded lane-change observation; scenario intent is separate from criticality. DefaultVehicle proxy, elevated chase camera, not ambulance calibration.')
    ann['risk_assessment'].update(justification='No reviewed risk assessment protocol; configuration intent is not a label')
    ann['outcome'].update(collision_status='no_collision_recorded' if not summary['vehicle_contact_samples'] else 'vehicle_contact_recorded',
        collision_scope='All actor sampled vehicle-contact flags; not a universal contact oracle',manoeuvre_status='completed_under_definition',
        completion_definition_id='experiments.annotation.verify_v01',interaction_status='not_assessed',notes='Physical lane change only; full overtaking and safety are not established')
    ann['verification'].update(status='automatically_verified',methods=['experiments.annotation.verify','RGB/state validation','runtime geometry validation'],
        physical_event_verified=True,criticality_verified=False,notes='Recorded landmarks; no independent criticality review')
    for sid,path,kind in [('trajectory',source/'trajectory.csv','recorded_simulator_state'),('features',output/'v02/pair_features.jsonl','derived_measurement')]:
        ann['ground_truth_sources'].append(dict(source_id=sid,kind=kind,uri=str(path.resolve()),sha256=sha256(path),field_paths=[],notes='Actual saved artifact'))
    m=get(best,'outer_envelope_planar_separation')
    ann['metrics']=[dict(metric_id='target_envelope_min',name='sampled_event_minimum_outer_envelope_planar_separation',actor_ids=ids,value=m['value'],unit='m',
        validity=dict(status='valid',reason='Minimum at sampled event steps only'),method=dict(id=m['method'],version='0.2.0',coordinate_frame=m['coordinate_frame'],assumptions=m['assumptions'],source_ids=['features'],filter_description=None),
        time_support=ann['time_interval'],prediction_horizon_s=None,censored=False,uncertainty_note=m['uncertainty_note'])]
    schema_check(ann,schema)
    assert all(by[step,actor] for step in [start,end] for actor in ids)
    write_json(output/'event_annotation.json',ann)
    write_json(output/'event_validation.json',dict(passed=True,schema_constructs_checked=True,actor_and_step_references_checked=True,
        source_hashes_checked=True,physical_event_verified=True,criticality_verified=False))
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--reference',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(analyze(a.source,a.reference,a.output),indent=2))
