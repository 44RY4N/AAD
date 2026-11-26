"""Proposed bounded-event contract, versioned separately from historical v0.1."""
import json
import math
from pathlib import Path

FAMILIES = {
 'A': dict(name='lane_change', inclusion='Verified sampled lateral departure and target settling',
           exclusion='Lane-ID change alone; command without motion', context='Source/target lane geometry and nearby actors',
           boundaries='First |source lateral|>.35 m through 11th consecutive target-tolerance sample; otherwise censored',
           measures=['envelope separation','shared-path gap','lane occupancy','fixed-model TTC']),
 'B': dict(name='overtaking', inclusion='Target-specific departure and pass/return or explicit controller abort',
           exclusion='No-pass control is not an intentional abort', context='Leader identity, relative order, passing/source lanes',
           boundaries='Physical departure through confirmed return ahead or confirmed abort return behind; censored if unfinished',
           measures=['envelope separation','relative order','lane occupancy','fixed-model TTC']),
 'C': dict(name='braking_deceleration', inclusion='Recorded speed decrease sustained over three samples',
           exclusion='Desired-speed command without observed deceleration; intent inferred from speed alone',
           context='Leader command, subject/leader actual speeds and time',
           boundaries='First sustained decrease exceeding .5 km/h cumulative from pre-command speed through stable 11-sample plateau or censoring',
           measures=['actual speed','finite-difference acceleration','following gap','envelope separation']),
 'D': dict(name='intersection_crossing', inclusion='Vehicle footprint enters defined path-intersection zone',
           exclusion='Legal-priority violation without road-rule evidence; absence of target treated as conflict',
           context='Crossing routes, explicit zone polygon and per-actor occupancy intervals',
           boundaries='First relevant zone entry through last relevant actor exit; censored if occupied at end',
           measures=['zone occupancy','sampled temporal clearance','envelope separation','fixed-model TTC'])}


def schema():
    string={'type':'string','minLength':1}; nullable={'type':['integer','null'],'minimum':0}
    properties={
        'schema_version':{'const':'aad-event-0.2-proposed'},'event_id':string,'scenario_id':string,
        'family':{'enum':list(FAMILIES)},'subject_role':{'const':'subject'},
        'target_role':{'type':['string','null']},'surrounding_roles':{'type':'array','items':string,'uniqueItems':True},
        'actor_scope':{'const':'simulator_reference_roles_only'},
        'start_step':nullable,'end_step':nullable,'dt_s':{'type':'number','exclusiveMinimum':0},
        'start_s':{'type':['number','null']},'end_s':{'type':['number','null']},
        'censored':{'type':'boolean'},'physical_behaviour':{'enum':['detected','not_observed','insufficient_evidence']},
        'attention_context':{'type':'object'},'interaction_evidence':{'type':'object'},
        'response':{'type':'object'},'outcome':{'type':'object'},
        'pcds_candidate':{'enum':['unassessed','insufficient_evidence']},
        'abnormality_assessment':{'const':'unassessed'},
        'initial_conditions':{'type':'object'},'camera_evidence':{'type':'array'},
        'geometry_evidence':{'type':'object'},'evidence_quality':{'type':'object'},
        'provenance':{'type':'object'},'approval':{'const':'proposed_pending_supervisor_review'}}
    return {'$schema':'https://json-schema.org/draft/2020-12/schema',
            '$id':'urn:aad:event:0.2:proposed','type':'object','additionalProperties':False,
            'required':list(properties),'properties':properties}


def validate(event, roles):
    # The repository's restricted-schema validator handles precisely these schema
    # constructs; semantic checks below are not delegated to JSON Schema.
    from experiments.aad_lane_change_v01.analyze import schema_check
    schema_check(event, schema())
    if event['subject_role'] not in roles or event['target_role'] is not None and event['target_role'] not in roles:
        raise ValueError('Unresolved subject/target')
    selected=[event['subject_role']]+([event['target_role']] if event['target_role'] else [])+event['surrounding_roles']
    if len(set(selected)) != len(selected) or not set(selected)<=set(roles):
        raise ValueError('Duplicate/unresolved actor selection')
    start,end=event['start_step'],event['end_step']
    if (start is None)!=(end is None) or start is not None and end<start:
        raise ValueError('Invalid bounded event interval')
    for step,time in [(start,event['start_s']),(end,event['end_s'])]:
        if step is None:
            if time is not None:raise ValueError('Unknown boundary has known timestamp')
        elif time is None or not math.isclose(step*event['dt_s'],time,abs_tol=1e-9):
            raise ValueError('Step/time mismatch')
    if event['physical_behaviour']=='detected' and start is None:
        raise ValueError('Detected event needs observed boundaries')
    json.dumps(event,allow_nan=False)
    return True


if __name__=='__main__':
    folder=Path(__file__).parent
    for name,data in [('event_schema.json',schema()),('operational_families.json',FAMILIES)]:
        with (folder/name).open('x',encoding='utf-8') as f:json.dump(data,f,indent=2)
