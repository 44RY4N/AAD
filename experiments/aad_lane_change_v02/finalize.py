"""End-to-end validation/annotation of fresh representative captures and sweep evidence."""
import argparse
import contextlib
import io
import json
from pathlib import Path
from aad_analysis.io import read_json,sha256,write_json
from aad_analysis_v02.validate import validate as validate_features
from experiments.aad_lane_change_v01.analyze import analyze,schema_check,SCHEMA
from .evidence import evidence


def run(output,sweep,refinement,prefix='aad_lc2',resume=False,delayed_context=False):
    output=Path(output)
    if (output/'comparison.json').exists():raise FileExistsError('Completed output is immutable')
    output.mkdir(parents=True,exist_ok=resume);selected=[]
    for condition in ['clear','concurrent','delayed']:
        for repeat in [False,True]:
            suffix='_rgb_repeat_01' if repeat else '_rgb_01'
            other='_rgb_01' if repeat else '_rgb_repeat_01'
            capture_condition='delayed_context' if condition=='delayed' and delayed_context else condition
            raw=Path('test_output')/(prefix+'_'+capture_condition+suffix)
            reference=Path('test_output')/(prefix+'_'+capture_condition+other)
            destination=output/(condition+('_repeat' if repeat else ''))
            cached=(destination/'annotated_event.json').exists()
            if cached and not resume:raise FileExistsError(destination)
            if not cached:
                with contextlib.redirect_stdout(io.StringIO()):
                    analyze(raw,reference,destination)
                    validate_features(destination/'v02')
            result=evidence(raw,destination/'v02')
            if not cached:write_json(destination/'window_evidence.json',result)
            annotation=read_json(destination/'event_annotation.json')
            hits=result['concurrent_model_interaction_samples']
            annotation['context'].append(dict(kind='concurrent_target_lane_model_interaction',value=bool(hits),
                evidence_status='derived',source_ids=['features'],notes=result['model_interaction_definition']))
            annotation['notes']+=' Observed model interaction sample steps: '+str([h['step'] for h in hits])+'. Physical contact and criticality are separate.'
            if hits:
                best=min(hits,key=lambda r:r['model_ttc_s'])
                item=dict(annotation['metrics'][0]);item.update(metric_id='event_model_ttc_min',
                    name='sampled_event_minimum_valid_constant_velocity_fixed_orientation_envelope_ttc_2d',value=best['model_ttc_s'],unit='s',
                    validity=dict(status='valid',reason='Minimum of finite valid model TTC samples in verified event window; not physical impact prediction'),
                    prediction_horizon_s=5.)
                item['method']=dict(item['method'],id='continuous_separating_axis_time_intervals',assumptions=[
                    'Current envelopes translate at constant world velocity with fixed orientation',
                    'Five-second model horizon; minimum over sampled states only'])
                annotation['metrics'].append(item);annotation['conflict_evidence']['metric_ids'].append('event_model_ttc_min')
                annotation['conflict_evidence']['description']='Concurrent target occupancy and finite model-predicted envelope intersection observed; calibrated conflict/criticality assessment remains indeterminate'
            schema_check(annotation,read_json(SCHEMA))
            if not cached:write_json(destination/'annotated_event.json',annotation)
            selected.append(dict(condition=condition,repeat=repeat,raw=str(raw),analysis=str(destination),
                event=result['event'],event_minimum_m=result['event_minimum_envelope_separation_m'],
                event_minimum_step=result['event_minimum_step'],model_ttc_min_s=result['minimum_valid_model_ttc_s'],
                interaction_steps=[h['step'] for h in hits],command_to_departure_steps=result['command_to_departure_steps'],
                subject_departure_speed_kmh=result['subject_departure_speed_kmh']))
            print(condition,repeat,selected[-1]['interaction_steps'],flush=True)
    repeats=[]
    for condition in ['clear','concurrent','delayed']:
        a,b=output/condition,output/(condition+'_repeat')
        for version,field in [('v01','feature_fingerprint_without_generated_ids'),('v02','normalized_feature_sha256')]:
            assert read_json(a/version/'validation.json')[field]==read_json(b/version/'validation.json')[field]
        assert read_json(a/'summary.json')['observed_event']==read_json(b/'summary.json')['observed_event']
        repeats.append(dict(condition=condition,trajectory_event_policy_repeat=True,normalized_v01_v02_repeat=True))
    comparisons=[]
    for screening in [Path(sweep),Path(refinement)]:
        for row in read_json(screening/'comparison.json'):
            name=row['config']['scenario_id']
            if row['exit_code']==0:
                raw=Path(read_json(screening/name/'manifest.json')['source_directory'])
                detailed=evidence(raw,screening/name)
                row.update(model_interaction_steps=[h['step'] for h in detailed['concurrent_model_interaction_samples']])
            else:
                log=Path(row['log']).read_text(encoding='utf-8')
                row['vehicle_contact_termination_reported']="'crash_vehicle': True" in log
                row['terminal_record_available']=(Path(row['log']).parent/name/'termination.json').exists()
                row['feature_status']='unavailable: incomplete geometry rejected by standard full-run extractor; raw prefix/log retained'
            comparisons.append(row)
    write_json(output/'sweep_comparison.json',comparisons)
    write_json(output/'comparison.json',dict(representatives=selected,reproducibility=repeats,risk_assessment='unassessed'))
    code=list(Path(__file__).parent.glob('*.py'))+list(Path(__file__).parent.glob('configs/*.json'))
    write_json(output/'provenance.json',dict(code_sha256={str(p):sha256(p) for p in code},
        completed_sweep_trials=sum(r['exit_code']==0 for r in comparisons),failed_sweep_trials=sum(r['exit_code']!=0 for r in comparisons),
        annotation_schema_sha256=sha256(SCHEMA),risk_assessment='unassessed'))


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sweep',type=Path,default=Path('analysis_output/aad_lc2_sweep_01'))
    p.add_argument('--refinement',type=Path,default=Path('analysis_output/aad_lc2_refine_01'))
    p.add_argument('--prefix',default='aad_lc2')
    p.add_argument('--resume',action='store_true');p.add_argument('--delayed-context',action='store_true')
    a=p.parse_args();run(a.output,a.sweep,a.refinement,a.prefix,a.resume,a.delayed_context)
