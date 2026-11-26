"""Build the end-semester CSVs and figures from retained experiment evidence."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

from .events import braking_window, lines
from .replay_evaluation import ANALYSIS, ARCHIVE, RAW, vision_jobs

SEPARATION = 'outer_envelope_planar_separation'
GAP = 'shared_path_signed_longitudinal_envelope_gap'
TTC = 'constant_velocity_fixed_orientation_envelope_ttc_2d'


def load(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def rows(path):
    with path.open(newline='', encoding='utf-8-sig') as file:
        return list(csv.DictReader(file))


def save(path, data):
    fields = list(dict.fromkeys(key for row in data for key in row))
    with path.open('x', encoding='utf-8', newline='') as file:
        writer = csv.DictWriter(file, fields)
        writer.writeheader()
        writer.writerows(data)


def experiments(work):
    statuses = {}
    archive = {Path(j['output']).name:j for j in load(ARCHIVE/'rgb_batch_results.json')}
    ordinary = RAW/'shared_braking_ordinary_01'
    for source in sorted(set(RAW.glob('cctv_*')) | {ordinary}):
        execution = source/'cctv_execution.json'
        meta = source/'metadata.json'
        analysis = ANALYSIS/source.name
        event_path = analysis/'event.json'
        validation = analysis/'validation.json'
        record = load(execution) if execution.exists() else {}
        metadata = load(meta) if meta.exists() else {}
        settings = metadata.get('settings',record.get('settings',{}))
        family = record.get('family')
        index = source/'cctv_cameras.json'
        is_rgb = index.exists()
        trajectory = source/'trajectory.csv'
        raw = rows(trajectory) if trajectory.exists() else []
        event = load(event_path) if event_path.exists() else {}
        if source == ordinary:
            subject = [dict(step=int(r['step']), speed_kmh=float(r['speed_kmh'])) for r in raw if r['role']=='subject']
            start,end,censored = braking_window(subject,settings['brake_step'])
            pairs_path = work/'braking_ordinary_geometry/pair_features.jsonl'
            pairs = [r for r in lines(pairs_path) if start<=r['step']<=end and set(r['actor_roles'])=={'subject','leader'}]
            minima = {}
            for name in (SEPARATION,GAP,TTC):
                values = [m['value'] for r in pairs for m in r['metrics'] if m['name']==name and m['validity']=='valid']
                minima[name] = dict(value=min(values) if values else None, valid_samples=len(values), window_samples=len(pairs))
            event = dict(family='C',start_s=start*metadata['dt_s'],end_s=end*metadata['dt_s'],censored=censored,
                         physical_behaviour='detected',target_role='leader',
                         interaction_evidence=dict(event_window_minima=minima, details={}))
        start = event.get('start_s')
        end = event.get('end_s')
        evidence = event.get('interaction_evidence',{})
        minima = evidence.get('event_window_minima',{})
        detail = evidence.get('details',{})
        checks = load(validation) if validation.exists() else {}
        cam_entries = load(index) if is_rgb else []
        cameras = [source/c['directory'] for c in cam_entries]
        run = archive.get(source.name,{})
        if source.name=='cctv_braking_ordinary_20261125_04':
            run=dict(config='aad_system/configs/braking_ordinary.json',cameras='aad_cctv/configs/roadside_wide.json')
        if source.name=='cctv_concurrent_near_20261125_01':
            run=dict(config='experiments/aad_lane_change_v02/configs/concurrent_context.json',
                     cameras='aad_cctv/configs/roadside_near_pilot.json')
        if source.name=='cctv_concurrent_high_20261125_01':
            run=dict(config='experiments/aad_lane_change_v02/configs/concurrent_context.json',
                     cameras='aad_cctv/configs/roadside_high_pilot.json')
        failure = record.get('failure')
        crash_count = sum(r['crash']=='True' for r in raw) if raw else None
        final_speed = {r['role']:float(r['speed_kmh']) for r in raw}
        outcome = ('failed_before_complete_recording' if failure else 'contact_observed' if crash_count
                   else 'no_contact_in_recorded_samples' if raw else 'recording_incomplete_or_unavailable')
        geometry_report = source/'geometry/validation.json'
        model_witness = minima.get(TTC,{}).get('value')
        co_occupancy = detail.get('simultaneous_occupancy_steps')
        trigger = ('fixed_model_overlap_witness' if model_witness is not None and model_witness>0
                   else 'sampled_zone_co_occupancy' if co_occupancy else None)
        statuses[source.name] = dict(experiment=source.name,
            family=event.get('family') or {'baseline_lane_change':'A','lane_change':'A','overtaking':'B','abort':'B','braking':'C','crossing':'D'}.get(family),
            condition=settings.get('scenario_id'), viewpoint='fixed CCTV' if is_rgb else 'no completed CCTV frames',
            seed=settings.get('seed'), scenario_config=run.get('config'), camera_config=run.get('cameras'),
            camera_frames=sum(load(c/'camera.json')['frames'] for c in cameras), target_role=event.get('target_role'),
            event_start_s=start,event_end_s=end,event_duration_s=round(end-start,9) if start is not None and end is not None else None,
            censored=event.get('censored'), physical_behaviour=event.get('physical_behaviour'),
            roi_min_envelope_separation_m=minima.get(SEPARATION,{}).get('value'),
            separation_valid_samples=minima.get(SEPARATION,{}).get('valid_samples'),
            roi_min_signed_model_gap_m=minima.get(GAP,{}).get('value'),
            roi_min_valid_model_ttc_s=minima.get(TTC,{}).get('value'),
            ttc_valid_samples=minima.get(TTC,{}).get('valid_samples'),
            pair_window_samples=minima.get(TTC,{}).get('window_samples'),
            sampled_zone_clearance_s=detail.get('sampled_zone_clearance_s'),
            simultaneous_zone_occupancy_samples=len(detail['simultaneous_occupancy_steps']) if 'simultaneous_occupancy_steps' in detail else None,
            commanded_leader_speed_kmh=settings.get('leader_after_kmh'),final_subject_speed_kmh=final_speed.get('subject'),
            final_leader_speed_kmh=final_speed.get('leader'), outcome=outcome, contact_actor_samples=crash_count,
            provisional_reference_review='flag_for_review' if trigger else 'unassessed',
            review_trigger=trigger, risk='unassessed', failure=failure, validation=('camera_and_event_checked' if checks else
                'geometry_checked' if geometry_report.exists() and load(geometry_report).get('passed') else 'no_passing_report_located'),
            run_path=source.as_posix(),trajectory_path=trajectory.as_posix() if raw else None,
            event_evidence=event_path.as_posix() if event_path.exists() else (work/'braking_ordinary_geometry/pair_features.jsonl').as_posix() if source==ordinary else None,
            validation_evidence=validation.as_posix() if checks else geometry_report.as_posix() if geometry_report.exists() else None,
            frame_manifests=' | '.join((c/'observations.jsonl').as_posix() for c in cameras))
    for name in ('sweep','refine'):
        folder=ANALYSIS/f'aad_lc2_{name}_01'
        path=folder/'comparison.json'
        for record in load(path):
            config=record['config']
            label=config['scenario_id']
            window=record.get('event_window_steps')
            statuses[label]=dict(experiment=label,family='A',condition=config.get('condition_intent'),viewpoint='historical lane-change study',
                seed=config['seed'],event_start_s=window[0]*.1 if window else None,event_end_s=window[1]*.1 if window else None,
                event_duration_s=round((window[1]-window[0])*.1,9) if window else None,
                roi_min_envelope_separation_m=record.get('event_minimum_envelope_separation_m'),
                roi_min_valid_model_ttc_s=record.get('minimum_valid_model_ttc_s'),
                validation=record['outcome'],outcome=record['outcome'],contact_actor_samples=record.get('vehicle_contact_samples'),
                failure=f"exit_code={record['exit_code']}" if record.get('exit_code') else None,risk='unassessed',
                event_evidence=path.as_posix(),run_path=(folder/label).as_posix() if (folder/label).is_dir() else None)
    return statuses


def metrics(work, replay, followup=None, confidence_check=None):
    output=[]
    def add(scope,stage,metric,value,unit,n,method,status,path):
        output.append(dict(scope=scope,stage=stage,metric=metric,value=value,unit=unit,sample_size=n,
                           method=method,validity=status,evidence=path.as_posix()))
    for job in vision_jobs():
        if job['returncode']:
            continue
        name=job['name']
        vision=Path(job['output'])/'report.json'
        stats=load(vision)
        diagnostic=replay/name/'reference/report.json'
        ref=load(diagnostic)
        feature=replay/name/'temporal/evidence.json'
        temp=load(feature)
        for key,method in [('frames_with_detections','Frames with any retained YOLOX output at 0.35'),
                           ('created_tracks','Number of sticky IoU tracker labels created'),
                           ('tracks_with_at_least_10_observations','Tracks with at least ten detections, gaps permitted')]:
            add(name,'vision',key,stats[key],'count',stats['frames'],method,'descriptive',vision)
        for key,count,unit in [('position_discrepancy_rmse_m','position_pairs','m'),('speed_discrepancy_mae_m_s','speed_pairs','m/s')]:
            add(name,'mapping',key,ref[key],unit,ref[count],
                'Projected IoU association, differing camera box contact and simulator actor reference points',
                'conditional_proxy' if ref[count] else 'unavailable_no_valid_pairs',diagnostic)
        add(name,'tracking','missing_captures_within_track_spans',
            sum(t['missing_captures_between_first_and_last'] for t in temp['tracks']) if temp['tracks'] else None,
            'count',len(temp['tracks']),'Count of gaps between first and last detection of unverified track labels',
            'descriptive' if temp['tracks'] else 'unavailable_no_tracks',feature)
        add(name,'event','camera_lane_transition_hypotheses',len(temp['events']) if temp['lane_divider_world_y_m'] is not None else None,
            'count',temp['frames'],'Consecutive observations on opposite sides of static surveyed divider',
            'unvalidated_provisional' if temp['lane_divider_world_y_m'] is not None else 'not_applicable',feature)
        add(name,'event','camera_review_candidates',sum(e['review_status']=='candidate_for_review' for e in temp['events'])
            if temp['lane_divider_world_y_m'] is not None else None,'count',len(temp['events']),'Transition and three consecutive closing estimates',
            'unvalidated_provisional' if temp['lane_divider_world_y_m'] is not None else 'not_applicable',feature)
    camera_checks = list(ANALYSIS.glob('cctv_*_20261119_final01/validation.json'))
    camera_checks.append(ANALYSIS/'cctv_braking_ordinary_20261125_04/validation.json')
    camera_checks.extend(ANALYSIS.glob('cctv_concurrent_*_20261125_01/validation.json'))
    for path in sorted(camera_checks):
        for view in load(path)['cameras']:
            scope=path.parent.name+'/'+view['camera_id']
            for key,unit in [('max_projection_error_px','px'),('known_ground_point_roundtrip_error_m','m')]:
                add(scope,'calibration',key,view[key],unit,view['frames'],'Simulated projection cross-check over saved captures','algebra_only',path)
    scale=work/'scale_braking/report.json'
    pilot=load(scale)['samples']
    for variant in ('baseline','cropped'):
        add('braking','scale_pilot',variant+'_frames_with_predictions',sum(r[variant]>0 for r in pilot),
            'count',len(pilot),'Same images, unchanged weights/confidence; method in pilot report','descriptive',scale)
    scale_ref=work/'scale_braking_reference/report.json'
    add('braking','scale_pilot','cropped_projected_actor_associations',sum(load(scale_ref)['matched_projected_samples'].values()),
        'count',len(pilot),'Greedy IoU >= 0.5 against collision envelopes','conditional_proxy',scale_ref)
    review=ANALYSIS/'cctv_human_review_20261119_01/validation.json'
    for stage,keys in [('detection',('precision','recall','AP')),('tracking',('IDF1','HOTA')),
                       ('event',('precision','recall')),('assessment',('precision','recall'))]:
        for name in keys:
            add('fixed CCTV',stage,name,None,'ratio',None,'Requires independent labels appropriate to stage',
                'unavailable_independent_labels',review)
    if followup is not None:
        for row in rows(followup/'hypotheses.csv'):
            scope=f"{row['view']}/{row['preprocessing']}/{row['family']}"
            for key in ('hypotheses','review_flags'):
                add(scope,'extended_camera_event',key,int(row[key]),'count',int(row['frames']),
                    'Provisional camera-only output by applicable family','unvalidated_provisional',Path(row['evidence']))
        for case in ('clear','concurrent_roadside'):
            path=followup/case/'ablation.json'
            ablation=load(path)
            for variant in ('full','tiles'):
                scope=f'{case}/{variant}'
                add(scope,'crop_ablation','frames_with_output',ablation['frames_with_detections'][variant],
                    'count',ablation['same_image_count'],'Same first 100 images and unchanged model weights','descriptive',path)
                pairs=ablation['event_associations']
                for role,total in pairs['projected_support'].items():
                    add(scope+'/'+role,'crop_ablation','event_envelope_associations',pairs[variant].get(role,0),
                        'count',total,'Same event frames; projected envelope IoU >= 0.5','conditional_proxy',path)
        for pilot_path in sorted(followup.glob('concurrent_*/pilot.json')):
            scope=pilot_path.parent.name
            pilot=load(pilot_path)
            add(scope,'camera_placement','frames_with_output',pilot['frames_with_output'],
                'count',pilot['frames'],'Fixed viewpoint pilot; same YOLOX weights and preprocessing','descriptive',Path(pilot['vision']))
            ref_path=Path(pilot['reference'])
            for role,total in pilot['support'].items():
                add(scope+'/'+role,'camera_placement','event_envelope_associations',pilot['associations'].get(role,0),
                    'count',total,'Projected envelope IoU >= 0.5','conditional_proxy',ref_path)
            ref=load(ref_path)
            for key,count,unit in [('position_discrepancy_rmse_m','position_pairs','m'),('speed_discrepancy_mae_m_s','speed_pairs','m/s')]:
                add(scope,'camera_placement',key,ref[key],unit,ref[count],
                    'Differing camera box-contact and simulator reference points',
                    'conditional_proxy' if ref[count] else 'unavailable_no_valid_pairs',ref_path)
    if confidence_check is not None:
        for path in sorted(confidence_check.glob('*/comparison.json')):
            comparison=load(path)
            for i,threshold in enumerate(comparison['confidence']):
                scope=f'{path.parent.name}/confidence_{threshold:g}'
                add(scope,'confidence_check','frames_with_output',comparison['frames_with_predictions'][i],'count',
                    comparison['frames'],'Identical camera images, weights, preprocessing and tracker','descriptive',path)
                for role,total in comparison['event_projected_support'].items():
                    for key in ('associations','longest_consecutive_same_label'):
                        add(scope+'/'+role,'confidence_check',key,comparison[key][role][i],'count',
                            total,'Event-window projected envelope association, no identity certification','conditional_proxy',path)
            scope=f"{path.parent.name}/confidence_{comparison['confidence'][-1]:g}"
            events=comparison['camera_hypotheses']
            add(scope,'confidence_check','hypotheses',len(events),'count',comparison['frames'],
                'Camera-only output with provisional criteria','unvalidated_provisional',path)
            add(scope,'confidence_check','review_flags',sum(e['review_status']=='candidate_for_review' for e in events),
                'count',comparison['frames'],'Camera-only review output with provisional criteria','unvalidated_provisional',path)
    return output


def figure_data(output, replay, by_id, data, followup=None):
    plt.rcParams.update({'font.size':11, 'axes.spines.top':False, 'axes.spines.right':False,
                         'axes.labelcolor':'#173149','text.color':'#173149','axes.titlecolor':'#173149'})
    folder=output/'figures'
    folder.mkdir()
    clear='cctv_clear_20261119_final01'
    dense='cctv_concurrent_20261119_final01'
    fig, axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    for key,label,color in [(clear,'Clear','#147c80'),(dense,'Concurrent','#bc5317')]:
        run=by_id[key]
        pairs=lines(ANALYSIS/key/'v02/pair_features.jsonl')
        selected=[(m['time_support']['start_s'],m['value']) for p in pairs
                  if set(p['actor_roles'])=={'subject','background_far'}
                  for m in p['metrics'] if m['name']==SEPARATION and m['validity']=='valid'
                  and run['event_start_s']<=m['time_support']['start_s']<=run['event_end_s']]
        r=[row for row in rows(RAW/key/'trajectory.csv') if row['role']=='subject' and run['event_start_s']<=float(row['time_s'])<=run['event_end_s']]
        axes[0].plot([float(x['time_s']) for x in r],[float(x['y']) for x in r],label=label,color=color,linewidth=2)
        axes[1].plot(*zip(*selected),label=label,color=color,linewidth=2)
    axes[0].set(title='Similar lateral motion',ylabel='Subject road coordinate y (m)',xlabel='Simulation time (s)')
    axes[1].set(title='Different surrounding separation',ylabel='Minimum envelope separation (m)',xlabel='Simulation time (s)')
    axes[1].legend(frameon=False)
    fig.savefig(folder/'lane_change_context.png',dpi=190);plt.close(fig)

    fig, axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    for key,label,color in [('cctv_braking_ordinary_20261125_04','Ordinary | fixed CCTV reference','#147c80'),
                            ('cctv_braking_20261119_final01','Stronger | fixed CCTV reference','#bc5317')]:
        run=by_id[key]
        for ax,role in zip(axes,('leader','subject')):
            r=[row for row in rows(RAW/key/'trajectory.csv') if row['role']==role]
            ax.plot([float(x['time_s']) for x in r],[float(x['speed_kmh']) for x in r],label=label,color=color)
            ax.set(title=f'{role.capitalize()} observed speed',xlabel='Simulation time (s)',ylabel='Speed (km/h)')
    axes[0].legend(frameon=False,fontsize=8)
    fig.savefig(folder/'braking_response.png',dpi=190);plt.close(fig)

    fig, axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    for ax,key,label in zip(axes,('clear','abort'),('Clear lane change','Aborted overtake')):
        job=next(j for j in load(ARCHIVE/'vision_batch_results.json') if j['name']==key)
        camera=Path(job['source'])
        vision=Path(job['output'])
        count=load(vision/'frame_counts.json')
        step=next(r['step'] for r in count if r['step']%10==0 and r['detections']>0)
        path=(camera/'frames'/f'{step:06d}.png') if key=='clear' else (vision/f'overlay_{step:06d}.jpg')
        with Image.open(path) as image:
            ax.imshow(image)
        ax.set_title(f'{label} | {"RGB" if key=="clear" else "YOLOX + IoU track"} | step {step}')
        ax.axis('off')
    fig.savefig(folder/'fixed_cctv_examples.png',dpi=190);plt.close(fig)

    fig, ax=plt.subplots(figsize=(8,4),layout='constrained')
    counts=[r for r in data if r['metric']=='frames_with_detections']
    ax.barh([r['scope'].replace('_',' ') for r in counts],[100*r['value']/r['sample_size'] for r in counts],color='#147c80')
    ax.set(xlabel='Frames with at least one YOLOX prediction (%)',title='Baseline output coverage across fixed views')
    ax.invert_yaxis()
    fig.savefig(folder/'vision_coverage.png',dpi=190);plt.close(fig)

    job='abort'
    report=replay/job/'reference/reference_associations.json'
    matches=[r for r in load(report) if r['role']=='subject' and r['estimated_contact_xy_m'] is not None]
    fig, axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    for i,ax in enumerate(axes):
        time=[r['time_s'] for r in matches]
        ax.scatter(time,[r['reference_xy_m'][i] for r in matches],label='Simulator actor reference',marker='x',s=25,color='#173149')
        ax.scatter(time,[r['estimated_contact_xy_m'][i] for r in matches],label='Camera box-contact estimate',s=12,color='#bc5317')
        ax.set(xlabel='Simulation time (s)',ylabel=f'World {"xy"[i]} (m)')
    axes[0].legend(loc='best',frameon=False,fontsize=8)
    fig.suptitle('Aborted overtake | synchronized projected-envelope associations')
    fig.savefig(folder/'trajectory_comparison.png',dpi=190);plt.close(fig)

    if followup is not None and all((followup/f'concurrent_{p}/pilot.json').exists() for p in ('near','high')):
        fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
        for ax,placement,title in zip(axes,('near','high'),('Closer, lower view','Elevated view')):
            pilot=load(followup/f'concurrent_{placement}/pilot.json')
            path=Path(pilot['source'])/'camera/frames/000072.png'
            with Image.open(path) as frame:
                ax.imshow(frame)
            ax.set_title(title+' | step 72')
            ax.axis('off')
        fig.savefig(folder/'viewpoint_occlusion.png',dpi=190)
        plt.close(fig)


def run(output, work, replay, followup=None, confidence_check=None):
    output.mkdir(parents=True, exist_ok=False)
    by_id=experiments(work)
    data=metrics(work, replay, followup, confidence_check)
    save(output/'EXPERIMENTS.csv',list(by_id.values()))
    save(output/'METRICS.csv',data)
    figure_data(output,replay,by_id,data,followup)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--replay',type=Path,required=True)
    parser.add_argument('--followup',type=Path)
    parser.add_argument('--confidence-check',type=Path)
    args=parser.parse_args()
    run(args.output,args.work,args.replay,args.followup,args.confidence_check)
