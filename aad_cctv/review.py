"""Prepare a fresh timestamped 30-case human-review package; never answer cases."""
import argparse
import base64
from datetime import datetime
import hashlib
import json
from pathlib import Path
from .camera import write


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_responses(data,cases_hash,case_ids):
    if data.get('schema_version')!='vehicle-case-review-0.2' or data.get('cases_sha256')!=cases_hash:
        raise ValueError('Wrong review provenance')
    rows=data.get('responses',[])
    if len(rows)!=len(case_ids) or {r['case_id'] for r in rows}!=set(case_ids):raise ValueError('Missing/duplicate/unknown cases')
    for r in rows:
        if type(r['review_complete']) is not bool or not isinstance(r['reviewer'],str):raise ValueError('Invalid reviewer state')
        for field in ['vehicle_present','vehicle_class','already_counted','too_ambiguous']:
            choices=['car','truck','uncertain'] if field=='vehicle_class' else ['yes','no','uncertain']
            if r[field] is not None and r[field] not in choices:raise ValueError('Invalid response')
            if r['review_complete'] and r[field] is None:raise ValueError('Incomplete answers')
        if r['vehicle_present']=='no' and r['vehicle_class'] not in [None,'uncertain']:raise ValueError('Absent object cannot have confirmed class')
        if r['review_complete']:
            if not r['reviewer'].strip() or not r.get('reviewed_at_utc'):raise ValueError('Reviewer and timestamp required')
            timestamp=datetime.fromisoformat(r['reviewed_at_utc'].replace('Z','+00:00'))
            if timestamp.utcoffset() is None or timestamp.utcoffset().total_seconds()!=0:raise ValueError('UTC timestamp required')
    return dict(cases=len(rows),completed=sum(r['review_complete'] for r in rows),
        incomplete_case_ids=[r['case_id'] for r in rows if not r['review_complete']],independence='Reviewer identity is self-reported; must be verified externally')


def prepare(source,output):
    output.mkdir(parents=True,exist_ok=False)
    original=source/'review_cases.json';cases=json.loads(original.read_text())
    for c in cases['cases']:
        path=Path(cases['image_provenance'][c['frame_id']]['image']);run=path.parents[2]
        meta=json.loads((run/'metadata.json').read_text()) if (run/'metadata.json').exists() else {}
        c['context']=dict(scenario_id=meta.get('settings',{}).get('scenario_id'),event_id=None,
            actor_id=None,candidate_id=c['case_id'],identity_status='Not independently linked to physical actor/event; do not guess identity from box')
    cases['source_cases_sha256']=sha(original);write(output/'review_cases.json',cases)
    digest=sha(output/'review_cases.json')
    responses=dict(schema_version='vehicle-case-review-0.2',cases_sha256=digest,
        responses=[dict(case_id=c['case_id'],reviewer='',review_complete=False,reviewed_at_utc=None,
            vehicle_present=None,vehicle_class=None,already_counted=None,too_ambiguous=None,notes='') for c in cases['cases']])
    validate_responses(responses,digest,[c['case_id'] for c in cases['cases']]);write(output/'responses_unreviewed.json',responses)
    images={}
    for fid,item in cases['image_provenance'].items():
        path=Path(item['image'])
        if sha(path)!=item['image_sha256']:raise ValueError('Original image hash mismatch')
        images[fid]='data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode()
    template=Path(__file__).with_name('review_template.html').read_text(encoding='utf-8')
    template=template.replace('/*CASES*/',json.dumps(cases)).replace('/*RESPONSES*/',json.dumps(responses)).replace('/*IMAGES*/',json.dumps(images))
    (output/'review.html').write_text(template,encoding='utf-8')
    (output/'REVIEW_INSTRUCTIONS.md').write_text('Open review.html locally. Enter your own reviewer identifier. Review all 30 cases without redrawing boxes. Red is the candidate; optional blue boxes are other already-counted predictions, not ground truth. Confirm visible vehicle presence, car/truck/uncertain, duplicate identity and ambiguity. Use uncertain when needed. Mark complete only after answering every question; a UTC timestamp is saved. Export JSON and retain it. Do not change the historical annotation file. Event/actor identities remain unknown where independent association is absent. No review has been completed by this preparation script.\n',encoding='utf-8')
    write(output/'validation.json',dict(cases=30,completed=0,source_sha256=sha(original),cases_sha256=digest,
        image_hashes_checked=True,human_review_performed=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('action',choices=['prepare','validate']);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--cases',type=Path)
    a=p.parse_args()
    if a.action=='prepare':prepare(a.source,a.output)
    else:
        if a.output.exists():raise FileExistsError(a.output)
        cases=json.loads(a.cases.read_text());report=validate_responses(json.loads(a.source.read_text()),sha(a.cases),[c['case_id'] for c in cases['cases']]);write(a.output,report)
