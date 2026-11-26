"""Replay the preserved pre-fix first-stage runner, including ineffective speed probes."""
import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import sys
from aad_analysis.io import read_json,write_json


def run(source,output):
    source,output=Path(source),Path(output)
    snapshot=read_json(source/'controller_snapshot.json')
    snapshot={k:{p.replace('\\','/'):value for p,value in v.items()} if k in ['sources','sha256'] else v for k,v in snapshot.items()}
    scenario='experiments/aad_lane_change_v02/scenario.py'
    if hashlib.sha256(Path(scenario).read_bytes()).hexdigest()!=snapshot['sha256'][scenario]:
        raise ValueError('Scenario source differs from frozen stage-1 code')
    output.mkdir(parents=True,exist_ok=False)
    runner=output/'frozen_stage1_runner.py';runner.write_text(snapshot['sources']['experiments/aad_lane_change_v02/run.py'],encoding='utf-8')
    configs=read_json(source/'sweep_design.json')['configurations'];results=[]
    env=dict(os.environ);env['PYTHONPATH']=str(Path.cwd())+os.pathsep+env.get('PYTHONPATH','')
    for config in configs:
        name=config['scenario_id'];cp=output/(name+'.json');write_json(cp,config)
        cmd=[sys.executable,str(runner),'--config',str(cp),'--output',str(output/name),'--box-lanes','--record-geometry']
        r=subprocess.run(cmd,env=env,capture_output=True,text=True)
        (output/(name+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
        results.append(dict(config=config,output=str(output/name),exit_code=r.returncode));print(name,r.returncode,flush=True)
    write_json(output/'execution.json',results)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.source,a.output)
