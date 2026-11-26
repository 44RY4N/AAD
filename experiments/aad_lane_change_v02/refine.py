"""Declared refinement between successful and contact-terminated first-stage cells."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
import sys
from aad_analysis.io import write_json


def run(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    base=json.loads((Path(__file__).parents[1]/'aad_lane_change_v01/ordinary.json').read_text())
    configs=[dict(base,target_vehicle_gap_m=g,target_vehicle_speed_kmh=v,command_step=s)
             for g,v,s in itertools.product([-24,-20,-16],[25,30],[45,56])]
    ref=dict(base,target_vehicle_gap_m=-28,target_vehicle_speed_kmh=30,command_step=35)
    configs.extend([dict(ref,subject_speed_kmh=v) for v in [20,40]])
    for i,cfg in enumerate(configs):cfg.update(scenario_id=f'lc2_refine_{i:03d}',condition_intent='refinement_sweep')
    write_json(output/'design.json',dict(configurations=configs,reason='Bracket safe -28m/30kmh versus contact at closer/faster first-stage cells. Correct active subject target_speed for OAT. No collision suppression or risk thresholds.'))
    results=[]
    for cfg in configs:
        cp=output/(cfg['scenario_id']+'.json');write_json(cp,cfg);trial=output/cfg['scenario_id']
        command=[sys.executable,'-m','experiments.aad_lane_change_v02.run','--config',str(cp),'--output',str(trial),'--box-lanes','--record-geometry']
        r=subprocess.run(command,capture_output=True,text=True)
        (output/(cfg['scenario_id']+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
        result=dict(config=cfg,output=str(trial),exit_code=r.returncode);results.append(result)
        write_json(output/('result_'+cfg['scenario_id']+'.json'),result);print(cfg['scenario_id'],r.returncode,flush=True)
    write_json(output/'execution.json',results)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--output',type=Path,required=True);run(p.parse_args().output)
