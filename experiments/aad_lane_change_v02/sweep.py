"""Predeclared factorial headless sweep, with every trial/failure retained."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
import sys
from aad_analysis.io import write_json


def configurations():
    root=Path(__file__).resolve().parents[1]
    base=json.loads((root/'aad_lane_change_v01/ordinary.json').read_text())
    result=[]
    for gap,speed,step in itertools.product([-28,-12,12],[20,30,40],[35,56]):
        result.append(dict(base,target_vehicle_gap_m=gap,target_vehicle_speed_kmh=speed,command_step=step))
    # One-factor probes around a fixed reference, not post-hoc trajectory editing.
    ref=dict(base,target_vehicle_gap_m=-28,target_vehicle_speed_kmh=30,command_step=35)
    for overrides in [dict(subject_speed_kmh=20),dict(subject_speed_kmh=40),
            dict(subject_longitude=80),dict(command_step=75),dict(idm_time_wanted_s=.75),dict(idm_time_wanted_s=2.)]:
        result.append(dict(ref,**overrides))
    result += [dict(base),dict(json.loads((root/'aad_lane_change_v01/interaction.json').read_text()))]
    for i,item in enumerate(result):item.update(scenario_id=f'lc2_trial_{i:03d}',condition_intent='controlled_sweep')
    return result


def run(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    configs=configurations();write_json(output/'sweep_design.json',dict(configurations=configs,
        design='18 full-factorial + 6 one-factor sensitivity probes + 2 reference controls',
        selection='Compare actual verified-window geometry; retain all failures; no criticality labels'))
    results=[]
    for cfg in configs:
        config=output/(cfg['scenario_id']+'.json');write_json(config,cfg)
        trial=output/cfg['scenario_id']
        command=[sys.executable,'-m','experiments.aad_lane_change_v02.run','--config',str(config),'--output',str(trial),'--box-lanes','--record-geometry']
        completed=subprocess.run(command,capture_output=True,text=True)
        (output/(cfg['scenario_id']+'.log')).write_text(completed.stdout+completed.stderr,encoding='utf-8')
        results.append(dict(config=cfg,output=str(trial),exit_code=completed.returncode))
        write_json(output/('result_'+cfg['scenario_id']+'.json'),results[-1])
        print(cfg['scenario_id'],completed.returncode,flush=True)
    write_json(output/'execution.json',results)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--output',type=Path,required=True);run(p.parse_args().output)
