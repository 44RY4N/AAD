"""Record the remaining selected/repeated representative conditions in fresh paths."""
import argparse
from pathlib import Path
import subprocess
import sys
from aad_analysis.io import write_json


def run(output,prefix='aad_lc2',all_conditions=False,delayed_context=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);results=[]
    configs=Path(__file__).with_name('configs')
    schedule=[('clear',False),('delayed',False),('concurrent',True),('clear',True),('delayed',True)]
    if all_conditions:schedule.insert(0,('concurrent',False))
    for condition,repeat in schedule:
        if condition=='delayed' and delayed_context:condition='delayed_context'
        name=prefix+'_'+condition+('_rgb_repeat_01' if repeat else '_rgb_01')
        target=Path('test_output')/name
        if target.exists():raise FileExistsError(target)
        cmd=[sys.executable,'-m','experiments.aad_lane_change_v02.run','--config',str(configs/(condition+'.json')),
             '--output',str(target),'--box-lanes','--record-rgb','--record-geometry']
        r=subprocess.run(cmd,capture_output=True,text=True)
        (output/(name+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8')
        results.append(dict(condition=condition,repeat=repeat,output=str(target),command=cmd,returncode=r.returncode))
        print(name,r.returncode,flush=True)
        write_json(output/(name+'_execution.json'),results[-1])
    write_json(output/'execution.json',results)


if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--prefix',default='aad_lc2');p.add_argument('--all',action='store_true')
    p.add_argument('--delayed-context',action='store_true')
    a=p.parse_args();run(a.output,a.prefix,a.all,a.delayed_context)
