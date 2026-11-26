"""Read actual generated X-block topology and available assets into a new file."""
import argparse
import json
from pathlib import Path
from metadrive import MetaDriveEnv
from metadrive.component.map.pg_map import MapGenerateMethod


if __name__ == '__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    env=MetaDriveEnv(dict(map_config={'type':MapGenerateMethod.BIG_BLOCK_SEQUENCE,'config':'X',
        'lane_num':2,'lane_width':3.5,'exit_length':80},traffic_density=0,use_render=False,
        random_spawn_lane_index=False,num_scenarios=1,start_seed=0))
    try:
        env.reset(seed=0)
        lanes=[]
        for start, ends in env.current_map.road_network.graph.items():
            for end, items in ends.items():
                for i,lane in enumerate(items):
                    lanes.append(dict(index=[start,end,i],type=type(lane).__name__,length=float(lane.length),
                        start=list(map(float,lane.position(0,0))),end=list(map(float,lane.position(lane.length,0))),
                        heading=float(lane.heading_theta_at(0))))
        a.output.write_text(json.dumps(dict(lanes=lanes,config=repr(env.config)),indent=2))
        print(json.dumps([l for l in lanes if l['type']=='StraightLane'],indent=2))
    finally:env.close()
