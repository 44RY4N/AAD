"""Shared legacy runner adapter with explicitly timed leader desired-speed changes."""
import argparse
import json
from pathlib import Path
from unittest.mock import patch
from experiments.aad_overtake_v01 import run as runner
from experiments.aad_overtake_v01.scenario import ObservedIDM,ControlledTraffic as OriginalTraffic


class BrakingTraffic(OriginalTraffic):
    def populate(self,settings):
        self.settings=settings;self.calls=0;super().populate(settings)
    def before_step(self):
        self.calls+=1
        if self.calls==self.settings['brake_step']:
            leader=next(v for role,v in self.vehicles if role=='leader')
            policy=self.get_policy(leader.id)
            policy.NORMAL_SPEED=self.settings['leader_after_kmh'];policy.target_speed=self.settings['leader_after_kmh']
        return super().before_step()


def main() -> None:
    p=argparse.ArgumentParser(__doc__);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--box-lanes',action='store_true');p.add_argument('--record-geometry',action='store_true');p.add_argument('--record-rgb',action='store_true')
    a=p.parse_args();settings=json.loads(a.config.read_text())
    if settings['leader_after_kmh']<=0:raise ValueError('IDM requires positive post-change desired speed')
    settings['negative_control']=True
    # Scoped adapter, restored on exit; separate CLI subprocess isolates legacy
    # class-level settings. Historical source files are never edited or copied.
    with patch.object(runner,'ControlledTraffic',BrakingTraffic):
        runner.run(settings,a.output,box_lanes=a.box_lanes,record_rgb=a.record_rgb,record_geometry=a.record_geometry)


if __name__=='__main__':main()
