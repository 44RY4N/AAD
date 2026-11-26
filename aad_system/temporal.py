"""Image-only track summaries. No simulator labels or actor identities are read."""
import json
from pathlib import Path


def run(source: Path,output: Path) -> None:
    if output.exists():raise FileExistsError(output)
    tracks={}
    for line in (source/'camera_observations.jsonl').read_text().splitlines():
        frame=json.loads(line)
        for track in frame['tracks']:tracks.setdefault(track['track_id'],[]).append(track)
    output.mkdir(parents=True,exist_ok=False);events=[]
    for identity,samples in tracks.items():
        first,last=samples[0],samples[-1]
        events.append(dict(track_id=identity,start_s=first['time_s'],end_s=last['time_s'],observations=len(samples),
            displacement_px=[last['centre'][i]-first['centre'][i] for i in range(2)],
            image_motion='rightward' if last['centre'][0]>first['centre'][0] else 'leftward_or_stationary',
            event_family=None,validity='unavailable',reason='Image motion alone does not identify subject, lane, road geometry or overtake completion',
            risk_assessment='unassessed'))
    (output/'track_motion.json').write_text(json.dumps(events,indent=2))
