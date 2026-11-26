"""Read-only geometry/state and RGB decoding checks independent of event family."""
import csv
import json
from pathlib import Path
from PIL import Image,ImageStat
from experiments.validate_geometry import validated_data


def check(source: Path) -> dict:
    report,_,_=validated_data(source)
    camera=source/'camera/frames.jsonl'
    if camera.exists():
        with (source/'trajectory.csv').open(newline='') as file:rows=list(csv.DictReader(file))
        grouped={}
        for r in rows:grouped.setdefault(int(r['step']),[]).append(r)
        frames=[json.loads(s) for s in camera.read_text().splitlines()]
        for f in frames:
            assert [{k:str(v) for k,v in r.items()} for r in [f['subject']]+f['traffic']]==grouped[f['step']]
            with Image.open(source/'camera'/f['file']) as im:
                im.load();assert im.mode=='RGB' and im.size==(640,360)
                assert max(ImageStat.Stat(im).stddev)>1
        report.update(rgb_images_decoded=len(frames),all_camera_states_match=True)
    return report
