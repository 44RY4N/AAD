"""Apply the calibrated road strip to video-only proxy tracks before tagging."""
import argparse
import json
from pathlib import Path

from aad_system.vision import sha
from .camera import write
from .video_tags import RULES, annotate, tag_tracks


ROAD_STRIP_Y_M = [1.75, 8.75]


def in_road_strip(track):
    point=track['xy_m']
    return point is not None and ROAD_STRIP_Y_M[0]<=point[1]<=ROAD_STRIP_Y_M[1]


def run(video, calibration, source, output):
    report=json.loads((source/'report.json').read_text())
    if (report['video_sha256']!=sha(video) or report['calibration_sha256']!=sha(calibration)
            or report['rules']!=RULES):
        raise ValueError('Derived tracks must match the video, static calibration and baseline rules')
    tracks=json.loads((source/'tracks.json').read_text())
    if len(tracks)!=report['frames']:
        raise ValueError('Frame count mismatch')
    frames=[[t for t in frame if in_road_strip(t)] for frame in tracks]
    tags=tag_tracks(frames)
    for tag in tags:
        tag['start_s']=tag['start_frame']/report['fps']
        tag['end_s']=tag['end_frame']/report['fps']
    output.mkdir(parents=True,exist_ok=False)
    static=json.loads(calibration.read_text())
    write(output/'report.json',dict(report,tags=tags,rules=dict(RULES,road_strip_y_m=ROAD_STRIP_Y_M),
          derived_from_code_sha256=report['code_sha256'],code_sha256=sha(Path(__file__))))
    write(output/'tracks.json',frames)
    annotate(video,output/'tagged.mp4',frames,tags,report['fps'],(static['width'],static['height']))
    print(json.dumps(tags,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('video','calibration','source','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args()
    run(args.video,args.calibration,args.source,args.output)
