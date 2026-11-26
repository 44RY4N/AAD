"""Small preregistered crop-scale pilot on saved fixed-camera images."""
import argparse
import json
from itertools import islice
from pathlib import Path

import cv2

from aad_system.vision import Detector, sha
from .camera import ground_point, write
from .events import lines
from .vision import observations, tiled_detections


def run(camera, baseline, weights, output):
    report = json.loads((baseline/'report.json').read_text())
    if (report['model_sha256'] != sha(weights)
            or report['source_manifest_sha256'] != sha(camera/'observations.jsonl')
            or report['calibration_sha256'] != sha(camera/'calibration.json')):
        raise ValueError('Baseline weights, camera frames or calibration changed')
    calibration = json.loads((camera/'calibration.json').read_text())
    output.mkdir(parents=True, exist_ok=False)
    detector = Detector(weights, confidence=report['confidence'])
    saved = {row['step']: row for row in lines(baseline/'camera_observations.jsonl')}
    comparisons = []
    with (output/'camera_observations.jsonl').open('x', encoding='utf-8') as file:
        # Identical sample across conditions: 12 captures spaced by 15 frames.
        for record, path in islice(observations(camera), 0, 180, 15):
            image = cv2.imread(str(path))
            if image is None:
                raise ValueError(f'Cannot read {path}')
            detections = tiled_detections(detector,image)
            baseline_row = saved[record['step']]
            if baseline_row['image_sha256'] != record['image_sha256']:
                raise ValueError('Baseline observation hashes differ')
            comparisons.append(dict(step=record['step'], baseline=len(baseline_row['detections']), cropped=len(detections)))
            for index, result in enumerate(detections):
                x1, _, x2, y2 = result['box_xyxy']
                estimate, reason = ground_point([(x1+x2)/2, y2], calibration)
                result.update(track_id=f"sample-{record['step']}-{index}",
                              estimated_ground_contact_world_m=estimate, ground_estimate_status=reason)
            file.write(json.dumps(dict(step=record['step'], time_s=record['time_s'],
                                       image_sha256=record['image_sha256'], tracks=detections), allow_nan=False)+'\n')
    write(output/'report.json', dict(samples=comparisons, model_sha256=sha(weights),
          method='Two overlapping full-height 60%-width crops, same YOLOX weights and confidence, classwise IoU .5 merge',
          source_camera=str(camera), baseline=str(baseline), identity='Per-frame synthetic keys for projected association only'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('camera', 'baseline', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--weights', type=Path, default=Path('models/yolox_opencv_2022nov/model.onnx'))
    args = parser.parse_args()
    run(args.camera, args.baseline, args.weights, args.output)
