"""Re-evaluate the archived CCTV batch without simulation or detector replay."""
import argparse
import json
from pathlib import Path

from .analyze import compare
from .evaluate import run as evaluate
from .temporal import temporal
from .camera import write

ARCHIVE = Path('analysis_output/cctv_execution_20261119_01')
RAW = Path('test_output')
ANALYSIS = Path('analysis_output')


def vision_jobs():
    jobs = json.loads((ARCHIVE/'vision_batch_results.json').read_text())
    # This view was run separately. Evaluation checks the saved camera lineage.
    return jobs + [dict(name='concurrent_roadside', returncode=0,
                        source='test_output/cctv_concurrent_20261119_final01/camera',
                        output='analysis_output/cctv_vision_roadside_20261119_01'),
                   dict(name='braking_ordinary', returncode=0,
                        source='test_output/cctv_braking_ordinary_20261125_04/camera',
                        output='analysis_output/cctv_vision_braking_ordinary_20261125_01')]


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    for job in vision_jobs():
        if job['returncode']:
            continue
        name = job['name']
        camera = Path(job['source'])
        source = next(parent for parent in camera.parents if (parent/'cctv_execution.json').exists())
        event = ANALYSIS/source.name/'event.json'
        vision = Path(job['output'])
        evaluate(source,camera,vision,output/name/'reference',event if event.exists() else None)
        temporal(camera,vision,output/name/'temporal',lane_divider_y=None if 'crossing' in name else 5.25)
    source = RAW/'cctv_concurrent_20261119_final01'
    write(output/'concurrent_headless.json',compare(source,RAW/'cctv_concurrent_20261119_headless_final01'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args().output)
