"""Run the AAD video pipeline on saved CCTV clips."""
import argparse
from contextlib import redirect_stdout
import json
from pathlib import Path
from tempfile import mkdtemp

from .camera import write
from .roadside_video_tags import run as tag_lane_change
from .video_event_families import run as tag_families
from .video_tags import infer

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def signature(report):
    return report['frames'], sorted(
        (e['family'], e['tag'], e['start_frame'], e['end_frame']) for e in report['tags']
    )


def replay(case, spec, output):
    video, calibration = (ROOT / spec[key] for key in ('video', 'calibration'))
    for path in (video, calibration):
        if not path.is_file():
            raise FileNotFoundError(path)
    work = output / case
    work.mkdir()
    result = work / 'result'
    with (work / 'run.log').open('x', encoding='utf-8') as log, redirect_stdout(log):
        infer(video, calibration, work / 'tracks', overlay=False)
        if spec['mode'] == 'lane_change':
            tag_lane_change(video, calibration, work / 'tracks', result)
        else:
            tag_families(video, calibration, work / 'tracks', result,
                         spec['layout'], spec['braking_window'])

    report = read(result / 'report.json')
    expected = spec.get('expected')
    # Open saved expected results only after video inference.
    matched = signature(report) == signature(read(ROOT / expected)) if expected else None
    print(f'{case}: {len(report["tags"])} tags | {result}', flush=True)
    return dict(case=case, frames=report['frames'], tags=report['tags'], matches_saved_replay=matched,
                video=f'{case}/result/tagged.mp4', report=f'{case}/result/report.json')


def main():
    cases = read(Path(__file__).with_name('demo_cases.json'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', nargs='?', default='lane_change_concurrent',
                        choices=[*cases, 'all', 'custom'])
    parser.add_argument('--list', action='store_true', help='List the saved cases')
    parser.add_argument('--output', type=Path, help='Fresh output directory')
    parser.add_argument('--video', type=Path, help='Custom input MP4')
    parser.add_argument('--calibration', type=Path, help='Static calibration for custom input')
    parser.add_argument('--layout', choices=['straight', 'intersection'])
    args = parser.parse_args()
    if args.list:
        print('\n'.join(cases))
        return 0
    if args.case == 'custom':
        if not all((args.video, args.calibration, args.layout)):
            parser.error('custom requires --video, --calibration and --layout')
        selected = {'custom': dict(video=str(args.video.resolve()),
                                   calibration=str(args.calibration.resolve()),
                                   mode='families', layout=args.layout, braking_window=30)}
    else:
        if any((args.video, args.calibration, args.layout)):
            parser.error('custom video options require the custom case')
        selected = cases if args.case == 'all' else {args.case: cases[args.case]}

    if args.output:
        output = args.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
    else:
        runs = Path.cwd() / 'aad_runs'
        runs.mkdir(exist_ok=True)
        output = Path(mkdtemp(prefix='replay_', dir=runs))
    print(f'Output: {output}', flush=True)
    results = [replay(case, spec, output) for case, spec in selected.items()]
    write(output / 'summary.json', dict(cases=results))
    lines = ['# AAD replay results', '',
             'Replay checks compare saved event labels and frame ranges.', '',
             '| Case | Frames | Tags | Saved replay | Files |',
             '|---|---:|---:|---|---|']
    for row in results:
        match = {True: 'Match', False: 'Different', None: 'Custom input'}[row['matches_saved_replay']]
        lines.append(f'| {row["case"]} | {row["frames"]} | {len(row["tags"])} | {match} | '
                     f'[Video]({row["video"]}) · [JSON]({row["report"]}) |')
    (output / 'RESULTS.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'Open {output / "RESULTS.md"}')
    return int(any(row['matches_saved_replay'] is False for row in results))


if __name__ == '__main__':
    raise SystemExit(main())
