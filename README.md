# AAD End-Semester Submission

## Project in one sentence

This project takes a roadside-CCTV-style video and a static camera calibration,
detects and tracks red proxy vehicles, estimates their positions on the road,
and produces behaviour tags that can help a human reviewer identify possible
AAD (Advanced/Ambulance Alerting and Detection) situations.

The delivered demo is an offline replay pipeline. It does not require
MetaDrive, and it does not claim to recognise real ambulances or measure real
road safety. The review labels are simulated, reproducible research outputs,
not real-world decisions.

## What the input is

The current demo uses **nine generated and saved test clips** stored in
[`samples/`](samples/). Each clip contains:

| Generated input file | Purpose |
|---|---|
| `input.mp4` | Recorded 10-fps proxy-vehicle video for one test scenario |
| `calibration.json` | Fixed camera geometry used to map image positions to a road coordinate system |
| `expected_report.json` | Saved frame numbers, tags and evidence that the replay is expected to reproduce |

The nine cases cover:

- two lane-change cases: clear context and a nearby receiving-lane vehicle;
- one pass-and-return case;
- one excursion with an aborted pass;
- ordinary, stronger and unseen braking cases;
- a clear crossing and a crossing with simultaneous interaction.

These videos are generated from simulated road scenes and contain ordinary red
vehicles as visual proxies. The project therefore measures whether the software
can recover scripted behaviours from representative offline inputs. It does not
measure performance on real CCTV, real traffic, emergency vehicles or humans.

The sample calibrations are derived for the fixed simulation cameras. They are
not measurements of a real roadside camera.

## What we test

The submission separates rule correctness from end-to-end replay:

1. **Unit tests** test individual rules and edge cases, including:
   - missing track observations must not be bridged;
   - stationary off-road patches must not become vehicles;
   - lane-change tags must change with receiving-lane context;
   - a pass requires return and order reversal;
   - crossing review tags depend on the interaction gap;
   - braking detection is not the same as simulator command timing.
2. **Saved-video replay tests** run the complete video-to-report pipeline for
   all nine clips. The replay compares the newly inferred frame count, event
   labels and frame ranges with each saved `expected_report.json`.
3. **Reference-audit evidence** contains scenario checks that compare camera
   outputs with simulator reference records:
   [`evidence/aad_video_roadside_holdout_01.json`](evidence/aad_video_roadside_holdout_01.json),
   [`evidence/aad_braking_holdout_01.json`](evidence/aad_braking_holdout_01.json),
   and [`evidence/aad_all_families_01.json`](evidence/aad_all_families_01.json).

## What is validated in this submission

The most direct validated result is **reproducibility**:

- six portable rule tests pass;
- all nine saved replay cases reproduced their saved event labels and frame
  ranges in the fresh run created by `run_aad.cmd all`;
- the replay output is written to a new folder under `aad_runs`;
- every report includes an event family, frame interval and supporting evidence.

The existing held-out and scenario evidence provides additional alignment
checks against simulator reference events. In particular, it can verify that
the expected lane-change or braking actor is associated with an observed
interval. These checks are limited: they are not independent real-world
annotation studies, and they do not establish event precision, recall,
abnormality, AAD urgency or risk.

An ordinary-braking sample is also intentionally retained as a documented
reference-subject miss. A matching replay validates that the miss is
reproducible; it does not indicate that the miss should be hidden or ignored.

## Run in Windows Command Prompt

Open Command Prompt at the repository root or in the extracted submission
folder. If the provided `.venv` already exists, no setup is required.

On a fresh computer, create the Python 3.11 environment once:

```cmd
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-aad.txt
```

Main replay commands:

```cmd
run_aad.cmd --list
run_aad.cmd
run_aad.cmd all
```

- `run_aad.cmd --list` prints the available sample names.
- `run_aad.cmd` replays `lane_change_concurrent`.
- `run_aad.cmd all` replays all nine saved clips.

For each run, open `aad_runs\<new-run>\RESULTS.md`. It links to every tagged
MP4 and report JSON. A report explains the frame interval and evidence used;
`summary.json` provides the machine-readable result summary.

Run the portable unit tests with:

```cmd
.\.venv\Scripts\python.exe -m unittest aad_cctv.test_video_tags aad_cctv.test_video_event_families
```

The demo can also accept a custom MP4 and calibration:

```cmd
run_aad.cmd custom --video C:\path\to\input.mp4 --calibration C:\path\to\calibration.json --layout straight
```

Use `--layout intersection` for an intersection-style crossing scene. Custom
inputs have no saved `expected_report.json`, so replay status is reported as
`Custom input` rather than `Match`.

## Read the code in pipeline order

1. [`aad_cctv/demo.py`](aad_cctv/demo.py)
   selects the replay case and creates a fresh output folder.
2. [`aad_cctv/video_tags.py`](aad_cctv/video_tags.py)
   reads the video, detects red proxy regions, tracks them, and projects track
   points using the supplied static calibration.
3. [`aad_cctv/roadside_video_tags.py`](aad_cctv/roadside_video_tags.py)
   applies lane-change and receiving-lane context rules.
4. [`aad_cctv/video_event_families.py`](aad_cctv/video_event_families.py)
   applies straight-road and intersection behaviour rules and writes the tagged
   video.
5. [`aad_cctv/evaluate_video_tags.py`](aad_cctv/evaluate_video_tags.py)
   and [`aad_cctv/audit_video_event_families.py`](aad_cctv/audit_video_event_families.py)
   provide the wider simulator/reference audit workflow.

The full workspace can also record fresh simulations in
[`aad_cctv/run.py`](aad_cctv/run.py)
and [`aad_cctv/crossing.py`](aad_cctv/crossing.py),
but the saved replay demo does not need those tools or MetaDrive.

## Scope and limitations

- The system detects red simulated vehicle proxies, not ambulances.
- Most tags describe physical behaviour or review context, not danger.
- A candidate review flag does not establish an abnormality or a safety-critical
  event.
- The speed values are geometry-derived proxy measurements, not certified real
  vehicle speeds.
- The validation set is scripted and small; it is not an independent test of
  real-world performance.
- Human annotation, real emergency-vehicle data and external expert review are
  needed before operational or clinical use.
