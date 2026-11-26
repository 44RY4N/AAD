# Matched lane-change research prototype v0.1

Executed 2026-11-15 with the existing Windows Python 3.11.9 venv, MetaDrive 0.4.3
and Panda3D 1.10.16. This is a working simulator-ground-truth research prototype,
not a calibrated ambulance model or abnormality classifier.

## Implementation and taxonomy

New `run.py` is an isolated adaptation of the validated experiment runner, using
the same three-lane 500 m road, `--box-lanes`, collision/termination checks,
640x360 elevated chase RGB camera and runtime geometry recorder. `scenario.py`
supplies scheduled steering targets and configurable traffic. Existing runner,
configurations, v0.1/v0.2 extractors and all old outputs remain unchanged.

Both conditions start at rest, use subject desired speed 30 km/h, leader desired
speed 10 km/h with 24 m reference gap, lane width 3.5 m and seed 0. Lane 1 is the
source, lane 0 the target. At action call 56 the subject steering controller is
given lane 0 as its target; before that it follows lane 1. This is an explicitly
scheduled manoeuvre rather than unmodified IDM discretionary lane selection.
Longitudinal acceleration remains IDM using detected front traffic on the
steering target lane. No vehicle is teleported, no lane index is assigned, no
speed is forced, and no termination/collision check is suppressed.

`ordinary.json`: target-lane vehicle starts +100 m relative to subject reference,
desired speed 30 km/h. `interaction.json`: target vehicle starts -16 m, desired
speed 40 km/h. These two parameters and descriptive IDs/intents are the only
between-condition differences. They are configurable JSON values, not initial
realized speeds. The target actor retains recorder-compatible role
`background_far`; use this role when joining it, regardless of its actual proximity.
The source-lane leader and opposite-side background actor are also recorded.

The existing taxonomy already separates physical event, attention context,
conflict evidence, evasive response and outcome. It was frozen verbatim in
`research/aad_v0.1/AAD_TAXONOMY_v0.1_SUPERVISOR_REVIEW_FROZEN.md`, with source hash,
pending supervisor approval. These experiments map to family A lane change;
leader-passing motivation does not establish a full overtaking event (family B).
Speed adaptation is motion evidence, not an automatically evasive or abnormal
braking event (C). Family D crossings are not implemented here. The separate
literature traceability matrix links these choices to evidence and future work.

## Executed results and important deviation

| Observed measurement | Ordinary intent | Interaction intent |
| --- | ---: | ---: |
| Decision steps / synchronized RGB frames | 300 / 301 | 300 / 301 |
| Physical departure / target-centre settled / confirmation steps | 60 / 75 / 85 | 75 / 120 / 130 |
| Departure-to-confirmation sampled duration | 2.5 s | 5.5 s |
| Lateral displacement at confirmation | -3.704486 m | -3.798609 m |
| Subject speed at departure | 15.5647 km/h | 5.61483 km/h |
| Target traffic speed at departure | 29.9999 km/h | 39.9998 km/h |
| Target-pair minimum envelope separation during verified event | 109.773090 m | 17.673280 m |
| Target-pair minimum over entire 0-30 s recording | 95.499051 m | 1.408174 m |
| Entire-interval target-pair minimum step | 1 | 59 |
| Target-pair finite valid TTC samples during event | 0 | 0 |
| Target-pair undefined TTC samples during event | 26 | 56 |
| Recorded actor vehicle-contact samples | 0 | 0 |

Both variants physically complete the lane change and all actor samples stay
localized on the controlled road. However, the interaction vehicle passes before
the verified lateral departure. The subject slows and merges later rather than
producing the intended close concurrent cut-in. This is the closest defensible
realized alternative: a challenging approach/speed-adaptation/delayed-merge
comparison. It demonstrates measurable different geometry/motion, not a proven
near-crash or critical lane change. Do not present the 1.408 m minimum at step 59
as a minimum inside the verified event window 75..130. Undefined TTC is not safety.
Both condition names describe intent; both risk assessments remain unassessed.

The unchanged physical verifier uses sampled lateral thresholds and 11 settled
samples. Its departure is not earliest physical motion or first footprint
intrusion; confirmation is not a full pass/return or arbitrary whole-footprint
completion proof. No universal safety threshold was introduced.

## Captures, analysis and validation

Successful RGB+geometry captures:

* `test_output/aad_lc_ordinary_02`
* `test_output/aad_lc_ordinary_repeat_01`
* `test_output/aad_lc_interaction_01`
* `test_output/aad_lc_interaction_repeat_01`

`test_output/aad_lc_interaction_probe_01` is the successful headless geometry
probe, with the same interaction trajectory checksum. The first sandboxed RGB
attempt `aad_lc_ordinary_01` failed texture loading; retained intact. Graphics
asset access outside the sandbox resolved it without environment changes.

Analysis directories are `analysis_output/aad_lc_{ordinary,interaction,
ordinary_repeat,interaction_repeat}_01/`. Each has:

```text
scenario_provenance.json  # new module hashes/settings, supplementary to old exporter list
summary.json
relative_motion.jsonl    # 301 target-pair relative s/l, vx/vy, ax/ay samples
event_annotation.json    # actual observation, unchanged annotation schema 0.1
event_validation.json
v01/                    # unchanged v0.1 motion / reference-point feature outputs
v02/                    # unchanged v0.2 geometry / occupancy / TTC feature outputs
```

Raw directories preserve metadata/settings/initial placements, CSV positions,
orientations, lane IDs, collisions, policies, localization rays, 301 RGB PNGs,
frame metadata and geometry sidecars with direct 3D velocity, actor shapes,
road geometry and camera calibration. Acceleration/relative acceleration is an
offline central-difference estimate from unchanged v0.1 derivatives, with future
support and null endpoints, not a direct accelerometer or braking-intent signal.
All per-step envelope gaps/occupancy/TTC remain in v02 outputs with original
validity/reason semantics; summaries do not replace those records.

Existing recording validators decoded every image and checked all frame/state
joins, event phases and exact repeat trajectories/decisions. Geometry validators
passed every capture (301 samples / 1,204 actor samples), with native-transform
and projection cross-checks. Both extractors accepted all four captures. v0.2
outputs each contain 1,204 actor records, 1,806 unordered pair records, 1,204
occupancy records and six pair summaries. Separate v0.2 validation fully
recomputed all four outputs and checked source/code/artifact hashes.

Within-condition repeats match original state trajectories, policy decisions,
event results, normalized geometry and both normalized feature versions exactly.
Trajectory hashes excluding generated IDs:

* Ordinary: `53aa7c91f883d451c3657acbfb42abb11ca6a4003e4790f4f9daaae982be6b55`
* Interaction: `2db27a12ee428167ebbcc601618442d0be5f97c7ed76c1a6d991c706ab0933c0`

v0.2 feature fingerprints:

* Ordinary: `2ecec2ce02dc98ed6801c1a1e3cff4f43af7c12bec2ea8a3bfa58a352106d9c9`
* Interaction: `2d8f5e097d4d5fbf904c5d2d989effdb454903d1825d7ee605e5159300ecf674`

The comparison is saved in `analysis_output/aad_lane_change_comparison_01/`.
79 tests passed: four new scenario/annotation/repeat checks and all 75 existing
geometry, audit, v0.2 and v0.1 tests. Local schema validation covers the constructs
used by the frozen schema; it is explicitly not a general JSON Schema engine.
Actor/step/source references and real event times were also checked.

Camera centre/frustum checks pass for the comparison actor throughout each event,
but frustum is not occlusion or usable visual resolution. The interaction frame
at step 75 was visually inspected: subject and nearby traffic are visible.
Ordinary comparison traffic is distant; it is not a strong visual detection target.
The unchanged camera is a virtual elevated chase view, not an ambulance dashcam.

## Reproduce with fresh directories

PowerShell, repository root; this runs two RGB captures and two analyses per
condition. Rendering requires normal graphics/asset access (the agent's sandbox
needed approved access). The timestamp avoids overwriting old evidence.

```powershell
cd C:\Users\aarya\repos\metadrive
$tag = Get-Date -Format 'yyyyMMdd_HHmmss'
foreach ($condition in @('ordinary', 'interaction')) {
    $config = "experiments/aad_lane_change_v01/$condition.json"
    $first = "test_output/aad_lc_${condition}_${tag}"
    $repeat = "test_output/aad_lc_${condition}_repeat_${tag}"
    .\.venv\Scripts\python.exe -m experiments.aad_lane_change_v01.run --config $config --output $first --box-lanes --record-rgb --record-geometry
    if ($LASTEXITCODE -ne 0) { throw 'First capture failed' }
    .\.venv\Scripts\python.exe -m experiments.aad_lane_change_v01.run --config $config --output $repeat --box-lanes --record-rgb --record-geometry
    if ($LASTEXITCODE -ne 0) { throw 'Repeat capture failed' }
    $analysis = "analysis_output/aad_lc_${condition}_${tag}"
    $repeatAnalysis = "analysis_output/aad_lc_${condition}_repeat_${tag}"
    .\.venv\Scripts\python.exe -m experiments.aad_lane_change_v01.analyze --source $first --reference $repeat --output $analysis
    if ($LASTEXITCODE -ne 0) { throw 'First analysis failed' }
    .\.venv\Scripts\python.exe -m experiments.aad_lane_change_v01.analyze --source $repeat --reference $first --output $repeatAnalysis
    if ($LASTEXITCODE -ne 0) { throw 'Repeat analysis failed' }
    .\.venv\Scripts\python.exe -m aad_analysis_v02.validate --output "$analysis/v02"
    .\.venv\Scripts\python.exe -m aad_analysis_v02.validate --output "$repeatAnalysis/v02" --compare "$analysis/v02"
}
.\.venv\Scripts\python.exe -m unittest experiments.aad_lane_change_v01.test_pipeline audits.test_geometry_boundary_v02 aad_analysis_v02.test_measurements aad_analysis_v02.test_pipeline experiments.test_geometry aad_analysis.test_pipeline
```

The scenario tests currently include checks against the named preserved captures
from this task. Headless execution remains optional: omit `--record-rgb` and keep
`--record-geometry` for a run, then use the existing geometry validator/extractor;
the combined `analyze` command intentionally requires RGB as well.

## Remaining research work

The requested close concurrent target-lane conflict was not demonstrated: IDM
adaptation delayed the merge. A later explicitly approved parameter/controller
study could target concurrent occupancy, retaining wait/abort/contact outcomes.
No threshold tuning or forced collision was performed to create a desired label.
Independent event/criticality review, matched sweeps/held-out seeds, visibility
evaluation, real-world transfer, calibrated ambulance dynamics, sirens/priority,
full overtakes, braking/crossing event detectors and perception remain missing.
v0.2 planar envelope and near-zero/TTC limitations still apply; v0.3 remains an
unapproved design draft. No classifier, risk score or injury estimate is produced.
