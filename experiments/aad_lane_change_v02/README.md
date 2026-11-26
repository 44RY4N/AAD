# Controlled concurrent lane-change experiment, generation 2

Executed 2026-11-15, Windows, existing Python 3.11.9 venv, MetaDrive 0.4.3 and
Panda3D 1.10.16. Separate experiment code/configurations/outputs; no simulator,
v0.1/v0.2 feature semantics, taxonomy or historical baseline was changed.
All criticality assessments remain **unassessed**.

## Result

A reproducible concurrent interaction is now observed inside the verified
lane-change window. The subject merges with a closing rear vehicle in the target
lane. Both footprints occupy that lane at the same recorded steps; the existing
constant-velocity/fixed-orientation envelope model predicts future intersection
at steps 60..67. The follower subsequently slows; no vehicle contact is recorded
in this representative. This is measurable model-based interaction evidence,
not a claim of criticality, exact physical clearance or an inevitable collision.

| Actual result | A: clear | B: concurrent rear interaction | C: delayed |
| --- | ---: | ---: | ---: |
| Target reference spawn gap, m | +100 | -24 | -12 |
| Target desired speed, km/h | 30 | 30 | 30 |
| Subject desired speed, km/h | 30 | 30 | 30 |
| Steering target command step | 56 | 56 | 56 |
| Departure / settled / confirmed step | 60 / 75 / 85 | 60 / 75 / 85 | 80 / 147 / 157 |
| Command-to-departure delay | 4 steps | 4 steps | 24 steps |
| Subject departure speed, km/h | 15.564675 | 15.564675 | 5.048188 |
| Minimum target-pair envelope separation **within event**, m | 109.773090 | 2.151888, step 69 | 15.157944 |
| Minimum over whole recording, m | 95.499051 | 2.151888 | 1.343859 |
| Valid finite model TTC samples within event | 0 | 8, steps 60..67 | 0 |
| Minimum valid event model TTC, s | null | 0.9885595144, step 66 | null |
| Representative vehicle-contact samples | 0 | 0 | 0 |

Every representative completes 300 steps with 301 synchronized samples/frames
and 1,204 actor samples. The existing verifier's departure is a sampled lateral
threshold, not earliest motion; confirmation is 11 settled samples, not full
overtake or a general footprint-completion proof. C's whole-recording minimum
must not be presented as occurring inside its 80..157 event window.

## Why the original merge was delayed

Inspected `aad_lane_change_v01/scenario.py`, the runner, `FrontBackObjects`,
`IDMPolicy.acceleration/desired_gap/steering_control`, stock lane-change routing,
vehicle `before_step/after_step`, manager spawning/policy lifecycle and saved
decisions/trajectories. The old scheduled controller immediately switches
front-object lookup to the *target* lane at command step 56, even while the
subject is mostly in its source lane. It passes that front reference gap into IDM.

In the original interaction recording, step 55's target actor is 1.3833 m ahead
by reference x. The action at 56 uses a front distance of 1.3833 m instead of the
source leader gap. IDM's desired-gap-over-distance squared term causes braking.
Subject speed falls from 15.0036 km/h at 55 to 0.4541 at 59 and 0.1087 at 65.
Lateral displacement remains small while almost stopped, despite steering toward
the target lane. The target vehicle clears first, and verified departure occurs
at 75. This is not the controller deciding to abort or a localization failure.
PID steering controls orientation/lateral error; actual translation depends on
vehicle dynamics and longitudinal motion. The gap is a reference quantity, not
bumper clearance.

Generation 2 retains this scheduled steering/IDM longitudinal design so waiting
and speed response remain observable. It logs steering, longitudinal command,
actual subject speed, active target speed and IDM settings per decision. It does
not force lane IDs, teleport, override realized velocities, disable collisions,
or hold the simulator alive after termination. There is no implemented abort
state machine: slowing/delaying and naturally terminated manoeuvres are observed;
an intentional controller abort is not demonstrated.

## Sweep design, implementation issues and selection

Stage 1: full factorial gaps {-28,-12,+12} m x target desired speeds {20,30,40}
km/h x command steps {35,56}: **18 cells**. Six one-factor probes change subject
desired speed (20/40), subject starting longitude (40 to 80), command timing (75),
or subject IDM desired headway (.75/2 s), plus two previous-condition references:
**26 configurations total**. All actors spawn at rest; relative *realized*
velocity emerges through acceleration and surrounding traffic, and is recorded.
Changing subject starting longitude shifts the whole configured group; it tests
placement/translation rather than changing only a pair's gap.

Two findings are preserved rather than hidden:

* The initial `sweep_design.json` description mistakenly says 36 factorial cells;
  its actual explicit configuration list has 18, and 26 total. The preserved
  `controller_snapshot.json` documents the correction; current driver text is fixed.
* The copied runner changed `NORMAL_SPEED` but not active `target_speed` for the
  overridden scheduled policy. Thus first-stage subject-speed probes 018/019
  requested 20/40 but actually retained target 30. They are unsuccessful
  interventions, not evidence that desired speed has no effect. Generation 2
  now initializes both properties. Runtime logs confirm active 20/40 in the
  corrected refinement probes. No v0.1 runner or extractor was changed.

Refinement: bracket successful and contact-terminated cells using gaps
{-24,-20,-16} x target speeds {25,30} x command steps {45,56}: 12 cells; add two
corrected subject-speed probes: **14 configurations**. The gap bracket follows
first-stage safe/contact outcomes; it is not an arbitrary threshold search or
per-step manual intervention. Configurations are generated before each batch.

Across **40 trials**, 32 complete with verified merges and eight naturally
terminate with vehicle-contact reports. All contact trials are retained. No
termination or collision setting was relaxed and no collision was selected as
the desired representative. No intentional abort was reproduced. Quantitative
results for every trial are in [SWEEP_RESULTS.md](SWEEP_RESULTS.md) and the
machine-readable sweep comparison. There is no risk score or safety threshold.
Seven completed trials satisfy the separate same-step occupancy plus finite
model-TTC interaction criterion; the representative B is one of these outcomes.

Selection uses observed event-window evidence, not the condition name:

* A = stage-1 trial 024, clear reference.
* B = refinement 003 (-24 m, 30 km/h, command 56), with finite model TTC and
  simultaneous target-lane footprint occupancy during the verified event.
* C = stage-1 trial 009 (-12 m, 30 km/h, command 56), with near-stop/delayed merge.

`screen.py` also reports a stricter diagnostic named `concurrent_steps`: current
longitudinal footprint projections overlap *and* both have target-lane occupancy.
It is empty for these completed runs. That stronger current-overlap condition
is distinct from the required approaching/concurrent interaction. B's model
witness is not relabelled as current projection overlap or contact.
`evidence.py` explicitly records the separate same-step occupancy plus positive
finite model-TTC criterion, using the unchanged five-second prediction horizon
(a model setting, not a criticality cutoff). At step 66 subject occupancy is
about 43.84%, target occupancy about 100%, and rear relative vx is +2.735 m/s.

In B the target vehicle slows from approximately 30 km/h at 66 to 21.620 at 69
and 0.267 at 75, while the subject completes the merge. These are recorded
kinematics/IDM response, not independently reviewed evasive intent or proof of
human-like response. C nearly stops before merging later. A/B hold subject
control/timing fixed and differ only in target starting gap.

## Recordings, visibility failure and explicit camera variant

Original-camera representative captures are
`test_output/aad_lc2_{clear,concurrent,delayed}_rgb_01` and `_rgb_repeat_01`.
All six preserve the original 640x360 RGB camera, mount (0,-22,16), look-at
(0,15,0), timing and state guards.

The camera exposed two genuine limitations. B's rear actor centre is outside
the original frustum throughout the 26 event samples (parts of its body appear).
C's source leader leaves the original view at step 154, so the unchanged RGB
validator rejects its requirement that the leader remain visible throughout the
event. This failed validation was retained; no validator was weakened.

An **explicit opt-in** camera-pose variant uses the same sensor/lens/resolution,
capture/channel checks and geometry calibration, with mount (0,-35,22) and look-at
(0,8,0). It changes observation only. The baseline remains unchanged. Additional
captures are `aad_lc2_concurrent_context_rgb_01` and
`aad_lc2_delayed_context_rgb_{01,repeat_01}`. B's target centre is in-frustum for
22/26 event samples; at the peak model interaction it is fully visible. C's
leader and target centres are both in-frustum for all 78 event samples. Frustum
presence is not occlusion or image-quality certification.

Context-camera trajectory/event equality is checked against the original view;
B's normalized v0.2 features match exactly. Context C and its repeat retain the
same vehicle trajectory as original C, while satisfying the existing visibility
validator. This is still a virtual elevated chase camera, not an ambulance
dashcam. No emergency lights, sirens, priority or calibrated ambulance dynamics
are claimed.

RGB evidence (B's step 66 is inside the actual 60..85 event and model-witness
interval 60..67):

![Concurrent interaction, explicit rear-context camera, step 66](../../test_output/aad_lc2_concurrent_context_rgb_01/camera/frames/000066.png)

The subject is the central vehicle; the target-lane follower is behind/left.
The corresponding unmodified-view [frame](../../test_output/aad_lc2_concurrent_rgb_01/camera/frames/000066.png)
shows why rear evidence cannot be assumed visible. Also see
[clear frame 66](../../test_output/aad_lc2_clear_rgb_01/camera/frames/000066.png) and
[delayed context frame 80](../../test_output/aad_lc2_delayed_context_rgb_01/camera/frames/000080.png).

## Analysis, annotation and output contract

* `test_output/aad_lc2_sweep_01` and `aad_lc2_refine_01`: every generated JSON,
  raw capture, subprocess log and exit status. Stage snapshots preserve actual
  pre-fix code. Stage-1 contact runs retain pre-terminal trajectory/geometry and
  terminal crash details in logs/rays; refinement additionally captures the actual
  terminal state and `termination.json` without advancing another physics step.
  Incomplete geometry is not falsely marked complete for standard extraction.
* `analysis_output/aad_lc2_{sweep,refine}_01`: unchanged v0.2 geometry extraction
  for all 32 completed headless trials plus failure summaries for the other eight.
* `analysis_output/aad_lc2_motion_01`: unchanged v0.1 `actor_features` API reused
  for all 32 completed camera-free trials, with actor derivatives and relative
  velocity/acceleration. This explicitly is not the v0.1 RGB-requiring CLI schema.
* `analysis_output/aad_lc2_representatives_01`: six fully analyzed captures
  (clear original/repeat, concurrent original/repeat, delayed context/repeat),
  both unchanged extractors, sampled relative motion, event annotations and
  validity-aware `window_evidence.json`. `annotated_event.json` uses the unchanged
  annotation schema, links real IDs/steps/sources and adds model evidence while
  leaving calibrated conflict interpretation indeterminate and risk unassessed.
* `analysis_output/aad_lc2_concurrent_context_01`: the additional visible B
  capture, accepted by both extractors/recording validators with exact motion
  comparison to its original-camera capture.

Geometry outputs retain dimensions/individual components, full pose, direct
velocity, road/lane geometry and camera transforms. Acceleration is explicitly
an offline 10 Hz derivative, with future support and null endpoints, not a direct
accelerometer or observed brake-pedal command. Lane occupancy, longitudinal
envelope gaps and model TTC retain v0.2 validity/reasons. Undefined or censored
TTC is never a safety finding. Envelope separation is not exact Bullet clearance.
All event comparisons use actual inclusive departure..confirmation windows.

The first representative finalization stopped at the genuine C visibility
failure. Explicit `--resume` reused the already completed A/B analyses, without
rewriting them, and added C's validated context-camera analyses. Completed output
directories cannot be resumed/overwritten. Failed cases are not annotations of
successful merges. Raw termination evidence remains available for independent
review; no new collision-risk category is inferred from it.

## Reproduction commands

Executed validation: **85 tests passed**, including all 79 previous tests and six
new checks for the factorial design, frozen replay source, active desired speeds,
terminal-state retention, actual event-window witnesses and representative
annotation/repeat consistency. All six selected captures passed existing RGB and
geometry validators and both feature extractors; their v0.2 artifacts passed full
recomputation. The additional B context capture also passed and has identical
vehicle features to B's original view. Within-condition trajectory, policy, event
and normalized v0.1/v0.2 features repeat exactly. These are repeatability checks,
not independent safety validation.

From PowerShell at the repository root. Rendering needs normal graphics/asset
access; the agent used approved access as in earlier captures. Use a fresh tag.
These commands reproduce the corrected sweep and selected cases. To reproduce
the **historical pre-fix** first stage exactly, use the frozen replay alternative.

```powershell
cd C:\Users\aarya\repos\metadrive
$tag = Get-Date -Format 'yyyyMMdd_HHmmss'
$sweep = "test_output/lc2_sweep_$tag"
$refine = "test_output/lc2_refine_$tag"
$screen = "analysis_output/lc2_sweep_$tag"
$refineAnalysis = "analysis_output/lc2_refine_$tag"
$prefix = "lc2_$tag"

# Corrected design: active desired-speed probes now take effect.
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.sweep --output $sweep
# Historical alternative, instead of the preceding command:
# .\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.replay_stage1 --source test_output/aad_lc2_sweep_01 --output $sweep
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.refine --output $refine
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.screen --source $sweep --output $screen
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.screen --source $refine --output $refineAnalysis
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.headless_motion --screenings $screen $refineAnalysis --output "analysis_output/lc2_motion_$tag"
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.capture_batch --all --delayed-context --prefix $prefix --output "test_output/lc2_capture_logs_$tag"
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.finalize --prefix $prefix --delayed-context --sweep $screen --refinement $refineAnalysis --output "analysis_output/lc2_representatives_$tag"

# Additional B camera evidence, with original-controller trajectory comparison:
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v02.run --config experiments/aad_lane_change_v02/configs/concurrent_context.json --output "test_output/lc2_context_$tag" --box-lanes --record-rgb --record-geometry
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v01.analyze --source "test_output/lc2_context_$tag" --reference "test_output/${prefix}_concurrent_rgb_01" --output "analysis_output/lc2_context_$tag"

.\.venv\Scripts\python.exe -m unittest experiments.aad_lane_change_v02.test_pipeline experiments.aad_lane_change_v01.test_pipeline audits.test_geometry_boundary_v02 aad_analysis_v02.test_measurements aad_analysis_v02.test_pipeline experiments.test_geometry aad_analysis.test_pipeline
```

Sweeps deliberately continue after a trial returns a genuine failure, retaining
its log. Stop downstream commands if a batch itself fails before producing its
execution/comparison manifest. Tests include the preserved named evidence from
this task; moving/deleting it requires a separately updated fixture policy.

## Limitations

One seed, DefaultVehicle proxies, straight same-direction road and scheduled
steering only. A deterministic repeat is not held-out or external validation.
No implemented intentional abort/return manoeuvre, human gap acceptance,
emergency-priority response, perception model or risk calibration. Contact-ended
trials do not establish injury or model reliability. The finite-horizon TTC holds
orientation/velocity fixed; actual actors adapt. Numerical near-touch semantics
remain v0.2; the v0.3 draft was not implemented. Independent event/response review,
held-out sweeps and deployment-relevant visibility remain future work.
