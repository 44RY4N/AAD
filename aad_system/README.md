# Integrated research baseline: executed scope and acceptance gaps

2026-11-15. This increment adds an actual CPU vision path and braking adapter.
It does **not** meet the requested 90% acceptance test. Historical packages,
geometry semantics, source recordings and annotations are retained.

## Executed architecture

`python -m aad_system` exposes registry, preflight/simulation, read-only validation,
geometry extraction, RGB vision, image temporal summaries and review entry points.
Simulation adapters reuse existing runners without copying them. Scoped patching
in the braking adapter is restored at exit and isolated in a subprocess; this is
an interim adapter around legacy class-level settings, not a fully independent
scenario-engine interface. Families A/B reuse validated lane-change/overtaking;
C is newly experimental; D is discoverable but not implemented. Simulator class
availability does not certify pedestrian or legal priority behaviour.

New `vision.py` executes YOLOX-S ONNX with existing OpenCV5.0 CPU DNN, then a
greedy class-aware IoU tracker with missed-frame lifetime3 and matching gate .2.
Inference and tracking receive images/boxes/time only. Geometry projection and
IoU association occur afterward in a separate evaluation path; simulator IDs
never enter the tracker. Temporal outputs are image displacement/velocity in
pixels, not metric speed, lane assignment or risk. Event family stays null where
not inferable. No subject identity is borrowed from simulator metadata.

## Model and licensing

Verified source: [OpenCV Zoo model directory](https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox).
YOLOX-S `object_detection_yolox_2022nov.onnx`, 35,858,002 bytes, Apache-2.0 per
directory LICENSE. Download and license retained under `models/yolox_opencv_2022nov`.
Weights SHA256 `c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063`.
Preprocessing follows the published demo's raw RGB top-left 640x640 letterbox,
114 fill, no mean normalization; output grid/stride decode and class-aware NMS.
Confidence .35, NMS IoU .5; inference settings, source path and image hashes saved.
No torch/Ultralytics/scipy installed, no environment package changed. CPU execution
was used; GPU availability was not established. The model is not an ambulance
detector, and COCO benchmark results are not results on these recordings.

## Actual outputs

| RGB recording | Frames inferred | Retained road-user detections | Created track IDs | Candidate associations |
| --- | ---: | ---: | ---: | ---: |
| aad_lc2_concurrent_context_rgb_01 | 301 | 27 | 8 | 26 |
| overtake_clear_rgb_01 | 301 | 8 | 5 | 8 |
| shared_braking_stronger_rgb_01 | 181 | 0 | 0 | 0 |

Reports are `analysis_output/camera_{lanechange,overtake,braking}_01/report.json`.
These are **poor detection coverage**, not a successful event recognizer. Counts
are executed outcomes, not precision/recall. Empty detections were retained and
processed normally. At IoU>=.5, candidate-associated track changes were4/4/0;
these are diagnostic proxy counts, not certified ID switches or IDF1.
No AP/mAP, IDF1, HOTA or event F1 is reported: projected envelopes have not been
independently annotated as visible-object boxes, and camera-event labels are
not validated. No ground-truth actor ID is substituted for a missed detection.

Each vision directory contains detections/tracks/image hashes in
`camera_observations.jsonl`, separate `projection_candidates.jsonl`, evaluation-only
`associations.json`, sampled detection overlays, and `review.html`. This is a
read-only inspection page, not a completed annotator editing/sign-off workflow.
Candidates clip boxes to image bounds, reject behind/near-plane-straddling boxes
as unavailable and do not model occlusion or visual silhouettes. Near-plane edge
clipping still needs implementation; no candidate is called human-confirmed.

`camera_*_temporal_01/track_motion.json` contains pixel motion summaries; semantic
event-family predictions remain unavailable. Fitting a supervised event classifier
would currently be premature: sparse/fragmented tracks, no reviewed camera-event
labels, few configurations and a single simulator seed. No trained ML model or
camera event-recognition performance is claimed.

`analysis_output/camera_split_01.json` groups seven available trajectory hashes,
keeping physical repeats and camera variants in the same partition. The split
is a frozen data-readiness manifest, not an executed independent test evaluation.
Development/test assignment is deterministic hash modulo5. No labels were tuned
or model trained on this split; the current model confidence is a declared baseline.

## Braking experiment

Four new simulations executed: ordinary, stronger and negative controls headlessly
and stronger RGB. Each completed180 steps with181 samples; zero contact termination.
At decision80, change the leader's *desired* speed30 to10,2 or30 km/h. Actual state
is advanced through IDM/physics. No direct speed/position assignment, teleportation,
or disabled collision checks. All vehicles start at rest and accelerate first.

| Condition | Leader speed before change | Final leader speed | Final subject speed |
| --- | ---: | ---: | ---: |
| ordinary | 29.99993 | 10.00000 | 10.36793 |
| stronger | 29.99993 | 2.52990 | 1.46416 |
| negative | 29.99993 | 29.99993 | 29.99991 |

Headless and RGB stronger runs have identical normalized trajectory hash
`863fe7463a613a43754b954a2cf74b618c4d67732908ad1109d9e4d377b273d5`.
Read-only geometry/full PNG decode/state synchronization checks passed; reports
are in `analysis_output/shared_braking_validation_01/results.json`. Legacy
`verified:false` means no lane change; it is not a braking failure. General
family-specific phase/annotation integration is incomplete; step80 is a command,
not a claimed physical onset. No hard-braking/criticality label was assigned.

## Validation and remaining acceptance

97 tests passed, zero failures/skips: previous89 plus8 new tests for IoU,
continuation, missed detections/retirement, class mismatch, invalid boxes/times,
missing weights and positive-speed preflight. These validate baseline components,
not tracking accuracy in the simulator domain. Historical hash inventories checked
2,964 files with no changes; this is not an exhaustive fresh pre-edit snapshot.
See `analysis_output/aad_system_audit_01` for Git status, code/model provenance
and preservation scope. v0.3 remains unapproved and unimplemented.

Conservative acceptance accounting against the user's13 component rows: only
recording is substantially validated end-to-end at the requested scope (1/13,
7.7% fully accepted rows). Simulation, ground truth, overtaking, vision, tracking,
temporal, classifier readiness, evaluation, integration, inspection, reproducibility
and documentation all have partial implementations or missing acceptance evidence;
none is rounded up to complete. This is an acceptance checklist percentage, not
an estimate of source-code effort, and does not mean the substantial prior work
is absent. A 90% claim would be unsupported.

Outstanding engineering: shared actor/controller/phase interfaces; intentional
abort; crossing/VRU feasibility; complete braking annotations; source manifests
for all adapters; candidate overlays and near clipping; real visible-label review;
detector-domain investigation; stronger tracker and occlusion tests; independent
tracking/event evaluation; actual camera event reconstruction; robust grouped
evaluation and end-to-end automated report orchestration. No silent fallback
claims risk, lane change or overtaking from a fragmented image track.

External research requirements remain: supervisor event/PCDS protocol, independent
annotation/adjudication, justified conflict/risk assessment, real ambulance/camera
data, calibrated vehicle/warning behaviour, held-out seed/context evidence and
external validity. These are not engineering-completion substitutes.

## Commands (fresh output names required)

```powershell
cd C:\Users\aarya\repos\metadrive
.\.venv\Scripts\python.exe -m aad_system registry
.\.venv\Scripts\python.exe -m aad_system simulate --family braking --config aad_system/configs/braking_stronger.json --output test_output/my_braking_run --rgb
.\.venv\Scripts\python.exe -m aad_system validate --source test_output/my_braking_run --output analysis_output/my_braking_validation.json
.\.venv\Scripts\python.exe -m aad_system vision --source test_output/aad_lc2_concurrent_context_rgb_01 --output analysis_output/my_camera_run
.\.venv\Scripts\python.exe -m aad_system temporal --source analysis_output/my_camera_run --output analysis_output/my_camera_temporal
.\.venv\Scripts\python.exe -m aad_system review --source analysis_output/my_camera_run
.\.venv\Scripts\python.exe -m unittest aad_system.test_system experiments.aad_overtake_v01.test_events experiments.aad_lane_change_v02.test_pipeline experiments.aad_lane_change_v01.test_pipeline audits.test_geometry_boundary_v02 aad_analysis_v02.test_measurements aad_analysis_v02.test_pipeline experiments.test_geometry aad_analysis.test_pipeline
```

Open the review.html path printed by `review` in a browser. `extract` delegates
to immutable v0.2 and therefore requires a successful saved geometry report;
the new read-only validator does not manufacture that report inside an archive.
Vision-only input without a geometry sidecar is supported, but assumes10Hz
numbered PNGs; generalized timestamp input remains a gap. Detection and tracking
are independent classes but the current CLI executes them together. No simulation
rerun, detector dependency or GPU is required for an existing geometry extraction.
