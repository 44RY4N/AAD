# Offline geometry/provenance and feature pipeline v0.1

This package processes the existing RGB experiment outputs without importing or
launching MetaDrive. It uses Python's standard library only. It never modifies
the source run or calls the recording validator (which writes into source outputs).
The original experiments/configurations and environment are unchanged.

From `C:\Users\aarya\repos\metadrive`, choose a new analysis directory:

```powershell
.\.venv\Scripts\python.exe -m aad_analysis.run --source test_output\rgb_extended_02 --output analysis_output\my_rgb02_analysis
.\.venv\Scripts\python.exe -m aad_analysis.run --source test_output\rgb_extended_03 --output analysis_output\my_rgb03_analysis
.\.venv\Scripts\python.exe -m unittest aad_analysis.test_pipeline -v
```

Existing destination directories are rejected. Output must be separate from the
input tree (neither inside it nor an ancestor). Failed input validation occurs
before output creation. Extraction may leave partial analysis artifacts if a disk
write fails; only an analysis containing a successful `validation.json` is complete.
Do not write analyses into an existing recording directory.

## Artifacts

```text
analysis_output/my_rgb02_analysis/
  manifest.json
  actor_features.jsonl
  pair_features.jsonl
  validation.json
```

`manifest.json` records all input file paths, sizes and SHA256 values, including
every PNG. The run ID is the content digest of that inventory. The scenario ID
hashes the recorded settings; it is a grouping aid, not a guarantee of complete
scene identity. Hashes for the current extractor and relevant simulator/experiment
source are stored separately, explicitly labelled as current-checkout provenance,
not proof of the source revision used during recording. Recorded Python/MetaDrive
versions, timestep/settings, actor IDs/roles and event landmarks are carried through.
There are no nondeterministic creation timestamps in the artifact contents.

Geometry separates three evidence levels:

- **Recorded:** fixed source-lane endpoints, lane identifiers, width setting,
  actor reference positions/headings and camera metadata references.
- **Reconstructed:** straight parallel lane strips from those endpoints/width and
  observed lane indices. These are labelled assumptions, not a runtime map export.
  Every input s/l coordinate is checked against the reconstructed fixed reference
  to a 0.0001 m serialization/numerical tolerance; this is not a risk threshold.
- **Source-inferred:** candidate default actor dimensions from AST inspection of
  the installed `DefaultVehicle` class (4.515 × 1.852 × 1.19 m). These are not
  resolved historical per-instance dimensions. Their presence does not enable
  footprint calculations. Runtime footprints, origin offsets, full camera
  intrinsics and conflict zones remain explicitly missing/null.

The geometry manifest does **not** pretend to retroactively add runtime ground
truth. Actual actor world z and full camera matrices were not captured. The camera
is the existing elevated chase view, not a driver-mounted ambulance camera.

## Feature records

Actor JSONL contains one record per CSV actor sample, including run/actor ID,
role, step/time, source CSV line, assigned lane, subject-only retrospective phase,
nearest same-assigned-lane front/rear actor identities, metrics and unassessed risk.
Pair JSONL contains one directed subject-to-other record per synchronized actor
pair (three pairs at each of 301 steps). No actor pairs across different steps
or generated identities are silently joined.

Every metric includes `value`, `unit`, `validity`, `reason`, `method`, and
`support_steps`. Non-valid values are JSON null. NaN/Infinity are rejected.
Statuses are `valid`, `unavailable`, `invalid`, `undefined`, `not_applicable`.
“Valid” means valid under the declared measurement method, not safe/critical.

Implemented estimates:

- World vx/vy and ax/ay from reference positions.
- Fixed-road longitudinal velocity, acceleration and jerk from source s.
- Signed lateral velocity, acceleration and jerk from source l.
- Speed-based tangential acceleration and jerk from scalar speed converted /3.6.
- Nearest front/rear **reference-point** separations in the assigned lane.
- Subject/other reference distance, signed longitudinal separation, and relative
  world vx/vy and longitudinal velocity (other minus subject).
- Explicitly named `point_following_ttc_proxy` and `gross_reference_headway`.
  The TTC proxy is projected reference-point catch-up time, not a footprint TTC
  or validated collision prediction, especially during lateral movement.

Let h be recorded dt, z_i a position coordinate and u_i speed in m/s:

```text
D z_i  = (z_(i+1) - z_(i-1)) / (2h)
D2 z_i = (z_(i+1) - 2z_i + z_(i-1)) / h²
D3 z_i = (z_(i+2) - 2z_(i+1) + 2z_(i-1) - z_(i-2)) / (2h³)
a_speed = D u; j_speed = D2 u
```

`a_longitudinal`/`j_longitudinal` are D2/D3 of fixed-road s, not body-frame or
accelerometer measurements. Lane normal is `(tangent_y,-tangent_x)`; in these
runs lateral coordinate is `3.5-y`. Negative lateral motion goes toward lane 0.
Heading is not assumed equal to velocity direction. Derivatives use future samples
and are offline-only, unfiltered estimates. Boundaries/missing stencil samples
produce unavailable values; failed localization, road changes or nonuniform
stencil timing invalidate the estimate. Endpoint zeros are not fabricated.
No derivatives bridge an absent actor sample. They do not detect brake application
or evasive intent and are not the full published Bagdadi method.

For an eligible other actor ahead in the same assigned lane and a broadly common
forward heading (`cos(heading difference)>0`): reference separation ds is divided
by positive relative longitudinal closing velocity for the TTC proxy, or positive
subject longitudinal velocity for gross headway. No closing/stationary follower
is undefined with null; actors behind/in other lanes are not applicable. The 1e-9
m/s denominator tolerance is numerical, not a safety threshold. Lane assignment
is not footprint occupancy; the proxy does not account for lateral collision
geometry. All spatial/relative features in v0.1 conservatively require valid
localization and the fixed source road; applicability outside it is not asserted.

Deliberately unavailable:

- Bumper gap and minimum footprint clearance.
- Footprint-based two-dimensional TTC.
- PET without shared conflict-zone/footprint occupancy definitions.
- Footprint lane occupancy and conflict-zone occupancy.

Each is emitted with a null value and an explicit missing-prerequisite reason.
No source-inferred rectangle is silently substituted. No normal/abnormal,
conflict/criticality, completed-overtaking or ambulance-intent labels are assigned.
All risk fields remain `unassessed`.

## Input validation and reproducibility

The pipeline validates scalar finiteness/booleans, lane tuples (safe literal
parsing, never eval), duplicate identities/steps, global step coverage, timestamps,
all frame-vs-CSV state fields, unique files/roles, PNG RGB headers/resolution and
frame-directory containment. It hashes input files before and after extraction,
rejecting concurrent input changes. PNG bodies are hashed but not fully decoded;
the existing saved recording validation reports full decode checks separately.
This pipeline does not claim a new independent image/occlusion validation.

`validation.json` reports record/frame counts and per-metric validity counts.
`feature_fingerprint_without_generated_ids` canonicalizes the actor/pair feature
records by role and step, removing run/actor IDs while preserving nearest-actor
roles. It is for repeat comparison, not independent scientific validation.
Floating point results are compared exactly on this environment; cross-platform
bit equality is not guaranteed. Manifest hashes of feature files support artifact
integrity checks. Keep source directories immutable after extraction.

Validation performed on both existing RGB runs:

- 301 synchronized frames; 1,204 actor records and 903 subject–traffic pairs each.
- Exact matching normalized feature fingerprint:
  `332f07f6668542fdb82d2fcfe235c1a12c59d6a0333a6a8fb6361ea263ae8b1c`.
- Longitudinal acceleration: 1,196 valid estimates and eight unsupported endpoint
  samples (two per actor). No endpoint estimates were replaced by zero.
- Point TTC proxy: 240 valid model-only values, 59 no-closing undefined values,
  two unsupported values and 602 non-applicable pairs. These counts do not measure
  conflicts or critical events.
- Physical event metadata retained at steps 56/70/80, with no reinterpretation.
- Tests cover analytic polynomial derivatives/units/sign, missing samples,
  localization failures, road changes, nonuniform timing, zero denominators,
  proxy-vs-footprint separation, duplicate rows, malformed boolean/nonfinite data,
  broken synchronization, overwrite/path protection and real repeated-run extraction.

Eight tests passed. Initial drafts of outputs (`*_v01`) were retained; final
validated outputs using the finished package are `analysis_output/rgb_extended_02_v02`
and `analysis_output/rgb_extended_03_v02`. Their core features have the same fingerprint.
All 720 pre-existing files under `experiments/` and `test_output/` retain the prior
aggregate content hash `02d46c14460997b8ba25cbd1b144fd64f1e285bdf29095e4f035b3b7c35b5caa`.

## Next dependency

For new captures, separately instrument resolved class/dimensions, reference-origin
offsets, world footprint corners, vector velocities, map polygons and calibrated
camera parameters. Validate those fields against runtime collision geometry before
enabling physical clearance/2D-TTC/occupancy features. That future runtime export is
not part of this offline package; historical missingness remains explicit.
