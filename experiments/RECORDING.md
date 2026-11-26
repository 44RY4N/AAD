# Synchronized RGB recording

Optional recording is integrated into the existing runner. The sensor reuses
`test_rgb_camera.py`: `RGBCamera(640,360)`, `image_observation=True`,
`image_source="rgb_camera"` and `perceive(to_float=False)`. Camera recording
uses an offscreen graphics buffer by default; add `--render` for a visible
window. It needs graphics support even without a visible window.

## Commands

From PowerShell, using a new output path for each run:

```powershell
cd C:\Users\aarya\repos\metadrive
.\.venv\Scripts\python.exe -m experiments.run --config experiments\lane_change_extended.json --output test_output\my_rgb_run --box-lanes --record-rgb
.\.venv\Scripts\python.exe -m experiments.validate_recording --output test_output\my_rgb_run --reference test_output\extended_segmented_01
```

To repeat and compare recorded trajectories:

```powershell
.\.venv\Scripts\python.exe -m experiments.run --config experiments\lane_change_extended.json --output test_output\my_rgb_repeat --box-lanes --record-rgb
.\.venv\Scripts\python.exe -m experiments.validate_recording --output test_output\my_rgb_repeat --reference test_output\my_rgb_run
```

Omit `--record-rgb` for the original non-rendering headless experiment. Existing
`--box-lanes` behavior is unchanged. Neither configuration JSON was modified.
`--render --record-rgb` is supported but was not tested in this task; the tested
capture path is offscreen.

## Output layout

```text
my_rgb_run/
  metadata.json                 # existing results/configuration + rgb_recording
  trajectory.csv                # original state schema and checksum
  decisions.json                # original IDM decisions
  rays.csv                      # existing localization diagnostics
  camera/
    camera.json                 # sensor/mount/format and capture-check details
    frames.jsonl                # exactly one JSON entry per saved frame
    validation.json             # written by validate_recording
    frames/
      000000.png
      000001.png
      ...
      000300.png
```

The PNG sequence contains **301 frames**: initial state at step 0, plus one
frame after each of 300 decisions. Timestamps span 0.0 through 30.0 simulation
seconds at 0.1 s intervals. These are simulation timestamps, not capture wall
clock times. PNG was chosen to avoid a video-encoding dependency and retain
individually inspectable frames. No encoded video is generated.

Each `frames.jsonl` entry contains:

- `step`, `time_s`, relative PNG `file`.
- `subject`: the exact subject trajectory row, including vehicle ID, position,
  heading, speed, lane ID, lane-local coordinates and collision/localization flags.
- `traffic`: all three surrounding vehicles' exact trajectory rows, identified
  by both generated ID and stable scenario role.
- `lane_change_state`: retrospective scenario phase from physical verification.
- Camera world position/orientation and projected vehicle-centre coordinates
  with `in_frustum` flags. A projection is not an occlusion or bounding-box label.

Event states are `lane_following` before departure, `lane_changing` from
departure to target-centre tolerance, `target_tolerance_pending_confirmation`
until confirmation, then `confirmed_target_lane`. For this experiment the
boundaries are steps 56, 70, and 80. These are scenario phase labels based on
the completed physical trajectory, not abnormal-driving or criticality labels.
On an interrupted experiment that reaches normal Python cleanup without a
verified result, saved records are labelled `unverified`.

## Synchronization and camera view

The runner samples trajectory rows and captures the image in the same loop
iteration before the next `env.step`. The recorder renders two graphics frames
to flush the current scene transforms, waits for rendering, and reads the
sensor. It never calls `env.step`, `taskMgr.step`, a policy, or a physics warm-up
step. Thus initialization includes no extra simulation steps. It checks every
vehicle's position, heading, speed and lane before and after capture and fails
if those values change. Full trajectory/IDM comparisons separately validate
the entire recorded run against the existing reference.

The CPU sensor provides BGR bytes. The recorder reverses channels and checks
the result against Panda3D's explicit `getRamImageAs("RGB")` readback at every
capture before writing an RGB PNG. This avoids guessing channel order from
the image appearance. PNG dimensions, mode, complete decoding and non-blank
content are checked again by the saved-artifact validator.

The sensor settings are reused, with a documented observational mount change:
an elevated chase view at vehicle-local `(0,-22,16)` looking toward `(0,15,0)`.
The original bonnet-view mount would not show the entire subject. Only the
camera is positioned; vehicles are not teleported. This view shows the subject,
road geometry and leader during the lane change.

## Validation results

Tested with the existing Python 3.11.9, MetaDrive 0.4.3 and Panda3D 1.10.16
environment, with the 300-step configuration and `--box-lanes`.

| Check | Result |
|---|---|
| First recorded run | `test_output/rgb_extended_02`, 300 steps completed |
| Repeated recorded run | `test_output/rgb_extended_03`, 300 steps completed |
| Frame count per run | 301 PNG files |
| Frame metadata count per run | 301 entries, steps 0..300, no missing/duplicate steps |
| Images | All fully decoded, RGB, 640×360, non-blank |
| State synchronization | All four frame state rows equal their exact CSV rows |
| Physical lane-change verification | Departure 56, tolerance 70, confirmation 80 |
| Displacement at confirmation | -3.7050280570983887 m |
| Recorded vs original reference | Exact trajectory and normalized-ID IDM-decision match |
| Recorded repeat | Exact trajectory and normalized-ID IDM-decision match |
| Optional non-camera path | `rgb_optional_headless_01` matches; no camera folder |
| Subject projected centre | Inside image frustum in all 301 frames |
| Leader projected centre | Inside image frustum in 104 frames, including every step 56..80 |

The unchanged trajectory SHA256 excluding generated vehicle IDs is:

```text
fb2414c3511fa617cd629136bdb38692adb93d539309d6bfd0b88df85cbd5a4c
```

Visual inspection of saved frames at steps 0, 56, 80, and 300 confirmed the
subject appears in the chase view, with the leader and more distant traffic
visible during the event. Frustum checks alone do not establish visibility;
this inspection complements them. At the final frame the overtaken leader is
behind the camera while the distant forward traffic remains visible.

Validation reports are saved at each successful run's `camera/validation.json`.
The first was compared against `extended_segmented_01`; the repeat was compared
against `rgb_extended_02`. The validator checks every state field, every image
and frame entry, the event results, and the normalized IDM decisions; failures
raise an error and return a nonzero exit code.

## Files inspected and changed

Inspected `test_rgb_camera.py`, `experiments/run.py`, scenario policies and
annotation logic; simulator `rgb_camera.py`, `base_camera.py`, `image_buffer.py`,
`image_obs.py`, rendering/physics hooks and sensor tracking defaults.

Modified `experiments/run.py` to add the optional recorder and RGB flag.
Added `experiments/recording.py`, `experiments/validate_recording.py`, and this
document. Added a documentation link to `experiments/README.md`. The camera
test, diagnostics, original configurations, simulator source, environment and
all existing output directories remain intact.

## Limits

- The first sandboxed attempt (`rgb_extended_01`) failed during asset loading
  before recording began. Running with approved graphics/asset access succeeded;
  no dependency changes were needed. That failed directory was retained.
- The elevated camera is a research observation view, not an ambulance-mounted
  driver-view camera. Additional views require separate visibility validation.
- The leader leaves the forward view after the subject passes it; its state
  remains logged. Distant background vehicles occupy few pixels at 640×360.
- Frames alone occupy about 85 MiB per run. Recording is slower than the
  simulation-only path; real-time throughput is not claimed.
- State trajectories and IDM decisions are reproducible. Bit-identical RGB
  pixels across different graphics drivers/platforms are not guaranteed.
- Sampling is at decision boundaries (10 Hz), not every internal physics substep.
  Frame phase labels describe this single controlled lane change, not a general
  multi-event annotation system. No detection/tracking/classification or video
  encoder was added.

The next useful validation is a second camera viewpoint with explicit visibility
coverage requirements, before expanding the scenario family or generating a
larger dataset.
