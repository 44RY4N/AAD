# Controlled lateral lane change — MetaDrive 0.4.3

Optional synchronized RGB recording is documented in
[RECORDING.md](RECORDING.md), including commands and validated frame/state checks.

The longer-run localization investigation is now documented in
[LOCALIZATION.md](LOCALIZATION.md), including an opt-in workaround validated
for 300 steps. The historical 14-second results and limitations below are
preserved as the original experiment record.

This is the foundational experiment for the ambulance research project. It
produces a contextual IDM overtake lane change, not a critical-event label.
No simulator source, existing diagnostic script, camera setup, virtual
environment, or dependency was changed.

## Run from PowerShell

```powershell
cd C:\Users\aarya\repos\metadrive
.\.venv\Scripts\python.exe -m experiments.run --output test_output\my_lane_change
.\.venv\Scripts\python.exe -m experiments.run --output test_output\my_lane_change_render --render
```

Choose a new output directory each time: existing results are never overwritten.
Use `--config path\to\settings.json` for a different configuration.
The existing environment is Python 3.11.9 and MetaDrive 0.4.3, imported from this
repository (editable installation). No installation commands are necessary.

## Files and responsibilities

- `run.py`: central runner, synchronized state sampling, output, failure checks.
- `scenarios.py`: controlled traffic manager and observed IDM subject policy.
- `annotation.py`: physical lane-change verification.
- `lane_change.json`: scenario parameters and seed.
- `__init__.py`: package entry.

Each successful output contains `trajectory.csv`, `decisions.json`, and
`metadata.json`. Trajectories include initial state and every 0.1-second decision
boundary for all four vehicles: IDs, roles, positions, heading, speed, lane,
fixed-source longitudinal/lateral coordinates, target lateral coordinates,
localization/contact flags. Decisions describe the action applied over the
interval ending at their step; they include the observed leader ID/distance and
selected steering lane. Metadata includes configuration, software versions,
geometry, verification result, and trajectory checksum excluding random IDs.
On an unexpected simulation failure the partial trajectory is retained and the
runner raises an error; successful metadata is not written for that failure.

## Inspection findings

Inspected the root traffic, camera, IDM inspection and lane-change diagnostics,
especially `test_rgb_camera.py`, `test_traffic_motion.py`,
`detect_lane_changes.py`, `trace_lane_change_decisions.py`,
`inspect_idm_after_motion.py`, and `inspect_surroundings.py`; also `setup.py`,
`metadrive/version.py`, `envs/base_env.py`, `envs/metadrive_env.py`,
`manager/traffic_manager.py`, `manager/base_manager.py`, `policy/idm_policy.py`,
`component/map/pg_map.py`, `component/map/base_map.py`,
`component/pgblock/first_block.py`, `component/vehicle/base_vehicle.py`,
`component/navigation_module/node_network_navigation.py`, `utils/pg/utils.py`,
and engine spawning/manager hooks. The simulator source paths here are relative
to `metadrive/` unless already prefixed.

The working camera script uses `RGBCamera(640, 360)`, image observations,
`image_source="rgb_camera"`, and `camera.perceive(to_float=False)`.
It was preserved, not rerun or integrated into this experiment.

Default traffic mode is `trigger`. It creates vehicles throughout later blocks
but only activates their policies when the agent reaches a trigger road.
Measured seed 0, density 0.2: 25 queued, zero active, unchanged after 100 steps
with the stationary ego. This explains the original lack of movement without
requiring a rendering explanation.

`basic` spawns immediately, only on `respawn_lanes`. The original map exposes
three 40 m lanes. With a 10 m spawn grid and density 0.2, the count is
`3 * ceil(0.2 * floor(40 / 10)) = 3`. This was measured directly.
Use `traffic_manager.traffic_vehicles` for active managed traffic;
`vehicles[1:]` assumes an ordering that the implementation does not promise and
can include inactive queued vehicles.

IDM overtaking requires a detected front vehicle, subject and leader speeds
more than 3 km/h away from NORMAL_SPEED, timer greater than 50, an allowed
adjacent lane, and a speed advantage greater than 10 km/h. An empty adjacent
lane is assigned a speed of 100 km/h if front/back distances exceed 15 m.
Thus identical 30 km/h free-flow traffic is a poor trigger. The policy uses
lidar surrounding objects followed by `FrontBackObjects`; decision logs confirm
the controlled leader is visible (initial distance 24 m).

`routing_target_lane` is not a stable lane-change command: `move_to_next_road`
can reset it to the physically occupied lane. The policy's lane-change result
selects the steering target, and `steering_control(lane)` turns that target into
a physical steering action. This experiment uses the original selection method
and a public custom policy/manager extension. `ObservedIDM.act` follows the
0.4.3 control sequence but deliberately surfaces errors that the original method
would catch broadly. Traffic vehicles use stock IDM with lane changes disabled.
No vehicle is teleported during the run, and no target-lane decision is forced.

## Configuration and measured validation

Seed 0; three 3.5 m lanes; first block length 500 m (controlled road from
x=10 to x=500, length 490 m); subject in lane 1 at longitudinal 40 m; leader
24 m ahead with desired speed 10 km/h; subject desired speed 30 km/h.
Two additional moving vehicles are 100 m ahead in lane 2 and 180 m ahead in
lane 0. Random PG traffic density is zero because the custom manager owns
the three explicit vehicles. Physics timestep 0.02 s, five repeats per decision.
Duration 140 decisions / 14 seconds.

Validated outputs:

- `test_output/validated_headless_01` and `_02`: exact identical trajectories
  after removing vehicle IDs, SHA256
  `a1c0605870fd4e61405117969c93596b45aa201cfd7f217fec23e9db42ed4ad5`.
- `test_output/controlled_render_01`: onscreen execution passed; position,
  speed and event results matched headless. This was before adding extra
  localization columns, so its CSV schema and checksum differ from final output.
- Departure beyond 0.35 m: step 56 (5.6 s). Target-centre tolerance reached at
  step 70 (7.0 s), sustained through step 80 (8.0 s).
- Displacement at confirmation: -3.705 m relative to fixed source geometry.
  Negative means toward lane 0. Source and target are on the same directed road.
- Subject travelled 87.503 m net; leader 37.484 m; other vehicles 104.909 m and
  103.602 m. Four vehicles, 141 samples each, 564 state rows.
- Equal-speed leader control, 120 steps: no lane change; correctly returned
  `verified=false` and a nonzero exit code. Output:
  `test_output/equal_speed_control_120`.
- Verification checks on recorded trajectories accepted the physical crossing,
  rejected a road-transition case, and rejected changing lane IDs without
  physical displacement.

The detector requires source-centre initial position, same-road samples,
no subject vehicle collision, continuous sampled motion (no >2 m jump per
0.1 s), and at least 11 consecutive samples within 0.35 m of the target centre
and at least 3.15 m away from the source centre. Lane-index changes alone are
insufficient. These thresholds are specific to this low-speed experiment.

## Limitations and next task

The 14-second experiment is validated; arbitrary seeds, geometries, speeds and
longer episodes are not. An extended 180-step test terminated at step 163
(x=156.598, y=7.047): lane localization returned `on_lane=False`, despite
geometric proximity to the lane centre; no vehicle, sidewalk or continuous-line
collision was reported. The equal-speed 140-step attempt similarly terminated
at step 130, which is why its valid negative control is limited to 120 steps.
This has been reproduced and narrowed to the ray-based localization path;
the underlying physics-ray failure remains unresolved. Earlier failed-run
directories are retained for transparency (`controlled_01`, `controlled_02`,
`extended_diagnostic`, `equal_speed_control`).

Next investigate the static-world lane collision geometry and ray localization
failure with a minimal straight-road run, then extend duration and seeds and
add blocking-adjacent-lane controls. After that, integrate the preserved RGB
camera and verify event visibility and frame/state synchronization. Camera/video
recording, ambulance appearance, criticality classification, full overtaking
completion, dataset replay and ML are deliberately deferred.
