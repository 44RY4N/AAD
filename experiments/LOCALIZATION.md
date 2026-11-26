# Lane-localization investigation and opt-in workaround

## Result

On Windows, Python 3.11.9, MetaDrive 0.4.3 and Panda3D 1.10.16, the default
short vertical Bullet ray can return no hit for an existing, geometrically
valid long lane localization prism. This is reproducible with a stationary
query in an isolated `BulletWorld`: neither vehicles nor navigation are needed.
The workaround replaces each straight-lane localization hull with a compound
of short boxes at runtime, using public Panda3D APIs. Simulator source and
dependencies are unchanged. It is opt-in, and the original 14-second settings
and saved outputs are preserved.

## Reproduce and run

Run from `C:\Users\aarya\repos\metadrive`, using fresh output paths:

```powershell
# Original extended experiment, expected failure at step 163:
.\.venv\Scripts\python.exe -m experiments.run --config experiments\lane_change_extended.json --output test_output\reproduce_extended

# Single vehicle, stock IDM, no traffic; expected failure at step 75:
.\.venv\Scripts\python.exe -m experiments.localization --output test_output\reproduce_straight

# Same straight road with runtime workaround, 300 steps:
.\.venv\Scripts\python.exe -m experiments.localization --output test_output\straight_fixed --box-lanes

# Original overtaking scenario extended to 300 steps:
.\.venv\Scripts\python.exe -m experiments.run --config experiments\lane_change_extended.json --output test_output\extended_fixed --box-lanes

# Original behavior and original 14-second configuration:
.\.venv\Scripts\python.exe -m experiments.run --output test_output\original_regression
```

Diagnostic failures produce a nonzero CLI exit code after saving evidence.
Do not use `--box-lanes` for the baseline reproduction. No reset, teleportation,
manual velocity changes, termination suppression, collision-check disabling,
or nearest-lane fallback is used to make the fixed run succeed.

## Implementation path inspected

- `metadrive/component/lane/straight_lane.py`: lane polygon samples both
  edges every metre. The road `('>>', '>>>')` extends x=10..500; its middle lane
  centre is y=3.5, with width 3.5 m.
- `metadrive/component/block/base_block.py`, `_construct_lane`: inserts
  polygon points at z=0 and z=-0.1 into `BulletConvexHullShape`; retains
  a lane-tagged static rigid body in the separate static physics world.
  A 490 m lane contributes 1,964 points to its hull.
- `metadrive/utils/coordinates_shift.py`: coordinates are passed through;
  no y inversion or z-from-vehicle interpretation causes this failure.
- `metadrive/utils/pg/utils.py`, `ray_localization`: queries
  `static_world.rayTestAll((x,y,+1), (x,y,-1))`. Sets `on_lane=True` only
  when a hit node has a lane type. Heading filtering happens afterwards and
  cannot turn that flag false.
- `metadrive/component/navigation_module/node_network_navigation.py`:
  `_get_current_lane` consumes this query; `_update_current_lane` copies the
  flag to the vehicle. Retaining or finding a closest lane after a miss does
  not repair `on_lane`. Therefore a plausible lane index may coexist with
  `on_lane=False`.
- `metadrive/envs/metadrive_env.py`, `_is_out_of_road`: treats
  `not vehicle.on_lane` as out of road, causing the observed termination.
- Also inspected lane masks/bodies, terrain clipping configuration,
  engine manager hooks, the existing runner, policies and recorded results.
  `cull_lanes_outside_map` defaults to false; the road lies inside the map.

## Evidence and causal isolation

1. The original scenario reproduced at step 163, x=156.5980835,
   y=7.0466933. Output: `test_output/localization_reproduce_01`.
2. A stock-IDM subject with lane changes disabled and no traffic reproduced at
   step 75, x=99.3990784, y=3.500000477. Fixed-source lateral error was
   -0.000000477 m. Both stored localization and an immediate repeated query
   were false; no vehicle/object/sidewalk/continuous-line collision was reported.
   Output: `test_output/rays_stock_baseline_01`.
3. At the failed minimal position the default ray has **zero hits**. A vertical
   ray from +10 to -10 hits the same lane surface at z≈0.07085. Thus the lane
   rigid body exists and is queryable. Vehicle height is not an input to the
   original ray, so vehicle suspension is not the cause.
4. Before any driving, integer-x scans at all three forward-lane centres yield
   27 short-ray misses in total. These are reproducible spatial misses, not
   transient object removal or vehicle interactions.
5. A separate `BulletWorld` with only one lane-shaped rigid body isolates the
   physics query. For 4,881 rays at x=11.0..499.0 in 0.1 m increments,
   y=3.5, the measured miss counts are:

| Shape | ±1 m ray misses | ±10 m ray misses |
|---|---:|---:|
| Original dense 490 m convex hull | 74 | 0 |
| Eight-corner equivalent hull | 75 | 0 |
| Eight-corner hull centred around its origin | 72 | 0 |
| Single 490 m box | 1,155 | 0 |
| Short overlapping boxes, at most 20 m each | 0 | 0 |

The hull has the correct geometric extent, but short-ray intersection is
numerically unreliable for this elongated thin primitive in the installed
Bullet/Panda3D build. The table rules out redundant polygon vertices and
coordinate centring as sufficient fixes. A single long box is also insufficient;
its earlier 300-step driving pass did not cover all problematic positions.

This establishes the failing layer and an independently reproducible cause:
the physics shape/ray combination returns false negatives, which localization
faithfully propagates. The precise internal Bullet C++ tolerance/iteration
branch was not instrumented, so no claim is made about a particular internal
algorithm defect or upstream fix. Longer rays are diagnostic evidence here;
the workaround retains the original ±1 m query and stock localization logic.

## Workaround details and scope

`lane_geometry.py:replace_straight_lane_hulls` runs immediately after reset,
before driving. It keeps each original static lane body, lane index/tag,
collision mask and static state. Only its localization shape is replaced.
For each straight lane, it subdivides its intended prism into boxes no longer
than 20 m, covering the same lateral bounds and z=-0.1..0. Internal segment
ends overlap by 1 cm on each side; outer lane ends are clamped to their original
bounds. This avoids exact-seam query misses without extending the road ends.
Original margin settings are retained. Hull and box margin implementations
need not yield identical near-boundary hits, so validation is of the intended
lane footprint, not bitwise equality of hull-surface intersections.

The bodies are detached and reattached while replacing shapes so Bullet updates
their collision representation. Dynamic-world chassis, wheels, ground,
sidewalks and lane-line bodies are untouched. Outside-road negative ray scans
still miss; termination and contact checks remain enabled.

The helper handles only untransformed `StraightLane` bodies with one original
hull; it rejects transformed bodies, clipped-lane configuration and unexpected
shape layouts rather than guessing. Curves are left unchanged. Apply once
after **each** reset if using it beyond the supplied one-reset runners.
`--box-lanes` means this segmented-box workaround, not a single long box.

## Acceptance results

- `rays_stock_segmented_01`: stock IDM, no traffic, completed **300 steps / 30 s**;
  all 301 samples localized. Geometry/scan output contains 18 replaced lane
  bodies (forward and reverse lanes in both straight blocks).
- Both the production-world scan (1,467 forward-centre rays) and isolated
  4,881-position scan have zero centre misses after the workaround.
  All 978 outside-road rays at y=-10 and y=30 miss as expected.
- `extended_segmented_01` and `extended_segmented_02`: completed **300 steps**,
  1,204 trajectory rows each, all four vehicles localized throughout, no
  vehicle/sidewalk collision flags. Subject travels 220.795 m net.
- Physical lane change still departs at step 56, settles from step 70 and
  confirms at step 80, displacement -3.705028 m at confirmation.
- Extended runs match exactly after generated IDs are removed. SHA256:
  `fb2414c3511fa617cd629136bdb38692adb93d539309d6bfd0b88df85cbd5a4c`.
- `regression_14s_01`, without the flag: original result matches exactly,
  including SHA256
  `a1c0605870fd4e61405117969c93596b45aa201cfd7f217fec23e9db42ed4ad5`.

All output paths above are under `test_output/`. Superseded diagnostic outputs
are retained as evidence, including the insufficient single-box experiments;
they are not the final workaround results.

## Outputs and remaining limits

`localization.py` writes `rays.csv`, `geometry.json`, `spatial_scan.json`,
`isolated_shape_probe.json`, and `summary.json`. Each row logs position,
fixed-lane coordinates, speed, stored and fresh localization flags, selected
lanes, all ray hit names/fractions/positions/normals at two ray lengths, and
collision/line flags. The failing terminal step is included.
Sampling is at every `env.step` boundary (0.1 s); the five internal 0.02 s
physics substeps are not individually sampled. The isolated scan has no
physics advancement or vehicle movement.

The main runner additionally writes `rays.csv` and adds workaround/localization
status to metadata; existing trajectory fields and verification are preserved.
Its failure path retains terminal-step rays and partial trajectories.

These tests validate this straight-road configuration and the installed Windows
build, not arbitrary curved/clipped maps, different platforms or all seeds.
Rendering was not retested for this change. The next task is validating camera
visibility and synchronized RGB capture over the now-extended scenario; expand
geometry/seed coverage before treating the workaround as a general simulator fix.
