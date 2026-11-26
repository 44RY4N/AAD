# Overtaking pilot: first master-task increment

2026-11-15. Implemented/executed feasibility work, not completion of the master
pipeline or a fully validated overtaking family. Existing lane-change runner,
recorders, geometry schema and v0.1/v0.2 extractors remain unchanged.

The new scheduled controller enters passing lane 0, observes actual full world-x
box-envelope clearance over the slower source-lane leader, and commands return
to lane 1 when rear-to-front clearance exceeds configured `return_gap_m=10`.
This is a controller setting, not a validated safety threshold. Actual return
requires 11 recorded samples within .35 m of source centre while fully ahead.
This definition is restricted to the tested straight road and does not certify
arbitrary full-footprint lane containment. No oncoming traffic, warning system,
ambulance calibration or intentional abort controller is implemented.

| Executed condition | Observed full pass | Return confirmation step |
| --- | --- | --- |
| clear | yes | 151 |
| interaction | yes | 151 |
| delayed | yes | 199 |
| negative control | no | null |
| slow blocker, target desired speed 1 km/h | no | null |

These names are design intents, not criticality findings. Interaction conflict
evidence has not yet been quantified over the entire overtake window. Risk is
unassessed. The zero-speed blocker failed at initialization/first action with
IDM division by zero; it was retained, not hidden or fixed in simulator source.
The separate slow-blocker alternative ran to 300 steps without completing a pass.

Nine simulations executed: clear headless feasibility probe, four matrix trials,
zero-speed blocked failure, slow-blocker alternative and two clear RGB runs.
Eight reached 300 steps, six demonstrate full pass/return under this definition;
the negative/slow-blocker controls do not. No contact termination occurred in
these new trials. Raw data are under `test_output/overtake_*`.

Both clear RGB captures contain 301 synchronized 640x360 frames and geometry.
The existing RGB validator verifies first lane-change phases and repeat
trajectories/decisions; its legacy phase labels do NOT describe the later return.
Use the separate geometry-derived `analysis_output/overtake_events_01` timelines
for full-event phases. Clear RGB/raw repeat trajectory hash excluding IDs is
`6bc8aef30ac2326a01b9663da5bca2e8664795eb110c103c84abede522942f36`.

`analysis_output/overtake_clear_01` contains both unchanged feature packages and
the original first-lane-change annotation. The separate
`analysis_output/overtake_events_01/overtaking_annotation.json` describes the
full pass/return using schema 0.1, event ID `overtake_clear_001`. Risk/response
intent remain unassessed. Geometry validation passed five successful headless
conditions and clear RGB; complete v0.2 recomputation passed. 89 tests passed,
including all previous 85 and four new event checks. These are limited synthetic
validation results, not external validity or comprehensive configuration tests.

Reproduce with unused paths:

```powershell
cd C:\Users\aarya\repos\metadrive
.\.venv\Scripts\python.exe -m experiments.aad_overtake_v01.run --config experiments/aad_overtake_v01/clear.json --output test_output/my_overtake_rgb --box-lanes --record-rgb --record-geometry
.\.venv\Scripts\python.exe -m experiments.aad_overtake_v01.run --config experiments/aad_overtake_v01/clear.json --output test_output/my_overtake_repeat --box-lanes --record-rgb --record-geometry
.\.venv\Scripts\python.exe -m experiments.aad_lane_change_v01.analyze --source test_output/my_overtake_rgb --reference test_output/my_overtake_repeat --output analysis_output/my_overtake
.\.venv\Scripts\python.exe -m experiments.aad_overtake_v01.events --source test_output/my_overtake_rgb --output analysis_output/my_overtake/full_pass_event.json
.\.venv\Scripts\python.exe -m unittest experiments.aad_overtake_v01.test_events experiments.aad_lane_change_v02.test_pipeline experiments.aad_lane_change_v01.test_pipeline audits.test_geometry_boundary_v02 aad_analysis_v02.test_measurements aad_analysis_v02.test_pipeline experiments.test_geometry aad_analysis.test_pipeline
```

Next work: validate pass/return onset and source hashes across all conditions,
preserve full-return phase labels in an explicit shared recorder interface,
quantify leader/passing-traffic metrics for the whole overtake, add abort/delay
decisions with validated geometry, systematic parameter sweeps and representative
RGB for non-clear conditions. The copied runner still uses the inherited
class-level settings convention; this is a pilot, not the requested final shared
framework. A zero desired-speed configuration remains unsupported and should
receive explicit preflight rejection in the next controller revision.
