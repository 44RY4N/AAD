# Geometry-aware offline analysis v0.2

Separate from the unchanged `aad_analysis/` baseline. Requires validated runtime
geometry schema 0.2; historical missing geometry is never backfilled. Uses the
existing Shapely installation; extraction does not launch MetaDrive or advance
physics. Optional Bullet validation uses isolated Panda3D worlds only.

See [the measurement specification](../research/aad_v0.1/GEOMETRY_AWARE_FEATURES_v0.2.md)
for exact commands, metric equations, validation results, provenance and limits.

```powershell
.\.venv\Scripts\python.exe -m aad_analysis_v02.run --source test_output\geometry_validation_01 --output analysis_output\my_new_v02 --horizon-s 5
.\.venv\Scripts\python.exe -m aad_analysis_v02.validate --output analysis_output\my_new_v02
```

Outputs describe projected outer-envelope geometry, not exact physical clearance
or real-world collision prediction. Non-valid metrics are null with reasons;
risk remains unassessed. Use a new destination for every extraction.
