# Executed sweep results - generation 2

Date: 2026-11-15. All criticality assessments unassessed. Minima/TTC are inside each actual verified lane-change window. Contact rows have no completed-event minimum; logs/raw prefixes are retained. Requested subject speeds 20/40 in stage-1 018/019 remained active target 30; refinement 012/013 correctly applied 20/40. See README for design and validity limits.

| Trial | Gap m | Target desired km/h | Command step | Subject requested / active km/h | Subject spawn s m | IDM headway s | Departure / confirmation | Event min m | Min valid model TTC s | Model interaction steps | Outcome |
| --- | ---: | ---: | ---: | --- | ---: | --- | --- | ---: | ---: | --- | --- |
| lc2_trial_000 | -28 | 20 | 35 | 30 / 30 | 40 | stock | 39 / 62 | 20.259454 | null | none | completed verified merge |
| lc2_trial_001 | -28 | 20 | 56 | 30 / 30 | 40 | stock | 60 / 85 | 17.849042 | null | none | completed verified merge |
| lc2_trial_002 | -28 | 30 | 35 | 30 / 30 | 40 | stock | 39 / 62 | 14.797423 | null | none | completed verified merge |
| lc2_trial_003 | -28 | 30 | 56 | 30 / 30 | 40 | stock | 60 / 85 | 6.146382 | 2.027981 | 60,61,62,63 | completed verified merge |
| lc2_trial_004 | -28 | 40 | 35 | 30 / 30 | 40 | stock | 39 / 62 | 11.626505 | null | none | completed verified merge |
| lc2_trial_005 | -28 | 40 | 56 | 30 / 30 | 40 | stock | not completed | null | null | unassessed | vehicle-contact termination at 67 |
| lc2_trial_006 | -12 | 20 | 35 | 30 / 30 | 40 | stock | 39 / 62 | 4.362201 | null | none | completed verified merge |
| lc2_trial_007 | -12 | 20 | 56 | 30 / 30 | 40 | stock | 60 / 85 | 1.951002 | 1.450070 | 60,61,62 | completed verified merge |
| lc2_trial_008 | -12 | 30 | 35 | 30 / 30 | 40 | stock | not completed | null | null | unassessed | vehicle-contact termination at 46 |
| lc2_trial_009 | -12 | 30 | 56 | 30 / 30 | 40 | stock | 80 / 157 | 15.157944 | null | none | completed verified merge |
| lc2_trial_010 | -12 | 40 | 35 | 30 / 30 | 40 | stock | not completed | null | null | unassessed | vehicle-contact termination at 44 |
| lc2_trial_011 | -12 | 40 | 56 | 30 / 30 | 40 | stock | 67 / 100 | 12.954284 | null | none | completed verified merge |
| lc2_trial_012 | 12 | 20 | 35 | 30 / 30 | 40 | stock | 39 / 71 | 10.944368 | null | none | completed verified merge |
| lc2_trial_013 | 12 | 20 | 56 | 30 / 30 | 40 | stock | 60 / 91 | 12.709745 | null | none | completed verified merge |
| lc2_trial_014 | 12 | 30 | 35 | 30 / 30 | 40 | stock | 39 / 65 | 14.212962 | null | none | completed verified merge |
| lc2_trial_015 | 12 | 30 | 56 | 30 / 30 | 40 | stock | 60 / 86 | 21.941085 | null | none | completed verified merge |
| lc2_trial_016 | 12 | 40 | 35 | 30 / 30 | 40 | stock | 39 / 64 | 15.301363 | null | none | completed verified merge |
| lc2_trial_017 | 12 | 40 | 56 | 30 / 30 | 40 | stock | 60 / 85 | 28.462140 | null | none | completed verified merge |
| lc2_trial_018 | -28 | 30 | 35 | 20 / 30 | 40 | stock | 39 / 62 | 14.797423 | null | none | completed verified merge |
| lc2_trial_019 | -28 | 30 | 35 | 40 / 30 | 40 | stock | 39 / 62 | 14.797423 | null | none | completed verified merge |
| lc2_trial_020 | -28 | 30 | 35 | 30 / 30 | 80 | stock | 39 / 62 | 14.797408 | null | none | completed verified merge |
| lc2_trial_021 | -28 | 30 | 75 | 30 / 30 | 40 | stock | not completed | null | null | unassessed | vehicle-contact termination at 85 |
| lc2_trial_022 | -28 | 30 | 35 | 30 / 30 | 40 | 0.75 | 39 / 62 | 16.542013 | null | none | completed verified merge |
| lc2_trial_023 | -28 | 30 | 35 | 30 / 30 | 40 | 2.0 | 39 / 63 | 13.548354 | null | none | completed verified merge |
| lc2_trial_024 | 100 | 30 | 56 | 30 / 30 | 40 | stock | 60 / 85 | 109.773090 | null | none | completed verified merge |
| lc2_trial_025 | -16 | 40 | 56 | 30 / 30 | 40 | stock | 75 / 130 | 17.673280 | null | none | completed verified merge |
| lc2_refine_000 | -24 | 25 | 45 | 30 / 30 | 40 | stock | 49 / 73 | 11.171663 | null | none | completed verified merge |
| lc2_refine_001 | -24 | 25 | 56 | 30 / 30 | 40 | stock | 60 / 85 | 7.717011 | null | none | completed verified merge |
| lc2_refine_002 | -24 | 30 | 45 | 30 / 30 | 40 | stock | 49 / 73 | 7.291394 | 2.503554 | 49 | completed verified merge |
| lc2_refine_003 | -24 | 30 | 56 | 30 / 30 | 40 | stock | 60 / 85 | 2.151888 | 0.988560 | 60,61,62,63,64,65,66,67 | completed verified merge |
| lc2_refine_004 | -20 | 25 | 45 | 30 / 30 | 40 | stock | 49 / 73 | 7.178764 | null | none | completed verified merge |
| lc2_refine_005 | -20 | 25 | 56 | 30 / 30 | 40 | stock | 60 / 85 | 3.730840 | 1.747699 | 60,61,62,63 | completed verified merge |
| lc2_refine_006 | -20 | 30 | 45 | 30 / 30 | 40 | stock | 49 / 73 | 3.297429 | 1.391549 | 49,50,51,52,53,54 | completed verified merge |
| lc2_refine_007 | -20 | 30 | 56 | 30 / 30 | 40 | stock | not completed | null | null | unassessed | vehicle-contact termination at 68 |
| lc2_refine_008 | -16 | 25 | 45 | 30 / 30 | 40 | stock | 49 / 73 | 3.203567 | 1.698162 | 49,50,51 | completed verified merge |
| lc2_refine_009 | -16 | 25 | 56 | 30 / 30 | 40 | stock | not completed | null | null | unassessed | vehicle-contact termination at 70 |
| lc2_refine_010 | -16 | 30 | 45 | 30 / 30 | 40 | stock | not completed | null | null | unassessed | vehicle-contact termination at 57 |
| lc2_refine_011 | -16 | 30 | 56 | 30 / 30 | 40 | stock | not completed | null | null | unassessed | vehicle-contact termination at 63 |
| lc2_refine_012 | -28 | 30 | 35 | 20 / 20 | 40 | stock | 39 / 66 | 13.350850 | null | none | completed verified merge |
| lc2_refine_013 | -28 | 30 | 35 | 40 / 40 | 40 | stock | 39 / 62 | 14.838310 | null | none | completed verified merge |
