# PushCube Dataset Quality Report

## Dataset Overview

- Input dataset: `data/simulation/isaac_rl/datasets/pushcube_manual_demos.hdf5`
- Number of demos: 10
- Success count: 10
- Episode length min / mean / max: 952 / 1444.90 / 2254
- Cube final-distance mean: 0.0912 m
- Cube-motion mean: 0.2096 m

## Demo Table

| demo | len | success | cube_motion | final_dist | ee_motion | action_sat | non_finite | status |
| --- | ---: | :---: | ---: | ---: | ---: | ---: | ---: | --- |
| demo_000000 | 2254 | yes | 0.2309 | 0.0794 | 0.7198 | 0.0000 | 0 | pass |
| demo_000001 | 1376 | yes | 0.2394 | 0.0207 | 0.6576 | 0.0000 | 0 | pass |
| demo_000002 | 953 | yes | 0.1757 | 0.1049 | 0.5024 | 0.0000 | 0 | pass |
| demo_000003 | 1461 | yes | 0.2407 | 0.0988 | 0.6397 | 0.0000 | 0 | pass |
| demo_000004 | 952 | yes | 0.2482 | 0.0390 | 0.3853 | 0.0000 | 0 | pass |
| demo_000005 | 1691 | yes | 0.1781 | 0.1386 | 0.6249 | 0.0000 | 0 | large_final_distance |
| demo_000006 | 1177 | yes | 0.1622 | 0.1465 | 1.0374 | 0.0025 | 0 | large_final_distance |
| demo_000007 | 1325 | yes | 0.2033 | 0.0731 | 0.6329 | 0.0000 | 0 | pass |
| demo_000008 | 2073 | yes | 0.2251 | 0.0766 | 1.3380 | 0.0002 | 0 | pass |
| demo_000009 | 1187 | yes | 0.1921 | 0.1344 | 0.9580 | 0.0000 | 0 | large_final_distance |

## Warning List

- `large_final_distance`: 3 demos

## Recommended Filtering Thresholds

- Minimum episode length: `100` steps
- Minimum cube motion: `0.030` m
- Maximum final cube-to-target distance: `0.120` m
- Maximum action saturation ratio: `0.350`
- Minimum end-effector motion: `0.030` m

## Suggested Number of Usable Demos

- Suggested usable demos with the above thresholds: **7**
- Demos needing review or removal: **3**

## Notes for Thesis Writing

- The raw manual dataset contains 10 real demonstrations; any later augmentation should be reported separately.
- With the proposed thresholds, 7 / 10 demos are immediately usable for BC training.
- The thesis should distinguish task success from trajectory quality: a successful demo can still be too short, overly saturated, or nearly stationary.
- 2D trajectory figures support quantitative claims, while 3D Isaac renders should be reserved for qualitative examples and failure-case discussion.
