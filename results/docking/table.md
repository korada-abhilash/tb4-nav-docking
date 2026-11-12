| Dock pose source | Start poses | Action reports success | Depth error, cm: median (p90) | Sideways error, cm: median (p90) | Heading error, deg: median (p90) | Within ±3 cm / ±2 cm | Also within ±3° |
|---|---|---|---|---|---|---|---|
| Map pose only (AMCL) | seed 0 | 19/20 | 26.8 (65.9) | 2.6 (5.5) | 2.0 (2.6) | 0/20 | 0/20 |
| AprilTag, calibrated at 0.7 m | seed 0 | 20/20 | 2.8 (3.1) | 5.6 (6.0) | 5.6 (7.7) | 0/20 | 0/20 |
| Map pose only (AMCL) | seed 1 | 20/20 | 25.7 (48.1) | 2.0 (7.0) | 1.6 (3.8) | 1/20 | 1/20 |
| AprilTag, + position bias correction (fitted on seed 0) | seed 1 | 20/20 | 1.1 (1.3) | 0.3 (0.6) | 7.6 (10.8) | 20/20 | 0/20 |
