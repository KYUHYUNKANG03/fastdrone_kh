# Sensor campaign status — 2026-09-29

## Current evidence

The current results use the `gust_lateral_p10_VL` case and GSLQR. Tracking and
propulsion-model validity are separate acceptance gates. Sensor seeds use
independent deterministic streams for IMU, GNSS, barometer, magnetometer, and
RPM. A dropped packet still advances its sensor's stochastic process, so paired
outage-duration comparisons receive identical measurements outside the outage.

The two-dimensional navigation campaign interpolates IMU and GNSS blocks from
nominal (level 0) to stress (level 1). Barometer, magnetometer, RPM, and rotor
observer settings remain nominal. Each cell below contains two sensor seeds.

| IMU level | GNSS level | Tracking passes | Model-domain passes | Mean velocity RMSE [m/s] | Mean altitude RMSE [m] |
|---:|---:|---:|---:|---:|---:|
| 0.0 | 0.0 | 2/2 | 2/2 | 0.250 | 0.100 |
| 0.5 | 0.0 | 2/2 | 1/2 | 0.279 | 0.114 |
| 1.0 | 0.0 | 2/2 | 1/2 | 0.325 | 0.132 |
| 0.0 | 0.5 | 0/2 | 0/2 | 0.680 | 0.698 |
| 0.5 | 0.5 | 0/2 | 0/2 | 0.760 | 0.681 |
| 1.0 | 0.5 | 0/2 | 0/2 | 0.860 | 0.667 |
| 0.0 | 1.0 | 0/2 | 0/2 | 1.313 | 1.516 |
| 0.5 | 1.0 | 0/2 | 0/2 | 1.514 | 1.445 |
| 1.0 | 1.0 | 0/2 | 0/2 | 1.644 | 1.443 |

The refined transition campaign gives:

| IMU level | GNSS level | Tracking passes | Model-domain passes | Mean velocity RMSE [m/s] | Mean altitude RMSE [m] |
|---:|---:|---:|---:|---:|---:|
| 0.0 | 0.125 | 5/5 | 5/5 | 0.338 | 0.229 |
| 0.0 | 0.250 | 3/5 | 4/5 | 0.436 | 0.351 |
| 0.0 | 0.375 | 1/5 | 3/5 | 0.550 | 0.498 |
| 1.0 | 0.125 | 4/5 | 3/5 | 0.420 | 0.231 |
| 1.0 | 0.250 | 1/5 | 1/5 | 0.547 | 0.349 |
| 1.0 | 0.375 | 0/5 | 0/5 | 0.688 | 0.488 |

GNSS level 0.125 corresponds to 9.375 Hz, 1.075 m position sigma,
0.231 m/s velocity sigma, 149 ms latency, and 1.25% random dropout. Level 0.25
corresponds to 8.75 Hz, 1.35 m, 0.313 m/s, 178 ms, and 2.5% dropout. Level
0.375 corresponds to 8.125 Hz, 1.625 m, 0.394 m/s, 206 ms, and 3.75% dropout.

For this case, nominal GNSS preserves tracking even with the fully stressed IMU
block. With nominal IMU, GNSS level 0.125 passes all five seeds, level 0.25 is a
mixed transition region, and level 0.375 has no full passes. With fully stressed
IMU, level 0.125 is already mixed and levels 0.25 and 0.375 have no full passes.
These are screening results, not certified sensor limits; the transition cells
still need more seeds and additional flight cases.

## GNSS outage duration

The outage campaign keeps all nominal sensor specifications and suppresses GNSS
packets from 5 s onward for the stated duration. Five paired seeds were run.

| Outage duration [s] | Tracking passes | Model-domain passes | Full passes | Mean velocity RMSE [m/s] | Mean altitude RMSE [m] |
|---:|---:|---:|---:|---:|---:|
| 0.0 | 5/5 | 5/5 | 5/5 | 0.321 | 0.199 |
| 2.0 | 5/5 | 4/5 | 4/5 | 0.336 | 0.267 |
| 3.0 | 5/5 | 5/5 | 5/5 | 0.344 | 0.273 |
| 3.5 | 4/5 | 5/5 | 4/5 | 0.357 | 0.247 |

The binary full-pass result is not strictly monotonic because the closed loop is
nonlinear and the time at which GNSS corrections resume matters. A noisy update
can create a brief domain excursion even when a longer outage does not. The
useful conclusion is that tracking survived every tested outage through 3 s,
while the first tracking failure appeared at 3.5 s. Continuous degradation of
GNSS quality was more damaging than a bounded outage with otherwise nominal
measurements.

## Implemented mitigations and remaining work

The stress profile includes a 100-sample stationary barometer calibration. The
calibrated offset is stored in the fixed-lag replay baseline so delayed updates
cannot erase it. RPM telemetry and its rotor observer were previously isolated
from the navigation failures; the current grid therefore keeps that block
nominal and targets the IMU/GNSS interaction.

Next evidence should add at least 20 seeds at GNSS levels 0.125, 0.25, and
0.375; repeat the boundary on acceleration, deceleration, climb, and yaw cases;
then screen V13/F13 at the surviving points. M17 should use a shorter screening
case or a larger explicit wall-time budget because it is too expensive for a
broad campaign with the current solver settings.

Results are saved in `results/navigation_grid_gslqr`,
`results/navigation_grid_gslqr_refined`,
`results/navigation_grid_gslqr_refined_seeds_3_5`, and
`results/gnss_outage_gslqr_corrected`.

## Superseded results

Campaigns generated before the per-sensor random-stream change are retained as
development artifacts only. Their qualitative observations motivated the
current grid, but their numerical thresholds must not be mixed with the current
tables. In the old generator, changing one sensor's packet schedule shifted the
shared random stream used by every other sensor.
