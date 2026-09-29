# Navigation updates and startup thrust — 2026-09-29

This study adds optional navigation-update traces and isolates barometer/GNSS
noise, latency, and constant height bias. Controller settings, plant dynamics,
estimator covariance, and the truth-feedback path remain unchanged. The purpose
is to locate the source of the startup corrections before changing fusion tuning.

## What the code records

Set `estimator.trace_updates: true` in a sensor profile. The default is false.
`control/arena_estimator.py` records each GNSS, barometer, and magnetometer packet
once, at its first processing, with:

- sample, arrival, and processing times; sequence and arrival-batch identifiers;
- raw and bias-corrected measurements, predicted measurement, innovation,
  innovation covariance, measurement covariance, and NIS;
- accepted/rejected/stale status, state before/after, covariance diagonals,
  15-component error-state injection, and barometer bias before/after;
- a separate arrival-batch record containing current state before/after replay
  and the number of older measurement updates replayed.

The error-state order in this implementation is
`[delta_p(3), delta_v(3), delta_theta(3), delta_ba(3), delta_bg(3)]`.
The nominal logged state is `[p(3), v(3), q_xyzw(4), omega(3)]`.
Barometer bias is a separate scalar heuristic, outside the 15-state covariance.
RPM continues through its separate rotor observer.

`control/validation_suite.py` writes `<case>_<controller>.navigation.json`
alongside the trajectory and solver logs. It does not embed the trace as an
object array in the NPZ. Detailed logs are intended for bounded diagnostic runs:
one 12-second baseline trace occupies about 8 MiB.

Two distinct differences must be kept separate:

1. An individual update applies `delta_x = K r` at the historical sample time.
   Its first-processing trace is immutable. A later replay may compute a
   different correction for that old packet; it does not create another packet
   record or rewrite the original one.
2. The arrival-batch state difference includes propagation, new updates, and
   replayed old updates. It is not an additive contribution from one sensor.

The postprocessor links each arrival batch forward to the first control tick
and next actual NMPC solver call, and records thrust before/at that solve. It
also lists all sensor kinds processed since the preceding solve. A terminal
arrival with no later control/solve has a null link. Timing associations alone
do not prove which sensor caused the entire thrust change.

## Controlled comparison

The campaign uses `gust_lateral_p10_VL`, the original 12-second trajectory,
seeds 1 and 2, and the matched nominal `nav_imu0_gnss0` sensor profile. V13 uses
its original A1/S1/50 Hz controller settings; F13 uses A0/S0/50 Hz. The optional
rotor startup guard remains off in this comparison.

V13 runs eight variants, two seeds each: baseline, zero barometer noise, zero
barometer latency, +1 m constant barometer bias, zero GNSS position/velocity
noise, zero GNSS latency, +1 m constant GNSS height bias, and zero noise on both
barometer and GNSS. F13 runs the four noise-isolation variants with both seeds.
Per-sensor random streams preserve paired draws across variants.

Estimator covariance stays fixed: GNSS position/velocity standard deviations
are 0.8 m / 0.15 m/s, and barometer standard deviation is 0.6 m. The generated
nominal barometer noise standard deviation is 0.25 m. Setting generated noise
to zero therefore does not set the filter covariance to zero. These are
diagnostic controls, not proposed hardware specifications.

See the [generated 24-trial comparison](../results/navigation_update_followup_v4/navigation_report.md)
for outcomes and the [startup figure](../results/navigation_update_followup_v4/navigation_startup.png).

All 24 trials completed 12 seconds with zero optimizer failures or timeouts.
22 passed tracking, and four passed both tracking and model-domain checks.
Those four are precisely the runs with both barometer and GNSS noise removed
(two controllers, two seeds). The other 20 have low-RPM excursions below the
assumed 10,000 RPM map boundary. Removing either noise source alone, or removing
either latency alone in V13, did not eliminate domain violations in this set.

| Controller | Measurement noise | Mean altitude RMSE [m] | Mean peak startup height-estimation error [m] | Full passes |
|---|---|---:|---:|---:|
| V13 | Nominal | 0.087 | 0.212 | 0/2 |
| V13 | Barometer and GNSS noise zero | 0.025 | 0.002 | 2/2 |
| F13 | Nominal | 0.088 | 0.212 | 0/2 |
| F13 | Barometer and GNSS noise zero | 0.031 | 0.002 | 2/2 |

IMU, magnetometer, and RPM errors remain nominal in every row. Two-seed means
are descriptive, not confidence bounds. In V13 seed 1, the startup peak height
estimation error drops from 0.3214 m to 0.0018 m and minimum startup rotor speed
rises from 6,878 RPM to 10,274 RPM when both navigation noise sources are removed.

Adding +1 m of constant barometer bias produces effectively the same V13
trajectory as baseline: startup calibration removes this constant offset.
It does not remove the retained random error in the first sample. In contrast,
both +1 m GNSS-height-bias trials fail the tracking checks, with altitude RMSE
of 0.900 m and 0.807 m. At the end of the first trial, estimated altitude is
19.976 m while true altitude is 19.114 m; the second ends at 20.035 m estimated
and 18.964 m true. The controller is regulating an inaccurate height reference.
These two sensor biases cannot be treated as interchangeable error magnitudes.

## The first barometer sample becomes a persistent reference error

For barometer height `y_b,k = p_z,k + b_b + eta_b,k`, startup currently sets
`b_hat_b,0 = y_b,0 - p_hat_z,0`. With exact initial height, the corrected later
measurement becomes `y_b,k - b_hat_b,0 = p_z,k + eta_b,k - eta_b,0` for a
constant true bias. The first noise draw is therefore retained as a common
offset. Under independent equal-variance samples, this component has variance
`2 sigma_b^2` for k > 0 and is correlated between subsequent measurements.
This algebra is a startup simplification; later GNSS bias learning and motion
add further coupling.

For V13 seed 1:

| Processing time | Observation and estimator effect | NMPC thrust at that solve |
|---|---|---|
| 20 ms | Barometer sample at 0 ms reads 19.77747 m for true initial height 20 m. Bias becomes -0.22253 m; innovation and height injection are zero. | See the saved trajectory. |
| 40 ms | Barometer sample at 20 ms reads 20.23957 m. Height innovation is +0.46216 m and sample-time height injection is +0.13436 m. | 9.99737 N before, 5.21302 N at the solve. |
| 120 ms | First GNSS packet sampled at 0 ms gives +0.10551 m height injection. Replaying the initial barometer changes its bias reference to -0.32804 m. A newly arriving barometer update adds its own +0.05715 m sample-time injection. | 6.61097 N before, 3.58569 N at the solve. |

The net arrival-height change at 120 ms is +0.14305 m, including propagation,
magnetometer updates, and replay of 16 older measurements. It must not be
reported as the GNSS injection alone. The 40 ms barometer NIS is 0.42082 and
the first GNSS NIS is 3.00999; both pass the existing gates.

The update equations used by the implementation are
`r = y - h(x_hat)`, `S = H P H^T + R`, `K = P H^T S^-1`,
`delta_x = K r`, and `NIS = r^T S^-1 r`. For the barometer,
`y = y_b - b_hat_b` and `h = p_hat_z`. Sensor error therefore enters the
feedback state through the covariance-weighted correction, then enters NMPC
through its initial state. INDI does not remove a navigation-reference error
merely by being robust to plant disturbances.

## Why delayed GNSS changes the initial barometer reference

At 120 ms the fixed-lag filter restores the initial state to insert the GNSS
sample at time zero. At equal sample times, that GNSS packet precedes the first
barometer packet in sequence order. GNSS corrects the historical height, then
the initial barometer calibration is recomputed against that corrected height.
This changes its reference even though no new initial barometer reading exists.

The [standalone replay example](../results/navigation_update_followup_v4/replay_calibration_probe.py)
reproduces this without a controller or moving plant. With initial vertical
variance 0.25 m² and GNSS variance 0.64 m²,
`K_z = 0.25 / (0.25 + 0.64) = 0.2808989`. A +0.375619 m GNSS innovation injects
+0.105511 m. Replaying the initial barometer shifts the bias from -0.222529 m
to -0.328040 m. The [saved output](../results/navigation_update_followup_v4/replay_calibration_probe.json)
records the full update and arrival traces.

This is a diagnostic of existing behavior, not a change to that behavior.

## Interpretation and the next experiment

The experiments begin from a known trim navigation state and cover two seeds
in one lateral-gust case. They cannot establish a sensor acceptance boundary.
NIS is saved for diagnosis; no statistical consistency claim follows from
these runs. The assumed propulsion-map RPM range is a model-validity check,
not a declaration that a physical vehicle becomes unstable below that RPM.

The next estimator experiment should compare the existing optional preflight
barometer averaging, installed in the replay baseline, against single-sample
initialization. Keep sensor noise, controller settings, and seeds matched,
and evaluate startup height jumps, thrust changes, tracking, and model domain
together. Calibration uncertainty and correlation need explicit treatment;
simply reducing measurement covariance does not remove a retained reference
error. GNSS bias learning should also be audited for its timing and gating.
Only after this comparison should a selected policy be tested on more seeds,
vertical gusts, initialization errors, and realistic sensor specification sweeps.

No estimator covariance, initialization policy, filter cutoff, or controller
gain is promoted or retuned in this study.

## Reproduction and verification

Use fresh output directories when rerunning the campaign. Commands and the log
switch are in [the sensor guide](SENSOR_FUSION.md#navigation-update-isolation).
`results/navigation_update_followup_v4/verify.py` checks frozen source hashes,
fixed controller/profile settings, baseline trajectory hashes, packet timing,
once-only update records, innovations/NIS, state injections, arrival-state
agreement with the saved feedback trajectory, and propulsion-domain verdicts.
It resolves traces inside either the staging or Desktop repository.

The focused sensor/trace and report/controller/validation checks passed 109
tests. Four repeated sensor baselines match the earlier trajectories bit for
bit. The truth-feedback code path and bundled configurations remain unchanged.
