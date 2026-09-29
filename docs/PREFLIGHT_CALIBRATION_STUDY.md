# Preflight barometer calibration — 2026-09-29

This experiment compares the existing first-flight-reading calibration with
preflight averages installed in the fixed-lag filter's initial snapshot.
It follows the [navigation-update diagnosis](NAVIGATION_UPDATE_DIAGNOSIS.md).
Sensor specifications, flight-noise sequences, controller settings, plant,
scenario, and estimator covariance are held fixed. The study tests calibration
policy; it does not retune NMPC or INDI.

## Implementation and preserved behavior

`control/arena_sensors.py` now accepts the optional sensor-profile field
`estimator.preflight_baro_rng`, either `shared` or `independent`.

- `shared` is the existing behavior and remains the default. Preflight draws
  consume the in-flight barometer RNG, preserving earlier nominal/stress setups.
- `independent` uses a sixth child of the sensor seed. The original five
  children retain the exact IMU, GNSS, barometer, magnetometer, and RPM streams.
  Increasing the preflight count therefore does not shift in-flight noise.
  Different averaging counts share a prefix of the preflight draw sequence.

The existing `preflight_baro_samples` chooses the average size. The existing
`ArenaSensorFeedback` calls `NavigationFilter.set_initial_baro_bias` before
flight packets are processed, storing the calibration in the replay baseline.
That filter code is unchanged. No new truth signal is fed to the controller;
the existing calibration helper assumes an exact known reference altitude,
supplied from the simulation's initial trim altitude.

This is an idealized stationary calibration at that reference altitude. It
does not simulate a takeoff, a preflight aircraft trajectory, pressure settling,
dropout, or arrival latency during calibration. Acquisition budgets quoted
below are `N / rate`, not measured wall time or a simulated launch sequence.

Detailed navigation logging remains enabled in every experimental profile.
The reference arena configuration, bundled sensor profiles, truth-feedback
path, and default calibration behavior are unchanged.

## Matched flight experiment

`control/preflight_calibration_experiment.py` creates these five profiles:

| Variant | Calibration before flight | Flight barometer-bias learning from GNSS |
|---|---|---|
| `baseline` | None; first arriving flight reading initializes bias | Enabled |
| `preflight1` | 1 independent reading | Enabled |
| `preflight25` | Average of 25 independent readings | Enabled |
| `preflight100` | Average of 100 independent readings | Enabled |
| `preflight100_frozen` | Same 100-reading average | Disabled |

Each uses seeds 1 and 2 with V13 and F13: 20 full 12-second trials of
`gust_lateral_p10_VL`. V13 remains A1/S1/50 Hz and F13 A0/S0/50 Hz; both retain
startup guard off. Generated barometer noise remains 0.25 m standard deviation,
GNSS position/velocity noise remains 0.8 m / 0.15 m/s, and sensor transport
delays are unchanged. Estimator measurement covariance stays fixed, including
the existing 0.6 m barometer tuning.

The one-reading preflight variant separates installing an independent
calibration before flight from averaging more samples. The frozen variant
separates initial calibration from the later GNSS-based bias learner. It is an
isolation control, not a proposed universal policy for real pressure drift.

See the [generated comparison](../results/preflight_followup_v5/preflight_report.md)
and [startup figure](../results/preflight_followup_v5/preflight_startup.png) for
the final outcomes. Startup metrics cover 0–0.5 s; pass/fail criteria cover the
complete 12-second trial. All seeds remain in the denominators.

## Observed outcomes

All 20 trials completed without optimizer failures or timeouts and passed
tracking. None passed the complete acceptance check: each left the propulsion
map's assumed RPM range. Reconstruction identifies only below-minimum-RPM
samples, with no high-RPM, excessive advance-ratio, or reverse-flow flags.

| Controller | Calibration policy | Mean altitude RMSE [m] | Mean peak startup height error [m] | Mean domain-outside fraction [%] |
|---|---|---:|---:|---:|
| V13 | Baseline | 0.0871 | 0.2123 | 0.8500 |
| V13 | 100 samples, learning enabled | 0.0967 | 0.0924 | 0.8250 |
| V13 | 100 samples, bias frozen | 0.0464 | 0.0924 | 0.7500 |
| F13 | Baseline | 0.0884 | 0.2122 | 0.5083 |
| F13 | 100 samples, learning enabled | 0.0999 | 0.0924 | 0.2500 |
| F13 | 100 samples, bias frozen | 0.0477 | 0.0924 | 0.2500 |

The initial calibration improvement is real, but its closed-loop benefit is
mixed. V13 seed 1's peak startup height error falls from 0.3214 m to 0.0846 m
with 100 samples. Seed 2 falls only from 0.1031 m to 0.1003 m, while its
domain-outside fraction rises from 0.3667% to 0.8667%. Mean largest startup
thrust steps do not improve with 100 samples: V13 rises from 3.7232 N to
3.9026 N and F13 from 2.9744 N to 3.1020 N.

Over the full trial, 100 samples with learning still enabled makes mean
altitude RMSE 11.1% worse for V13 and 13.0% worse for F13. Freezing that same
calibration reduces altitude RMSE by 46.7% and 46.0%, respectively, relative
to baseline, but does not eliminate domain violations. This separates the
initial-reference issue from the continuing bias learner and noisy feedback.
It does not justify freezing pressure compensation on a real vehicle.

The next fusion change should address uncertainty and measurement acceptance
in GNSS-based barometer-bias learning, then evaluate it with pressure drift and
reference-altitude error. A covariance-aware treatment of the common bias
error is preferable to treating all corrected barometer residuals as
independent. The existing controller settings should remain fixed during that
comparison; this study does not select a new calibration default.

## What averaging changes mathematically

For barometer model `y_k = z_k + b + eta_k`, define the preflight estimate as
`b_hat_0 = mean(y_preflight) - z_reference`. With exact reference altitude,
constant bias, and independent white noise of variance `sigma_b²`,

```
e_b = b_hat_0 - b = mean(eta_preflight)
Var(e_b) = sigma_b² / N
y_k - b_hat_0 = z_k + eta_k - e_b
Var(eta_k - e_b) = sigma_b² (1 + 1/N)
Cov(eta_i - e_b, eta_j - e_b) = sigma_b²/N, i != j
```

Thus averaging reduces the common calibration error. It does not reduce each
in-flight barometer reading's white noise, nor does it remove GNSS errors.
The remaining common offset makes successive corrected barometer measurements
correlated. This calibration uncertainty is outside the current 15-state
covariance; covariance tuning remains unchanged here to isolate the effect.

The [calibration-only audit](../results/preflight_followup_v5/calibration_sampling_audit.json)
uses 4,096 independent seeds, a constant +1 m true bias, and exact reference
height. It is separate from the two-seed controller experiment:

| N | Nominal budget at 50 Hz | Predicted calibration sigma | Measured calibration RMSE |
|---:|---:|---:|---:|
| 1 | 0.02 s | 0.2500 m | 0.2509 m |
| 25 | 0.50 s | 0.0500 m | 0.0503 m |
| 100 | 2.00 s | 0.0250 m | 0.0252 m |

The 25-sample calibration happens to be closer to zero than the 100-sample
calibration for flight seed 1 (-0.00426 m versus -0.02357 m). Averaging improves
the distribution, not every particular finite realization monotonically.
These values are model-generated, not measurements from a physical barometer.

## Replay and continuing GNSS updates are different mechanisms

In the baseline, delayed GNSS at 120 ms inserts its time-zero measurement
before the first barometer measurement. Replaying that barometer recomputes
its initial bias against a changed historical height.

With preflight calibration, the filter restores an initial snapshot that
already contains `b_hat_0` and a calibration timestamp. Replaying the first
barometer therefore applies its residual without reinitializing its bias.
GNSS can still change height and velocity. A constant bias value at that arrival
does not mean the navigation state or commanded thrust is constant.

Later GNSS packets, when enabled, use the existing learner:

```
observed_bias = latest_baro_height - GNSS_height
alpha = 1 - exp(-elapsed / baro_bias_tau_s)
b_hat <- b_hat + alpha (observed_bias - b_hat)
```

This estimate contains barometer and GNSS noise and any height difference
between their sample times. The existing age check permits near-simultaneous
samples; it does not compensate that height difference. The learner also runs
before the GNSS navigation-update NIS gate. This study leaves that behavior
unchanged; rejection-safe bias learning remains a separate follow-up concern.

For intuition only, a constant-gain learner driven by independent white
`observed_bias` noise has steady variance `alpha/(2-alpha) * Var(observed_bias)`.
At the nominal 10 Hz update rate and 1 s time constant, ignoring motion and
cross-correlation, the 0.25 m barometer and 0.8 m GNSS height noises give about
0.19 m standard deviation in that scalar learned bias. This simplified
calculation is not a measured filter-consistency result; it explains why an
accurate initial average can later be dominated by continuing bias updates.

The saved report distinguishes initial bias, bias after the first barometer,
the full bias change during the first GNSS arrival, sample-time GNSS height
injection, and the total current-state jump after replay. The latter includes
propagation and other sensor updates. Its relation to the next thrust command
is timing evidence, not a decomposition of per-sensor causality.

## Verification and limits

125 focused tests pass. The new tests cover legacy preflight values, identical
flight packets across averaging counts, installation before flight, survival
of delayed replay, later bias learning versus frozen calibration, and startup
report metrics. Four repeated baseline trajectories match the earlier saved
results bit for bit.

The [saved verifier](../results/preflight_followup_v5/verify.py) checks frozen
source/configuration hashes, profile changes, controller settings, complete
trials, trace timing, arrival-state equality, replay calibration behavior, and
reconstructed model-domain verdicts. It also subtracts each trial's truth state
from its raw barometer/GNSS packets to verify matching generated noise across
the different closed-loop trajectories. It resolves local result paths after
copying to Desktop.

Verification covered 38,380 unique measurement packets and 24,000 arrival-state
records. The maximum reconstructed barometer/GNSS noise difference between
matched variants is 1.42e-14, consistent with floating-point subtraction.
All 16 preflight trials preserve their bias at the first GNSS arrival, and all
four frozen-bias trials retain their installed value throughout the mission.

An assumed low-RPM propulsion-map boundary is a model-validity limit, not a
physical instability threshold. Two seeds in one gust case do not establish
hardware specifications. Exact initial navigation and reference altitude,
constant pressure bias, independent white calibration noise, and absent
calibration uncertainty in the filter covariance remain material assumptions.
No experimental calibration policy is promoted to the default by this study.

## Reproduce

Run from the repository using its Python environment. Output directories must
be new, so choose different names if the saved results already exist.

```bash
python -m control.preflight_calibration_experiment \
  --output results/preflight_v13_v5 --controllers V13 --sensor-seeds 1 2
python -m control.preflight_calibration_experiment \
  --output results/preflight_f13_v5 --controllers F13 --sensor-seeds 1 2
python -m control.preflight_calibration_report \
  results/preflight_v13_v5/experiment.json \
  results/preflight_f13_v5/experiment.json \
  --output results/preflight_followup_v5
python results/preflight_followup_v5/calibration_sampling_audit.py
python results/preflight_followup_v5/verify.py
```
