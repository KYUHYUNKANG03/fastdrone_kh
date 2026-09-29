# Arena sensor-fusion experiment

The v6 integration connects the sensor profile to the normal tuning and
distributed experiment runners through an inline `sensor_feedback` arena
configuration. See [the team handoff](SENSOR_TEAM_HANDOFF.md) for the current
`tune-final-7` workflow and [the recorded development checks](../results/joint_fusion_followup_v6/report.md).
The original truth configurations and legacy 15D estimator remain available.
An optional `joint_baro` estimator adds barometer bias to the covariance (16D
error state, unchanged 17-value controller interface). This candidate has not
been selected as a final high-speed sensor configuration.

The arena now has two explicit feedback modes:

* `truth` is the unchanged reference path. The controller receives the plant's
  17-state truth (with the existing configured observation delay behavior).
* `sensors` inserts a measurement and estimator pipeline between the plant and
  the controller. The plant still advances from truth, so performance changes
  are attributable to feedback quality rather than a second vehicle model.

Run a deterministic comparison on one arena scenario with:

```bash
python -m control.validation_suite --config configs/arena.json --smoke \
  --only-cases ref_accel_0_VH_rho1 --only-controllers GSLQR \
  --feedback sensors --sensor-profile configs/sensors/nominal.json \
  --sensor-seed 1
```

Profiles use schema `sensor/1`. Each sensor has a rate, noise, bias, dropout
and transport latency. IMU noise densities are in units per sqrt-Hz; the
generator converts them to a discrete sample standard deviation. GNSS packets
contain position and velocity together. Barometer and magnetometer are
optional. RPM telemetry is independent from the navigation state and feeds the
rotor observer; selecting `rotor_observer.source = predictor` deliberately
removes telemetry while retaining a first-order command model.

The `rpm` block is named for ESC telemetry, but values, bias, sigma, and the
legacy `initial_rpm` field use the plant rotor state's **rad/s**, not revolutions
per minute. Convert a mechanical RPM specification by multiplying by
`2*pi/60` before assigning it. Electrical RPM additionally needs the correct
motor pole-pair conversion before this mechanical conversion.

Each sensor owns a deterministic random-number stream derived from the campaign
seed. Packet loss still advances that sensor's noise and bias process. This
keeps unrelated sensor noise streams identical when one sensor profile changes and preserves
the same post-outage noise realization when outage duration changes.
Measured values still change when the true trajectories differ.

The discrete IMU noise convention is `sigma = density/sqrt(T_sensor)`, with
`T_sensor = 1/rate_hz`. Bias random walk increments scale with the square root
of elapsed IMU sample time, including lost packets. Sensor samples occur on
plant ticks; use a plant step fine enough for the fastest configured sensor.
This model does not implement an independent analog bandwidth or hardware
anti-aliasing filter.

The navigation filter uses the nominal state

\[
  x_n=[p,v,q,b_a,b_g],
\]

and a 15-state error vector

\[
  \delta x=[\delta p,\delta v,\delta\theta,\delta b_a,\delta b_g].
\]

For an IMU sample, the specific force is

\[
 f_b=R(q)^T(a_W-[0,0,-g]^T), \qquad
 \omega_m=\omega+b_g+\eta_g.
\]

The filter propagates position and velocity with `R(q)(f_b-b_a)+g_W`,
integrates attitude with a right-multiplicative quaternion increment, and uses
GNSS, barometer, and magnetometer packets as measurement updates. Packet
`sample_time` and `arrival_time` are retained. A delayed update rewinds the
bounded event log to its sample time, applies the correction, and replays later
events. Packets older than the fixed lag are rejected and counted in
`estimator_diagnostics`, allowing latency and dropout sweeps to be compared
without hiding transport effects.

The state and covariance are propagated to every accepted measurement's sample
time, including measurements between IMU samples, using the last available IMU
input. Navigation feedback currently holds the most recent event state between
events; it does not extrapolate that state to the controller's current time.

Rotor telemetry has its own arrival queue. It is unavailable before arrival,
and the first arrived packet initializes the rotor estimate. Subsequent packets
use an exponential low-pass filter based on elapsed sample time. Older samples
cannot overwrite newer telemetry. Predictor mode ignores telemetry completely
and propagates the command applied during the preceding simulation interval.

The estimator can also perform a preflight barometer calibration. The configured
number of stationary samples is averaged against known launch altitude, and the
resulting offset is stored in the fixed-lag replay baseline so delayed-event
rewinds cannot erase it. The stress profile uses 100 samples and freezes the
offset during the short mission because its degraded GNSS is not a reliable
source for continuous barometer-bias learning.

The saved trial `.npz` contains the truth trajectory (`xs`), the controller's
estimated trajectory (`xs_est`) for sensor runs, and the sensor profile name.
The JSON manifest records the complete profile, feedback mode, and seed. This
supports a threshold study: increase noise, bias, latency, dropout, or lower
rates until a controller first violates the arena acceptance criteria, while
keeping the same scenario and plant seed.

## Sensor campaigns

For one-factor sweeps, use the campaign runner. Each point runs in a fresh
process and writes its generated profile, arena output, console log, and a
combined `campaign.json` summary:

```bash
python -m control.sensor_campaign \
  --sweep gnss_latency \
  --values 0 0.05 0.12 0.25 0.5 \
  --only-cases ref_accel_0_VH_rho1 \
  --only-controllers V13 M17 F13 \
  --timeout-s 360
```

Available sweep dimensions include `gnss_latency`, `gnss_position_noise`,
`gnss_outage_duration`, `imu_noise_scale`, `imu_rate`, `rpm_noise`, and
`dropout`. GNSS outage-duration sweeps start the gap at 5 s. Keep the scenario,
controller list, and sensor seed fixed when comparing profiles. Every
profile/controller pair runs in a separate process. If its wall-clock limit is
exceeded, the result is recorded as `campaign_timeout` and the campaign
continues with the next controller.

For a joint IMU/GNSS severity grid with multiple seeds:

```bash
python -m control.navigation_grid_campaign \
  --imu-levels 0 .125 .25 .375 .5 1 \
  --gnss-levels 0 .125 .25 .375 .5 1 \
  --sensor-seeds 1 2 \
  --only-cases gust_lateral_p10_VL \
  --only-controllers GSLQR \
  --timeout-s 60
```

The grid linearly interpolates the IMU and GNSS blocks between the nominal and
stress profiles. It intentionally leaves GNSS outage windows disabled; use the
one-factor `gnss_outage_duration` sweep to study signal loss separately.

For a matched comparison with truth, ideal sensors, and two GNSS severities:

```bash
python -m control.sensor_controller_screen \
  --controllers GSLQR V13 F13 --sensor-seeds 1 2 \
  --gnss-levels 0 .25 --timeout-s 180
python -m control.sensor_screen_report results/sensor_timing_v2/controller_screen.json
```

This saves a checkpoint after each controller run. The matched GNSS grid
sets estimator covariance from the generated GNSS sigma at every level,
including level 0. The raw bundled nominal profile uses different covariance
defaults, so results from those two tuning policies must be distinguished.

See `SENSOR_TIMING_REVIEW.md` for the corrected timing contracts and
`SENSOR_CAMPAIGN_STATUS.md` for the latest verified results.

## Feedback-path diagnosis

Use single-factor ablations to distinguish gyro differentiation from rotor
telemetry effects. These variants change sensor profiles only; zero noise and
zero latency are diagnostic controls, not proposed hardware specifications.
Each variant uses the same sensor seed and matched GNSS covariance policy.

```bash
python -m control.sensor_feedback_ablation \
  --output results/sensor_feedback_ablation_v2 --controllers V13 \
  --sensor-seeds 1 --gnss-level 0 \
  --variants gyro_noise_zero rpm_noise_zero rpm_latency_zero rotor_filter_fast \
  --timeout-s 300
python -m control.sensor_feedback_ablation \
  --output results/sensor_gyro_candidate_v2 --controllers V13 F13 \
  --sensor-seeds 1 2 --gnss-level .25 --variants gyro_noise_quarter \
  --timeout-s 300
```

The quarter-noise variant sets gyro noise density to 0.0005 rad/s/sqrt(Hz)
instead of the nominal 0.002. The ESKF process covariance uses this same
configured density. It does not add a gyro filter or change controller gains.
The faster rotor-observer variant changes its time constant from 20 ms to 2 ms.
Use a new output directory when preserving an earlier campaign.

To repeat the controller comparison under a vertical gust:

```bash
python -m control.sensor_controller_screen \
  --output results/sensor_vertical_v2 --controllers GSLQR V13 F13 \
  --sensor-seeds 1 2 --gnss-levels 0 .25 \
  --case gust_vertical_m5_VL --timeout-s 180
python -m control.sensor_screen_report results/sensor_vertical_v2/controller_screen.json
```

Post-process the saved runs to reconstruct the propulsion-domain excursions and
the body-rate differentiation error. Inputs must be complete campaign checkpoints.
Filter reconstruction reads the cutoff and S0/S1 alignment from each trial's
manifest. Historical manifests without a cutoff use their original 50 Hz value.

```bash
python -m control.sensor_feedback_diagnostics \
  results/sensor_timing_v2/controller_screen.json \
  results/sensor_vertical_v2/controller_screen.json \
  results/sensor_feedback_ablation_v2/ablation.json \
  results/sensor_gyro_candidate_v2/ablation.json \
  --output results/sensor_feedback_followup_v2
```

Diagnostics exclude the first 0.5 s only when computing command-slew and
angular-acceleration-error RMS. Acceptance verdicts retain the complete trial,
including startup. See [the feedback follow-up](SENSOR_FEEDBACK_FOLLOWUP.md)
for the interpretation and remaining work.

## INDI filter and startup experiments

Controller options belong in the arena config, independently of sensor
specifications. V13 and F13 accept these optional fields:

```json
{
  "indi_cutoff_hz": 25.0,
  "time_align": "S1",
  "rotor_startup_guard": true
}
```

Merge these fields into the selected entry under `controllers`; the example is
not a complete arena config. Omitting the new fields preserves 50 Hz and a
disabled startup guard. F13 retains S0 in the bundled config, and V13 retains S1.
All actual settings are recorded in the run manifest.

`indi_cutoff_hz` configures the existing low-pass filter on differentiated,
bias-corrected gyro feedback. It does not filter the IMU stream fed into the
ESKF or alter NMPC's observed body rate. S1 applies the same exponential LPF to
midpoint rotor-force samples. This matches the controller's digital filter
timing but does not cancel RPM transport latency or the rotor-observer LPF.
Control effectiveness and the rotor-command increment still use the current
observed rotor state; S1 is not complete actuator-state time alignment.

With `rotor_startup_guard`, the arena reports whether a rotor estimate is
available. Telemetry becomes available only after its first packet arrives;
predictor mode is available from initialization as an explicit model estimate.
Before availability, INDI uses its model-based fallback and defers derivative
and rotor-force filter initialization. No true rotor speed is passed to the
sensor controller. This is an optional startup experiment, not a general
telemetry-outage or motor-arming safety system.

The runner creates separate arena configs and keeps the sensor profile fixed.
Each output directory must be new, to prevent overwriting an earlier campaign.

```bash
python -m control.sensor_filter_experiment \
  --output results/sensor_filter_v3 --controllers V13 F13 \
  --sensor-seeds 1 2 --gnss-level .25 \
  --variants baseline aligned50 aligned25 aligned12p5 --timeout-s 240
python -m control.sensor_filter_experiment \
  --output results/sensor_startup_v3 --controllers V13 F13 \
  --sensor-seeds 1 --gnss-level 0 \
  --variants baseline startup_guard --timeout-s 240
python -m control.sensor_filter_experiment \
  --output results/sensor_filter_combined_v3 --controllers V13 F13 \
  --sensor-seeds 1 2 --gnss-level .25 --variants guarded25 --timeout-s 240
python -m control.sensor_filter_report \
  results/sensor_filter_v3/experiment.json \
  results/sensor_startup_v3/experiment.json \
  results/sensor_filter_combined_v3/experiment.json \
  --output results/sensor_filter_followup_v3
```

The runner omits V13 `aligned50` because it equals the existing V13 baseline.
For F13, use `aligned50` to separate the S0-to-S1 change from the cutoff change.
`guarded25` is also available for a combined experiment. No variant is promoted
to the default automatically.

New hybrid-controller traces include `indi_path`, `indi_n_feedback`,
`indi_rotor_ready`, `indi_rotor_sample_time_s`, `indi_omega_dot_filtered`,
`indi_rotor_thrust_raw`, `indi_rotor_thrust_used`, `indi_allocation_error`, and
`indi_rotor_increment`. These arrays align with `us` and `ts[:-1]`. Unavailable
probe quantities use NaN, and paths are saved as Unicode strings, allowing
numeric/string traces to be read without enabling pickle.

See [the filtering and startup report](SENSOR_FILTER_EXPERIMENT.md) for the
measured noise/delay tradeoff and verified baseline preservation.

## Navigation update isolation

An optional `estimator.trace_updates: true` sensor-profile field enables
per-packet navigation diagnostics. It defaults to false and does not alter
filter arithmetic. Each arena trial writes a separate `.navigation.json`
beside its `.npz` trajectory and `.solver.json` log. It records first-processing
innovations, covariance, NIS, acceptance, 15-state injections, and sample /
arrival / processing times. Separate arrival-batch records show the total
current-state change after propagation and replay. Later replays do not
rewrite the first-processing records.

The diagnostic runner keeps `configs/arena.json` and estimator covariance
fixed. It changes only declared barometer/GNSS measurement fields. Zero-noise
variants leave IMU, magnetometer, and RPM errors at their nominal settings.
The commands below recreate the saved experiment; choose new output directory
names if these paths already exist.

```bash
python -m control.navigation_update_experiment \
  --output results/navigation_updates_v4 --controllers V13 \
  --sensor-seeds 1 2 --timeout-s 240
python -m control.navigation_update_experiment \
  --output results/navigation_updates_f13_v4 --controllers F13 \
  --sensor-seeds 1 2 \
  --variants baseline baro_noise_zero gnss_noise_zero both_noise_zero \
  --timeout-s 240
python -m control.navigation_update_report \
  results/navigation_updates_v4/experiment.json \
  results/navigation_updates_f13_v4/experiment.json \
  --output results/navigation_update_followup_v4
python results/navigation_update_followup_v4/verify.py
```

The postprocessor uses actual solver timestamps to link arrival batches to
subsequent thrust requests and includes every sensor kind processed since the
previous solve. These are timing links, not a per-sensor causal decomposition
of the controller output. A final arrival with no following control or solve
is left unlinked. Detailed JSON logging costs memory and disk space and is
intended for bounded diagnosis, not real-time deployment.

See [the navigation diagnosis](NAVIGATION_UPDATE_DIAGNOSIS.md) for the initial
barometer-reference noise and delayed-GNSS replay findings, controlled outcomes,
and limits of interpretation.

## Paired preflight calibration experiments

Use `estimator.preflight_baro_samples` to install an average before flight and
`estimator.preflight_baro_rng: "independent"` to keep the in-flight noise sequence
matched when changing that count. This opt-in setting uses a separate seeded
stream. The default `"shared"` retains the earlier behavior, including the
existing stress profile's consumption of barometer samples during calibration.

The runner `control.preflight_calibration_experiment` compares baseline,
`preflight1`, `preflight25`, `preflight100`, and `preflight100_frozen`. Only the
last also sets `estimate_baro_bias: false` to isolate continuing GNSS-based
barometer-bias learning. All sensor specifications and covariance/controller
settings remain fixed. The postprocessor `control.preflight_calibration_report`
compares tracking, startup height/thrust/RPM, and bias changes during replay.

This preflight helper assumes a stationary, known reference altitude and white
independent samples; it does not simulate preflight dynamics or transport.
Nominal sampling budgets for 1/25/100 readings at 50 Hz are 0.02/0.5/2 seconds.
See [the calibration study](PREFLIGHT_CALIBRATION_STUDY.md) for the equations,
results, verification, and reproduction commands.

## Keep sensor noise and estimator assumptions separate

`imu.accel_noise_density` and `imu.gyro_noise_density` control generated IMU
samples. The sample standard deviation is density divided by the square root
of the nominal sampling period. By default the navigation filter uses the
same densities for its process covariance, preserving the previous behavior.
Optional `estimator.accel_noise_density` and `estimator.gyro_noise_density`
override only the filter's assumed densities. Both the legacy 15D and joint
16D filters support these overrides. They must be finite and nonnegative.

Use explicit estimator overrides for a **fixed-estimator sensor sweep**. Let
them follow the sensor for a **known-noise matched sweep**. These answer different
questions; record the policy in the experiment contract. The joint filter also
supports independent assumed bias-walk densities and
`estimator.preflight_baro_sigma_m`. None of these overrides changes the samples
generated by `SensorSuite`.

The matching screen now includes four diagnostic controls:

| Variant | Generated errors restored from the base profile |
|---|---|
| `quiet_sampled` | None; sampled sensors still pass through the same navigation filter |
| `imu_only` | IMU noise, bias, delay, loss and vibration |
| `navigation_only` | GNSS, barometer and magnetometer errors |
| `rotor_only` | Rotor telemetry errors and its nominal observer filter |

The controls retain rates, enabled sensors, clipping, initialization policy,
estimator kind, measurement covariance and effective process/preflight covariance.
Except in `rotor_only`, rotor telemetry uses a near-instant filter (1 microsecond
time constant) with zero transport latency. They do not inject truth states at
the controller input. They deliberately retain nominal covariance assumptions
when generated noise is zero. They are diagnostic controls, not hardware profiles.

```sh
python -m control.sensor_matching_screen --output results/sensor_groups_v8 --controllers V13 F13 --variants quiet_sampled imu_only navigation_only rotor_only baseline --seeds 3 --timeout-s 900
python scripts/sensor_group_report.py results/sensor_groups_v8/experiment.json --output results/my_sensor_group_report
```

This is a group ablation, not a complete factorial experiment: passing isolated
groups does not prove that their combination passes. A stopped run's RMSE covers
only its recorded prefix. Do not compare it as if it completed the full trial.
Reports retain stops/timeouts, source hashes, configuration contracts and separate
tracking/model-domain verdicts. Raw trajectories remain local.

To separate rotor timing effects while retaining all nominal sensor noise:

```sh
python scripts/sensor_rotor_screen.py --output results/rotor_timing_v8 --controllers V13 F13 --seeds 3
python scripts/sensor_group_report.py results/rotor_timing_v8/rotor_unfiltered/experiment.json results/rotor_timing_v8/rotor_no_latency/experiment.json results/rotor_timing_v8/rotor_direct/experiment.json --output results/my_rotor_timing_report
```

`rotor_unfiltered` sets the telemetry measurement filter to 1 microsecond but
retains the nominal transport delay. `rotor_no_latency` removes only transport
delay; `rotor_direct` removes both. Rotor measurement noise stays at the nominal
value in all three. The physical motor time constant (`tau_m`) is unchanged.
The rotor observer's `tau_s` in telemetry mode is **additional measurement
smoothing**, not the plant's motor lag. In predictor mode that same legacy field
instead controls command-response prediction. Do not infer a motor time constant
from a passing telemetry smoothing configuration.

Each condition saves its full derived configuration. The parent screen verifies
its axes, base configuration, runtime and driver hashes on resume. The nested
matching screens retain completed failures and reject reserved evaluation seeds.
Use separate output folders for concurrent controllers or repeats.
