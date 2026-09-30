# Paired sensor operating-envelope screen — v10

This is development validation with fixed controller gains. It extends the v9
rotor observer beyond the single high-speed lateral-gust case. It does not freeze
a hardware specification, retune controllers, or consume held-out seeds.

## Physical and measurement comparisons

For each condition, the runner records a fused-feedback trial and a truth-feedback
control with the same physical scenario, controller settings, reference and
acceptance criteria. A deterministic truth trial can serve multiple sensor-only
conditions or seeds. Its `truth_trial_id` is explicit; repeated uses of the same
truth control must not be counted as independent samples.

The base is `configs/arena_rotor_projected_development_v9.json`. Controller gains,
the prediction model, aircraft definition and nominal sensor settings stay fixed.
The derived input retains all required arena scenarios and adds explicitly named
development scenarios where needed. Only the selected case executes.

| Condition | Deliberate change |
|---|---|
| `lateral_low`, `lateral_high` | Existing lateral gust at 20 / 85 m/s |
| `vertical_low`, `vertical_high` | Existing vertical gust at 20 / 85 m/s |
| `acceleration`, `braking`, `mission` | Existing arena reference / mission |
| `altitude_low` | Smooth +1 m altitude step at 20 m/s, 8 s trial |
| `plant_tau10ms`, `plant_tau40ms` | Physical motor tau 0.5× / 2×; observer remains 20 ms |
| `rpm_outage200ms` | No rotor samples during [3.5, 3.7) s of the high-speed lateral gust |
| `rpm_latency20ms` | Rotor transport latency 20 ms, other settings unchanged |
| `rpm_rate100hz` | Rotor sampling 100 Hz, other settings unchanged |
| `gnss_outage1s` | No GNSS samples during [3.5, 4.5) s |
| `initial_error` | Initial p error [0.2, −0.1, 0.3] m, v error [0.1, −0.1, 0.05] m/s, attitude error [0.5, −0.5, 1] degrees |

The outage intervals specify **sample times**: packets already in transit can
still arrive. The rotor observer continues model prediction during the outage.
With the unchanged controller defaults, stale `ready=false` does not activate a
fallback. The report therefore records stale time and allocations while stale,
alongside the actual tracking and model-domain verdicts.

These physical motor variations differ from the v9 `tau10ms` / `tau40ms` screen,
which changed the observer assumption while leaving the physical motor at 20 ms.
The new report reconstructs and verifies the recorded physical parameter hash,
then lists both actual and assumed time constants.

## Run and resume

Use the pinned Python environment and single-thread BLAS settings from the
[distributed guide](../DISTRIBUTED_RUN.md). On the reference Mac use
`.venv-paper/bin/python`; on a fresh installation use the locked `.venv`.

```sh
python scripts/sensor_envelope_screen.py --output results/envelope_new --plan-only
python scripts/sensor_envelope_screen.py --output results/envelope_new --controllers V13 F13 --seeds 3
python scripts/sensor_envelope_report.py results/envelope_new/experiment.json --output results/envelope_report_new
```

The default eight conditions are low-speed lateral gust, high-speed vertical
gust, acceleration, altitude step, two physical motor mismatches, rotor outage,
and initial error. For two controllers and one development seed this is **30
distinct trials: 14 truth controls + 16 fused-feedback trials**. It is a diagnostic
sample, not a statistical estimate of failure probabilities.

The same command resumes only missing records. Changes to configuration,
conditions, seeds, source code, driver, or timeout require a **new output folder**.
Timeouts and failed trials stay recorded and are not silently retried. Missing
trial output stops the batch after saving the failure; inspect `campaign.log` and
use a new folder after correcting an infrastructure/configuration problem.

Controllers can run in separate output directories. Combine both manifests in
the report command. Avoid running two writers against the same output directory.
The v10 reference run used one V13 process and one F13 process concurrently, each
with single-thread BLAS. Wall times reflect that execution arrangement.

For a later, explicitly separate extension:

```sh
python scripts/sensor_envelope_screen.py --output results/envelope_extension --conditions lateral_high vertical_low braking mission rpm_latency20ms rpm_rate100hz gnss_outage1s --seeds 4 5
```

That command is a planned extension, not evidence that those conditions have run.
Select development seeds outside the reserved main/tuning ranges.

## Follow-up startup diagnosis

The v10 low-speed result prompted an explicitly adaptive V13 diagnosis, using
development seed 3. The existing group screen tests quiet sampled measurements,
IMU-only, navigation-only, rotor-only and the full profile. A second screen
restores GNSS, barometer or magnetometer individually on the same quiet control.
These sensor-group experiments preserve the estimator's nominal process and
measurement covariance assumptions, including preflight barometer covariance.
They do not feed truth states directly to the controller.

```sh
python -m control.sensor_matching_screen --config configs/arena_rotor_projected_development_v9.json --output results/startup_groups_new --controllers V13 --seeds 3 --variants quiet_sampled imu_only navigation_only rotor_only baseline --cases gust_lateral_p10_VL
python scripts/sensor_navigation_screen.py --output results/navigation_startup_new --controllers V13 --seeds 3
python scripts/sensor_group_report.py results/startup_groups_new/experiment.json results/navigation_startup_new/gnss/experiment.json results/navigation_startup_new/barometer/experiment.json results/navigation_startup_new/magnetometer/experiment.json --output results/startup_report_new
```

The group controls deliberately simplify sensor errors and are **diagnostic
ablations**, not alternative hardware recommendations. Restoring a sensor block
restores both its noise and latency; this does not separate those two effects.
The eight adaptive trials are reported separately from the 30-trial paired screen.
Their complete profiles and outcomes are in the [startup report](../results/sensor_readiness_v10/startup_groups/report.md).

## Interpret the outcomes

- Track pass and propulsion-domain pass remain separate. Passing tracking while
  leaving the propeller data range is not a full pass under the existing rule.
- Compare truth and fusion before assigning a failure to sensor uncertainty. A
  truth failure can reveal an existing controller/scenario/model limitation.
- Report every seed and failure. A stopped run's RMSE covers only its prefix.
- Exact initial navigation state remains the default except `initial_error`.
  The steady-trim rotor prehistory remains an explicit assumption, not simulated
  takeoff or a full navigation warm-up.
- The sensor-only faults are isolated here. Combinations of physical mismatch
  and telemetry loss are a separate experiment, not covered by single-factor
  successes.
- Hardware data conversion, repeated seeds, broader maneuvers, declared fault
  handling, equal-budget tuning and native Windows reproduction still precede
  final paper claims. See the [paper protocol](SENSOR_PAPER_PROTOCOL.md).
