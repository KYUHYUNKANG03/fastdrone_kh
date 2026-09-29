# fast-drone-sensor-fusion

Sensor fusion and sensor-uncertainty experiments for **High-Speed Multicopter
Control Architecture Robust to Disturbances and Sensor Uncertainty:
NMPC–INDI Interface Separation and Sensor–Controller Matching**.

This repository continues the team simulation from
[`protkjj/fast-drone`, branch `protkjj/arena-completion`](https://github.com/protkjj/fast-drone/tree/protkjj/arena-completion).
The imported snapshot was checked against commit
`6cad076e05a881ff63c6c302ae5a5d177e605739`. This local source snapshot starts a new
Git history; it does not claim to reproduce upstream commit history. Existing
model attribution and notices remain in `models/team_light/` and `external/`.

## Current status

**Development software, not validated flight hardware or final paper results.**
The v7 checkpoint passed 230 software tests, with eight long legacy tests skipped.
Nineteen development trials use Python 3.13.7 and the pinned dependencies.
Both V13/F13 pass the 85 m/s truth and ideal-sensor references. All eleven tested
noisy high-speed variants fail tracking and propulsion-domain checks. Four
20 m/s baseline/fault runs pass tracking but exceed the propulsion model's domain.
These are limited development comparisons, not estimated failure probabilities.

- [Latest report and comparison figure](results/sensor_readiness_v7/report.md)
- [Sensor equations and implementation](docs/SENSOR_FUSION.md)
- [Sensor–controller matching protocol](docs/SENSOR_PAPER_PROTOCOL.md)
- [Team handoff and remaining experimental gates](docs/SENSOR_TEAM_HANDOFF.md)
- [Archived upstream root README](docs/history/UPSTREAM_ROOT_README_20260930.md)

## Feedback architecture

```text
truth plant [p, v, q, angular rate, rotor speed]
  -> timestamped IMU / GNSS / barometer / magnetometer / rotor measurements
  -> replay-capable navigation ESKF + separate rotor observer
  -> estimated 17-value state -> unchanged controller interface -> plant
```

Truth-feedback configurations remain available. The optional `joint_baro`
estimator has 16 error states; the legacy ESKF has 15. Sensor profiles configure
noise, bias, rates, latency, dropouts, vibration, outliers, and initialization.
The `rpm` profile fields use mechanical **rad/s**. Model-domain failures are
reported separately from tracking failures.

## Run locally

Use Python 3.13 (the recorded Mac environment is 3.13.7). From a shell:

```sh
git clone https://github.com/leo11dk/fast-drone-sensor-fusion.git
cd fast-drone-sensor-fusion
python3.13 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-lock.txt
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
```

Reproduce the recorded sensor-inclusive development case:

```sh
python scripts/sensor_reproduce.py --output results/my_sensor_reproduction
```

A reproduction pass means agreement with the recorded outcomes, including the
reference's known model-domain failure. Each replay uses a new output folder.

Run or resume a paired high-speed development comparison:

```sh
python -m control.sensor_matching_screen --output results/my_matching_screen --controllers V13 F13 --variants baseline startup_guard gyro_quiet rotor_tau2ms --seeds 3
```

Repeat the same command to resume; source/configuration changes require a new
folder. Completed failures and timeouts remain in the record. Do not run two
writers on one output folder. The development screen rejects reserved tuning
and main-experiment seeds. Controller gains and aircraft defaults are preserved.

A focused software check:

```sh
ARENA_QUICK=1 python -m pytest -q control/test_arena_sensor_fusion.py control/test_joint_estimator.py control/test_sensor_binding.py control/test_sensor_matching_screen.py scripts/test_sensor_reproduce.py scripts/test_sensor_workspace.py
```

For a portable Windows handoff, first create a source package:

```sh
python scripts/sensor_workspace.py build --output sensor-development.zip
```

Then follow [the Windows package instructions](docs/SENSOR_WINDOWS_DEVELOPMENT.md).
Native Windows execution and worker checks still need verification. Final tuning
must satisfy the separate gates in the team's `DISTRIBUTED_RUN.md`; the old
truth-only tuning records are not final sensor-inclusive tuning results.

## Version control and results

`origin` is this repository. Future verified changes are committed and pushed
here in coherent checkpoints. Python environments, credentials, caches, local
backups, bulk traces, and generated archives are ignored. Selected v7 summaries
and reference fixtures are versioned; raw campaigns remain in the local Desktop
checkout. Historical documentation can refer to those local-only artifacts.

Do not fill final tuning hashes with development records. Final paper work still
requires sensor/controller matching, equal-budget retuning, and held-out testing.
No additional license is imposed on the imported team or third-party material.
