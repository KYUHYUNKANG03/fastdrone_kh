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
The v9 checkpoint implements a timestamp-aware rotor observer and an explicit
steady-trim measurement-history startup. It passed 277 distinct software tests,
with eight long legacy tests skipped. Both V13/F13 now pass a full 85 m/s trial
with nominal sensor noise and the original 4 ms rotor telemetry delay when using
the declared nominal motor model and startup history. Cold starts and incorrect
observer models are tested separately and retain their failures. These are
development findings, not hardware limits or failure probabilities; wider
validation and controller matching remain necessary.

- [Latest report and comparison figure](results/sensor_readiness_v9/report.md)
- [Rotor observer equations, startup assumptions and commands](docs/ROTOR_OBSERVER.md)
- [Historical v8 diagnosis](docs/SENSOR_GROUP_DIAGNOSIS.md)
- [Historical v7 report](results/sensor_readiness_v7/report.md)
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

A reproduction pass means agreement with a recorded outcome, not validation of
the full operating envelope. The active v9 fixture is a passing 85 m/s V13 run
with the new observer. Each replay uses a new output folder. The separate
`scripts/data/sensor_legacy_reproduction_v9.json` fixture preserves the 20 m/s
legacy-observer regression, including its known model-domain failure. Its fresh
v9 trajectory is identical to v8 on the pinned Mac. Archived fixtures retain
their original runtime hashes and require their own code revision.

Compare the new observer and startup policies without changing controller gains:

```sh
python scripts/sensor_rotor_screen.py --output results/my_rotor_observer --controllers V13 F13 --seeds 3 --conditions rotor_projected_warm rotor_projected_cold rotor_telemetry_warm rotor_unfiltered_warm
```

`configs/arena_rotor_projected_development_v9.json` exposes the new configuration
with development seed 3. The original v6 candidate and main-paper configuration
remain unchanged; no final tuning profile has been frozen.

Run or resume a paired high-speed development comparison:

```sh
python -m control.sensor_matching_screen --output results/my_matching_screen --controllers V13 F13 --variants baseline startup_guard gyro_quiet rotor_tau2ms --seeds 3
```

Isolate sensor groups with fixed estimator assumptions, or separate rotor timing:

```sh
python -m control.sensor_matching_screen --output results/my_sensor_groups --controllers V13 F13 --variants quiet_sampled imu_only navigation_only rotor_only baseline --seeds 3 --timeout-s 900
python scripts/sensor_rotor_screen.py --output results/my_rotor_timing --controllers V13 F13 --seeds 3
python scripts/sensor_group_report.py results/my_sensor_groups/experiment.json --output results/my_group_report
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
backups, bulk traces, and generated archives are ignored. Selected v7/v8/v9 summaries
and reference fixtures are versioned; raw campaigns remain in the local Desktop
checkout. Historical documentation can refer to those local-only artifacts.

Do not fill final tuning hashes with development records. Final paper work still
requires sensor/controller matching, equal-budget retuning, and held-out testing.
No additional license is imposed on the imported team or third-party material.
