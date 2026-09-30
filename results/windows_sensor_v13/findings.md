# Native Windows sensor-reference verification

The [Windows 2022 x64 run](https://github.com/leo11dk/fast-drone-sensor-fusion/actions/runs/36717085838)
passes **all 89 targeted tests**, static preflight and both 12-second numerical
references at source `f2dc38be2be08a9bc0ee17c14161c682867670c9`.
Python is 3.13.7; NumPy 2.3.3, SciPy 1.16.3, CasADi 3.7.2 and Matplotlib 3.10.6
match the recorded Mac environment. All four numerical thread limits are one.
There are no failed or skipped tests in this successful attempt.

## What reproduced

| Reference | Speed | Development seed | Tracking | Model domain | Reproduction |
|---|---:|---:|---|---|---|
| Timestamped projected rotor observer | 85 m/s | 3 | Pass | Pass | Pass |
| Legacy rotor observer | 20 m/s | 4 | Pass | Fail | Pass |

Each trial completes 12 seconds and 600 optimizer calls with zero optimizer
failures. Both preserve all required verdicts and meet the original
`rtol=1e-3`, `atol=1e-9` numerical criteria. No gains, equations, reference
metrics or tolerances were changed to obtain agreement. The known legacy
model-domain failure remains a failure; reproducing it is the expected result.

The Windows trajectories are **not bit-identical** to the Mac trajectories.
The absolute velocity-RMSE differences are approximately
`1.85e-12 m/s` (projected) and `1.21e-13 m/s` (legacy). These small metric
differences do not establish an equivalent bound on every trajectory sample.
See the complete [projected](projected/comparison.json) and
[legacy](legacy/comparison.json) comparisons.

After downloading the artifacts, we independently checked the 115 runtime
hashes, the 193 arena source/configuration hashes, source revision, clean Git
state and environment. Both actual trajectory hashes were recomputed from the
downloaded time/state/command arrays and matched their recorded hashes. The
reference records match the published Mac fixtures, and the comparison
function returns no problems for either record.

## Failed attempts remain recorded

1. [Initial workflow](https://github.com/leo11dk/fast-drone-sensor-fusion/actions/runs/36716122629):
   GitHub rejected a `runner.temp` expression in job-level environment settings.
   No runner, tests or simulations executed. The cache setting was moved into
   a runner step.
2. [First native test attempt](https://github.com/leo11dk/fast-drone-sensor-fusion/actions/runs/36716291781):
   88 tests passed and one expected Unix path spelling on Windows. Numerical
   replays were skipped. The test now constructs a native temporary path;
   the process-environment implementation is unchanged. All seven preflight
   tests passed locally after that test repair.
3. The final run above passes the unchanged 89-test selection and both replays.

The failed attempts are execution/portability checks, not additional failed
flight trials. Their metadata and failed test XML are retained. They are not
pooled with the 93-task development study or counted as reliability samples.

## Evidence and limits

The [verification index](verification.json) hashes the compact evidence and all
24 downloaded artifact files. [Successful tests](windows_tests.xml),
[the prior test failure](attempt_2_tests.xml), [preflight](preflight.json),
[run metadata](attempt_3.json), [artifact receipt](artifact_receipt.json) and
both run manifests are retained. Complete logs and approximately 6 MB of raw
flight traces are saved in the Desktop checkout; they are not committed.

The refreshed Desktop archive is `sensor-development-v13-windows.zip`, built
from clean revision `d2e7b6fed8e158bd3d291c34227f85de04043456`. It includes the
native-path test repair and 312 source/data/document files. Its runtime hashes
match the verified Windows run. A fresh Mac extraction passes package byte,
dependency and model checks plus all six static preflight checks; this package
relocation check adds no new flight trials. See the [archive receipt](package_receipt.json),
[package check](package_check.json), [source preflight](package_source_preflight.json)
and [extracted preflight](package_preflight.json). The archive SHA-256 is
`d80b68ae88e3827d9089e23ebb32f47113049a63515c0ef10e98d359f19e3fae`.

This establishes numerical reproduction of **two V13 cases on one hosted
Windows machine**. It does not validate the entire envelope, the four school
computers, orphan-worker behavior or transfer of a completed full tuning
record. Short tuning/distributed unit checks do not replace those procedures.
Final tuning and held-out paper experiments have not started. Follow the
[Windows handoff](../../docs/SENSOR_WINDOWS_DEVELOPMENT.md) and resolve the
[pre-tuning findings](../sensor_validation_v12/findings.md) before freezing the
paper configuration.
