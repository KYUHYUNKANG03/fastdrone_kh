# Integration and GNSS startup development study

This study executes the [v11 preparation plan](SENSOR_PRE_RUN.md) with fixed
aircraft, controller gains and v9 sensor/observer runtime. Its 45 integration
tasks and 48 additional GNSS tasks use development seeds only. The plan is
currently being executed; the commands and analysis tools below do not imply
that all results have passed or that tuning is ready.

## Serial integration, then optional two-worker GNSS execution

Run numerical reproduction on the execution machine before the study. Use the
existing serial runner for integration, particularly M17 on the 8 GB Mac:

```sh
python scripts/sensor_preparation.py run --output results/prepared_sensor_validation_v11_final --stage integration
```

After that command finishes, the GNSS stage can use two independent V13/F13
workers instead of the serial command in the pre-run guide:

```sh
python scripts/sensor_preparation_parallel.py --output results/prepared_sensor_validation_v11_final
```

The parallel executor retains the scientific plan and its runtime/configuration
hashes. It writes a separate immutable `parallel_executor.json` with its own
source hash, two-worker limit and selected task identities. It shares the
serial runner's exclusive lock, so the two commands cannot write concurrently.
Each worker launches a fresh, single-threaded simulation subprocess into a
distinct task directory. One parent writes records atomically in plan order.

The executor reuses the two low-speed truth controls, preserves completed
failures and timeouts on resume, and saves the other in-flight result before
stopping after a broken command. M17 and integration are not supported by this
parallel executor. Wall-clock timeout is still the value fixed in the original
plan; change it only through a new plan, preserving the previous outcomes.

Two simultaneous local reference replays were bit-identical to the recorded v9
trajectory. This supports use of the executor on this Mac; it does not replace
numerical reproduction on another computer or provide a controller speed
benchmark. Timings collected during overlapping jobs include resource contention.

## Analyze all planned tasks

Once the plan has no pending tasks, use a fresh output directory:

```sh
python scripts/sensor_startup_diagnostics.py --source results/prepared_sensor_validation_v11_final --output results/sensor_validation_v12/final_analysis
```

The analysis first verifies source/configuration provenance, physical parameters
and recorded domain excursions using the existing prepared-plan reporter. It
preserves command failures, early simulation stops, and tracking/domain verdicts
separately. Non-INDI controllers retain state-estimation diagnostics without
inventing unavailable INDI probe measurements.

The first 0.5 s is a diagnostic window, not an exclusion from acceptance.
The report records initial state errors, first GNSS processing, historical
sample-time corrections, actual rotor minima, and first domain crossing.
An accepted delayed update acts at its historical sample time; propagation and
replay can change the controller's current estimate. An individual historical
correction must not be interpreted as the whole arrival-time state jump.

The four GNSS noise/delay cells are compared within each sensor context.
The isolated context also removes other sensor errors and uses the quiet rotor
policy, so the difference between contexts cannot be assigned to GNSS alone.
Unique trajectory counts identify deterministic repetitions. The full nominal
cells additionally check trajectory and verdict agreement with their serial
integration counterparts, including the effect of detailed update logging.

## Before final tuning

The inherited tuning failure rule is `stop_reason or paper_failed`. It does
not include model-domain validity. The report counts full-duration domain-only
failures that this rule would not penalize; it does not run tuning or move these
development cases into the tuning set. Declare the treatment of unsupported
propeller operation before final tuning. A model-domain exit is not evidence
that the real aircraft would fail, and a low RMSE outside that domain does not
establish a supported simulation result.

The sensor/initialization policy, final objective, matched-estimator/interface
comparisons and held-out sample size still require a documented freeze. Three
development seeds cannot establish a hardware reliability boundary. See the
[paper protocol](SENSOR_PAPER_PROTOCOL.md) for the remaining experimental gates.
