# Integration and GNSS startup development study

This study executes the [v11 preparation plan](SENSOR_PRE_RUN.md) with fixed
aircraft, controller gains and v9 sensor/observer runtime. Its 45 integration
tasks and 48 additional GNSS tasks use development seeds only. The plan is
complete: all 93 tasks finish 12 seconds, with 56 full passes, 34 model-domain
failures and three pre-gust settling failures. Read the
[v12 findings](../results/sensor_validation_v12/findings.md) and
[verification index](../results/sensor_validation_v12/verification.json).
Successful execution does not imply that every controller case passed or that
final tuning is ready.

The commands below describe the archived study at publication revision
`06dcb88b91e3330416ca18421dc486ce409e27fa`. Current source uses the portable v13
fingerprint runtime and intentionally rejects this old saved plan. Retain the
raw study folders with the publication revision for reanalysis; create fresh
plans/output directories for new work. See [source portability](SOURCE_PORTABILITY.md).

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
source hash, selected worker count (one or two) and task identities. It shares the
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

## Overlap stages in separate folders on the same host

To use two total simulation processes while the integration runner executes
M17 serially, take a read-only snapshot and run **one** GNSS worker there:

```sh
python scripts/sensor_preparation_split.py snapshot --source results/prepared_sensor_validation_v11_final --output results/prepared_sensor_validation_v12_gnss
python scripts/sensor_preparation_parallel.py --output results/prepared_sensor_validation_v12_gnss --workers 1
```

The snapshot captures one atomic results checkpoint, including already completed
truth controls. It does not copy raw traces: their absolute paths still point
to the original same-host folders. The two stages now have separate task storage,
locks and result writers. Keep the original folders and do not treat this as a
portable archive. Worker count is immutable for a started executor directory.

After **both** runners have finished, merge into the original plan:

```sh
python scripts/sensor_preparation_split.py merge --target results/prepared_sensor_validation_v11_final --source results/prepared_sensor_validation_v12_gnss
```

Merge exclusively locks both directories, requires identical source-bound plans,
rejects unknown trials and conflicting duplicates, and preserves every recorded
failure. Shared controls must be identical records. It records input hashes,
added/shared task IDs and the secondary executor contract in `merge_receipt.json`.
The final diagnostic report then verifies the merged records against the actual
simulation traces and physical parameters. A snapshot is not extra experimental
evidence; the resulting denominator remains 93 unique tasks.

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

For packet counts and NIS summaries, use the `update_statistics` reconstructed
from the first-processing navigation log. The legacy `estimator_diagnostics`
object is rebuilt from the retained history: its `updates` and `rejected_updates`
are not whole-flight totals. Its `delayed_updates` uses packet age at the current
filter time, rather than first-arrival latency, and `replays` includes ordinary
new-IMU processing calls. Those fields must not be interpreted as a transport-delay
failure rate or as the number of late packets. The current study retains those
raw fields but uses sample/arrival/processing timestamps for its timing analysis.

The four GNSS noise/delay cells are compared within each sensor context.
The isolated context also removes other sensor errors and uses the quiet rotor
policy, so the difference between contexts cannot be assigned to GNSS alone.
Unique trajectory counts identify deterministic repetitions. The full nominal
cells additionally check trajectory and verdict agreement with their serial
integration counterparts, including the effect of detailed update logging.

## Before final tuning

The inherited tuning failure rule is `stop_reason or paper_failed`. It does
not include model-domain validity or every tracking acceptance check, including
pre-gust settling. The report counts acceptance failures that this rule would
not penalize and separately reports the domain-failure subset. It does not run
tuning or move these development cases into the tuning set. Declare the treatment
of these criteria before final tuning. Avoiding the binary failure penalty does
not remove the ordinary RMSE cost. A model-domain exit is not evidence
that the real aircraft would fail, and a low RMSE outside that domain does not
establish a supported simulation result.

The sensor/initialization policy, final objective, matched-estimator/interface
comparisons and held-out sample size still require a documented freeze. Three
development seeds cannot establish a hardware reliability boundary. See the
[paper protocol](SENSOR_PAPER_PROTOCOL.md) for the remaining experimental gates.
