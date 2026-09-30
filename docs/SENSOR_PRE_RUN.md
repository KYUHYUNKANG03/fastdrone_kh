# Sensor experiment preparation — v11

The repository is prepared for bounded development validation. The final paper
campaign remains gated. V11 changes preparation, portability and reporting;
it does not change the 114 runtime sources, aircraft, controller gains, truth
baseline, sensor candidate, or historical reproduction fixtures. No new flight
campaign was started for this checkpoint.

## Run order

| Step | Purpose | Advance when |
|---|---|---|
| Local static preflight | Dependencies, thread limits, fitted model, arena facts, reference compatibility | All development checks pass |
| Numerical reproduction on each execution machine | Reproduce the v9 recorded sensor trajectory and verdicts | Metrics meet recorded tolerances; failures are investigated |
| Controller integration | Truth, quiet sampled and nominal fusion at the declared low/high speeds | Runs and records are trustworthy; characterize any tracking/domain failures |
| GNSS startup factorial | Separate noise, delay and their interaction at fixed filter/controller settings | Multi-seed diagnostics explain whether a startup/interface change is justified |
| Broader development envelope | Braking, mission, initial errors, faults and motor mismatch, with truth controls | Supported and unsupported conditions are documented |
| Freeze and equal-budget tuning | Select sensor policy and protocol; update the final paper spec | All five final records and required design/reproduction checks are valid |
| Held-out paper runs | Evaluate interface separation and sensor–controller matching | Frozen release, matrix, sample size and analysis rules; all guards pass |

An aircraft/model-domain failure is a result to preserve, not a reason to
delete a case, loosen an acceptance threshold, or silently tune a controller.
The integration stage need not produce all successful flights to be informative.

## Static preflight (does not simulate)

From the repository root, use the pinned interpreter:

```sh
python scripts/sensor_preflight.py --output results/preflight_local.json
```

The CLI sets all four BLAS thread limits to one in its own process, checks the
v9 development configuration, and writes a new report without replacing an
existing one. On this Mac it selects the installed Command Line Tools Git for
child-process provenance. It does not change the parent shell or accept an
Xcode license. Missing dependencies fail on every platform; permitted Windows
version differences remain warnings and require numerical reproduction.

`development_ready` means the static prerequisites for bounded experiments
passed. It is not a flight-quality verdict. The report lists checks it did not
perform. `--require paper` deliberately exits nonzero at this development
checkpoint, even if local checks pass. Research gates listed there are an
explicit checklist, not automatically verified by inspecting result filenames.
The existing main-run hash guards remain authoritative as well.

The current main-paper spec still selects v6. It must not be silently changed
to a development candidate and described as frozen. Its five empty tuning
hashes correctly prevent final execution.

## Saved development plan (does not simulate)

```sh
python scripts/sensor_preparation.py plan --output results/prepared_sensor_validation_v11_final
python scripts/sensor_preparation_report.py --source results/prepared_sensor_validation_v11_final --output results/prepared_status_before_run
```

The default plan records 93 unique tasks using development seeds 3, 4 and 5.
It embeds every full input configuration, truth pairing, runtime hashes,
driver hash and timeout. Planning validates configuration facts without
constructing controllers or launching simulations. Repeating an identical
plan is allowed; changed settings require a new directory. Never edit the
saved JSON to bypass a guard.

| Stage | Cases | Unique tasks |
|---|---|---:|
| `integration` | V13/F13/M17/GSLQR at 20 and 85 m/s; CPID at 20 m/s only. Each controller/case has one truth control, one quiet sampled control, and nominal fusion at three seeds. | 45 |
| `gnss_startup` | V13/F13 at 20 m/s, two sensor contexts × two GNSS noise levels × two delays × three seeds. Reuses two integration truth controls. | 48 additional |

Running only `gnss_startup` executes at most 50 tasks, including its two truth
dependencies. Running integration afterwards reuses those controls. This is
one writer at a time; stages do not run concurrently against the same folder.
The M17 stage should be run serially on this 8 GB Mac. The 1200 s timeout is
recorded per trial, not a runtime prediction. On a slower computer choose a
larger timeout **when making a new plan**, before starting.

The GNSS axes are generated position/velocity noise `(0, 0)` versus
`(0.8 m, 0.15 m/s)` standard deviations, and transport delay `0` versus
`0.12 s`. Rate stays at 10 Hz. Bias, dropout and outliers must be absent in
the nominal GNSS block for this experiment. Navigation Q, R, initial P and
preflight covariance stay nominal; innovation/update logging is enabled.
Controllers, plant, mission, references and acceptance criteria stay fixed.

In the **isolated** context all other sensors use the existing quiet sampled
diagnostic configuration, including its near-instant rotor measurement filter.
In the **full** context all other sensors and the timestamped rotor observer
retain the v9 candidate. Compare noise/delay within each context; their context
difference bundles the other sensor groups and rotor policy. Zero-latency and
quiet cases are diagnostic controls, not proposed hardware specifications.

Identical seeds pair random draws across cells, not the resulting closed-loop
trajectories. Quiet deterministic repetitions do not add independent evidence.
All these seeds remain development seeds, including ones used previously.

## Execute later, explicitly

First reproduce on the execution machine with the existing tool:

```sh
python scripts/sensor_reproduce.py --output results/reproduction_before_validation
```

The command above **does simulate**. It checks numerical agreement, not general
robustness. Then start one prepared stage:

```sh
python scripts/sensor_preparation.py run --output results/prepared_sensor_validation_v11_final --stage integration
python scripts/sensor_preparation.py run --output results/prepared_sensor_validation_v11_final --stage gnss_startup
```

The [startup study guide](SENSOR_STARTUP_STUDY.md) documents an optional two-worker
V13/F13 GNSS executor and the full-plan diagnostic report. Integration, including
M17, stays serial on the reference Mac. The alternative executor records its
own source contract and uses the same exclusive plan lock.

Each `run` first audits the local development prerequisites and verifies the
saved plan byte/content contract. Each task uses a fresh subprocess. Completed
failures and timeouts remain recorded and are skipped on resume. Missing trial
output is saved and stops the batch for diagnosis. An interrupted task without
a record is retried on the next explicit run; preserve partial artifacts.

A lock prevents two writers. A hard crash can leave `run.lock`; inspect its PID
and running workers before removing only that stale lock. No script kills
unrelated jobs or deletes a checkout. Storage subdirectories use short hashed
IDs; the readable identities remain in `plan.json` and `results.json`.

After any stage, write a new report directory:

```sh
python scripts/sensor_preparation_report.py --source results/prepared_sensor_validation_v11_final --output results/prepared_status_after_integration
```

The report retains all planned cases, including pending and execution-incomplete
cases. It checks trial identity, source/configuration hashes and physical
parameters before calculating trace diagnostics. Zero-step and nonfinite failure
traces retain their original verdicts with an explicit unavailable-diagnostic
reason. Corrupted shapes, mismatched provenance and domain reconstruction
disagreements still raise errors. Missing traces are flagged, never interpreted
as passing diagnostics.

## Interpret and freeze before paper runs

Read full duration, early-stop reason, optimizer failures, tracking pass and
propulsion-domain validity separately. RMSE from an early-stop prefix must not
rank above a completed trial merely because it is smaller. Inspect position,
velocity, attitude-angle, body-rate and rotor-state estimation RMS, rotor
sample age/readiness, command variation and available INDI derivative errors.
GNSS update logs allow comparing first-arrival corrections, innovations and
gating with the first domain crossing. Do not remove startup from acceptance.

Use the factorial to decide whether to change causal startup initialization,
latency handling, or an explicitly tested interface policy. Do not shrink the
initial covariance just because simulator truth is available. The current
exact-navigation initialization and steady-trim rotor history are assumptions
to test, not evidence of realistic takeoff.

For sensor matching, predeclare fixed-Q/R versus matched-Q/R comparisons and
the V13 interface ladder separately. Tune each eligible controller under the
same declared budget and sensor policy using the tuning seed range. Keep
held-out seeds 1000–1999 untouched until policy, source/config hashes, tuning
records, analysis rules and sample count are frozen. Three development seeds
do not estimate a reliable hardware failure boundary. Choose final repeats
from the precision needed for failure probability and paired performance,
report uncertainty per condition, and avoid pooling unlike cases.

Hardware claims also require documented noise-density/PSD and bandwidth
conventions, sampling/anti-aliasing assumptions, latency and jitter, bias/drift,
vibration, clipping and ESC electrical-to-mechanical speed conversion. The
current engineering profiles are not calibrated device specifications.

See [the full protocol](SENSOR_PAPER_PROTOCOL.md), [latest evidence](../results/sensor_readiness_v10/findings.md),
[broader envelope commands](SENSOR_ENVELOPE_SCREEN.md) and the team's
[distributed tuning/main procedure](../DISTRIBUTED_RUN.md) for the later gates.
