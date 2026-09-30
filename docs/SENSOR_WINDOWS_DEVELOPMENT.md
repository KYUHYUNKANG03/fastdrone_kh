# Windows sensor development package — v13 source portability

This package exercises the sensor path on the team aircraft. It is not the
final tuning release. The new observer has bounded development checks only. A
successful software/reproduction check must not be reported as successful
flight performance. No existing checkout needs to be removed.

## Receive and check the package

Copy `sensor-development-v13.zip` and its receipt to the school computer. Compare
the ZIP SHA-256 with the receipt, then extract into a **new empty directory**.
The ZIP has its repository files at the top level; open PowerShell there.
It excludes Git metadata, credentials, Python environments, and result trees.
Python 3.13 and the five packages in `requirements-lock.txt` are required.
Use a short extraction root such as `C:\drone\v13` to leave room for generated
trace paths. The package manifest selects the actual v9 development
configuration, rather than identifying the historical v6 candidate.
Runtime, tuning and arena source manifests now use relative paths with `/`
separators on both operating systems, while retaining exact byte hashes.
Tests with Windows/Posix path objects verify that naming rule; native Windows
numerical reproduction, worker lifecycle and tuning-record transfer still
require separate end-to-end checks. See the [source transition](SOURCE_PORTABILITY.md).

```powershell
Get-FileHash .\sensor-development-v13.zip -Algorithm SHA256
```

After extraction, run one command at a time from the extracted directory:

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
$env:OMP_NUM_THREADS="1"
$env:OPENBLAS_NUM_THREADS="1"
$env:VECLIB_MAXIMUM_THREADS="1"
$env:MKL_NUM_THREADS="1"
$env:PYTHONUTF8="1"
.\.venv\Scripts\python.exe scripts\sensor_workspace.py check > sensor_workspace_check.json
if ($LASTEXITCODE -ne 0) { throw "Workspace check failed; inspect sensor_workspace_check.json" }
```

This checks source/data bytes, dependency versions, thread limits, configuration
hash, and fitted controller-model coefficients using the team's existing checks.
The manifest also detects extra Python/config files in its covered directories.
Python/package patch differences may be WARN on Windows under the team's policy.
The numerical replay below is therefore a separate required check.

No PowerShell activation or execution-policy change is necessary; these commands
call the virtual environment's Python directly. Set the environment variables
again after opening a new PowerShell session.

## Reproduce the sensor reference

```powershell
.\.venv\Scripts\python.exe scripts\sensor_reproduce.py --output results\sensor_reproduction_windows
if ($LASTEXITCODE -ne 0) { throw "Sensor reproduction differs; inspect comparison.json" }
```

The default reference is an actual 12-second V13 run at 85 m/s, development seed 3,
with the joint barometer filter, timestamp-aware rotor observer and 0.1 s of
steady-trim rotor measurement history. It passed tracking and the propulsion
domain check. Replay must reproduce both outcomes, sensor identity,
solver counts, and numeric metrics within `rtol=1e-3`, `atol=1e-9`. Trajectory
bit identity is recorded separately. Keep `comparison.json`, the run manifest,
logs, and trace files. Each retry uses a new output folder.

The default fixture is `scripts/data/sensor_reproduction_v13.json`. The source
package check verifies the selected v9 configuration/model; this numerical
replay additionally enforces that configuration and the v13 runtime hashes.
To replay the separate legacy-observer regression at 20 m/s, pass
`--reference scripts/data/sensor_legacy_reproduction_v13.json`. That recorded run
retains its known propulsion-domain failure. Both references were freshly run
after the source-manifest change and checked against their v9 counterparts.
Archived fixtures are not silently relabeled for newer code.

This checks one sensor-inclusive serial case. It does not replace the original
full tuning-path reproduction or Windows orphan-worker check in
`DISTRIBUTED_RUN.md`. Native Windows execution of these new commands remains
unverified until run on a school computer.

## Prepare the next stages without running them

These commands only check and plan; they do not start a flight campaign:

```powershell
.\.venv\Scripts\python.exe scripts\sensor_preflight.py --output results\preflight_windows.json
if ($LASTEXITCODE -ne 0) { throw "Static preflight failed" }
.\.venv\Scripts\python.exe scripts\sensor_preparation.py plan --output results\prepared_v13
.\.venv\Scripts\python.exe scripts\sensor_preparation_report.py --source results\prepared_v13 --output results\prepared_status_before_run
```

After the numerical replay succeeds and its artifacts have been reviewed, run
one stage at a time. These commands **do start simulations**:

```powershell
.\.venv\Scripts\python.exe scripts\sensor_preparation.py run --output results\prepared_v13 --stage integration
if ($LASTEXITCODE -ne 0) { throw "Integration stage interrupted; inspect retained logs" }
.\.venv\Scripts\python.exe scripts\sensor_preparation.py run --output results\prepared_v13 --stage gnss_startup
```

The stages share truth controls and preserve failed trials. Read the
[plan, resume and interpretation rules](SENSOR_PRE_RUN.md). A native-Windows
worker-lifecycle check is still required for final distributed tuning; importing
the verifier successfully does not establish that check. Memory measurements
are explicitly unavailable where the Unix `resource` module is absent.

To build an equivalent portable archive on the source machine:

```sh
python scripts/sensor_workspace.py build --config configs/arena_rotor_projected_development_v9.json --output sensor-development-v13.zip
```

## Resume development comparisons

For example, the implemented single-factor high-speed screen is:

```powershell
.\.venv\Scripts\python.exe -m control.sensor_matching_screen --output results\sensor_matching_windows --controllers V13 F13 --variants baseline startup_guard gyro_quiet rotor_tau2ms --seeds 3
```

Run the identical command to resume after an interruption. It checks the full
configuration, source hashes, scenarios, seed list, variants, and time limit.
Changed settings require a new folder. Each controller/case/seed runs in its
own subprocess. Completed failures and timeouts remain recorded; they are not
silently retried or removed. Seed ranges 1000–1999 and 2000–2999 are reserved
for main experiments and tuning and rejected by this development runner.
Different screens may use different output folders; never launch two writers
against the same folder. Select a sufficient `--timeout-s` before starting on
a slower computer; the recorded timeout is part of the resume contract.

The fault screen can be run with:

```powershell
.\.venv\Scripts\python.exe -m control.sensor_matching_screen --output results\sensor_faults_windows --controllers V13 --variants baseline --faults nominal baro_drift gnss_outlier gnss_outage --cases gust_lateral_p10_VL --seeds 4
```

## Sensor group and rotor timing checks

For the new group and rotor timing diagnosis, run in separate output folders:

```powershell
.\.venv\Scripts\python.exe -m control.sensor_matching_screen --output results\sensor_groups_windows --controllers V13 F13 --variants quiet_sampled imu_only navigation_only rotor_only baseline --seeds 3 --timeout-s 900
.\.venv\Scripts\python.exe scripts\sensor_rotor_screen.py --output results\rotor_timing_windows --controllers V13 F13 --seeds 3
.\.venv\Scripts\python.exe scripts\sensor_rotor_screen.py --output results\rotor_observer_windows --controllers V13 F13 --seeds 3 --conditions rotor_projected_warm rotor_projected_cold rotor_telemetry_warm rotor_unfiltered_warm
```

These commands use fixed team gains and development seeds. Zero-latency controls
are diagnostic; they do not specify achievable ESC hardware timing. See
[the sensor equations and protocol](SENSOR_FUSION.md#keep-sensor-noise-and-estimator-assumptions-separate).

## Return results

```powershell
Compress-Archive -Path results,sensor_workspace_check.json,sensor-workspace.json -DestinationPath sensor-results-windows.zip
Get-FileHash .\sensor-results-windows.zip -Algorithm SHA256
```

Use a new archive filename for another collection. Send the archive and its
hash through the team's normal channel; no automatic sending or cleanup occurs.
Copying results to the Mac is allowed. Moving an in-progress final tuning job
between computers is not allowed by the team's resume protocol.

## Final tuning remains separate

Before tune7, settle the sensor/estimator configuration and matching protocol,
commit the actual source snapshot, create the agreed release, and fill its
configuration path/hash into the team's current instructions. Retune all five
controllers with the same budget. Run the full environment reproduction and
worker-lifecycle checks on each computer. Only completed and validated records
may fill the five final record hashes. This package does not remove those gates,
invent a release tag, or reuse the old truth-only V13/CPID carryover.
