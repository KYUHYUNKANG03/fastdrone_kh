# Sensor work aligned with the team handoff

Status: development candidate, 2026-09-30. The team controller gains, aircraft
model, allocation rules, and original truth-feedback configurations are unchanged.

The v7 continuation adds a strict resumable development screen, a sensor-inclusive
numerical reproduction check, and a portable checksum-verified source package.
Use [the Windows development commands](SENSOR_WINDOWS_DEVELOPMENT.md) with that
package. The [matching protocol](SENSOR_PAPER_PROTOCOL.md) separates fixed-gain
sensor sensitivity from estimator/controller matching and final held-out testing.
The [v7 report](../results/sensor_readiness_v7/report.md) uses the team's exact
Python 3.13.7 and pinned dependencies; older v6 numerical runs used a different
environment and are not pooled with it. None of these additions freezes the
sensor candidate or authorizes bypassing the existing final-experiment guards.

## Which instructions describe the current experiment?

The user identifies `protkjj/fast-drone`, branch `protkjj/arena-completion`, as
the latest team code. The branch head checked for this handoff is
`6cad076e05a881ff63c6c302ae5a5d177e605739` (2026-09-28). Its
`DISTRIBUTED_RUN.md` section A explicitly waits for a sensor configuration and
configuration hash before `tune-final-7`. It calls for all five controllers to
be retuned, with 120 evaluations each, on four Windows computers.

The supplied PowerShell text instead targets `tune-final-5`, `arena_v2.json`,
and M17/F13 tuning. The current repository explicitly lists tags 5/6 and the
old V13/CPID carryover as sensor-free comparison material. Do not use those
old commands to launch the sensor-inclusive final tuning run. Screenshots
are project context, not commands executed on this computer. The linked Notion
page could not be retrieved through the current connection; no unseen page
content was assumed.

The current `main_experiment` generator produces 726 scheduled runs after
design-region exclusions, **before checking trim availability**. Its nominal
matrix contains 15 reference profiles, five Table 7 baseline scenarios,
17 perturbations across those five scenarios, eight gusts, one high-speed
mission, and the V13 ladder. The screenshot estimating about 2,000 trials
uses a different matrix. Neither estimate includes the proposed repeated
sensor-noise seeds. The old 19.1-hour sequential estimate excludes the new
fusion cost and is not a measured Windows runtime.

## Sensor-to-controller contract

```text
plant truth x=[p,v,q,omega,n]
  -> timestamped measurements, bias/noise/vibration/outliers/dropout/latency
  -> navigation filter + separate rotor observer
  -> optional Table 7 observation-delay buffer
  -> estimated x17 delivered to the same controller interface
  -> command -> unchanged plant
```

The plant and evaluation metrics continue to use truth. The controller receives
the estimate. IMU acceleration is specific force `R(q).T @ (a_world - g_world)`;
the gyro measures body angular velocity plus bias and noise. The fusion layer
uses IMU propagation and timestamped GNSS position/velocity, barometer altitude,
and body-frame magnetic-field updates. Delayed updates replay retained filter
history. Rotor telemetry/prediction remains outside navigation covariance.
The `rpm` configuration values use mechanical **rad/s**, despite the field name.

`control/sensor_binding.py` embeds the complete sensor profile and tuning seed
inside an arena configuration. Thus the normal configuration hash covers the
sensor/estimator settings. The shared `run_trial` inherits it for tuning,
design checks, smoke runs, and distributed main runs. Explicit `--feedback
truth` remains available, and configurations without the block retain truth.
Table 7 state/rotor delays now apply after either feedback path. These are
additional observation delays, separate from per-sensor packet transport.
The inherited delay buffer rounds delays to plant steps (5 ms at 2 ms becomes
4 ms); effective values are recorded.

Tuning records and scenario scores record the sensor seed/profile hash.
Sensor-inclusive tuning also pins runtime Python sources and refuses to resume
or load records with different sources. This runtime hash excludes the final
specification JSON that is filled with record hashes after tuning completes.
Distributed main trials require declared held-out seeds, put the seed in the
trial ID, and preserve the existing resume/merge/hash guards. Shard files also
pin source-file hashes, so an unavailable Git revision cannot silently make
different source trees equivalent. Cross-machine comparisons check sensor
metadata as well as verdicts and numeric tolerances.

## Estimator choices and assumptions

The default remains the legacy 15-error-state ESKF:
`[dp,dv,dtheta,dba,dbg]`. Its separate scalar GNSS/barometer bias learner is
retained to reproduce previous results. The optional `joint_baro` filter adds
one covariance state for pressure-derived altitude bias (16 error states),
while the controller still receives exactly 17 values. This is an estimator
ablation; it is not a claim that the team's original 15D filter already had
joint barometer-bias covariance.

In joint mode, `z_baro = p_z + b_baro + noise`. GNSS updates can adjust the
height/bias correlation. An innovation gate rejects an outlier before changing
either mean or covariance; covariance uses Joseph updates and the attitude
error-coordinate reset. The right-multiplicative attitude convention follows
[Solà's ESKF treatment](https://arxiv.org/abs/1711.02508).

Additional configurable errors include barometer drift, timed GNSS/barometer
offsets, initial navigation errors, uncertain preflight height reference, and
resolved sinusoidal gyro vibration. Vibration is added to the gyro measurement,
not to the plant's angular dynamics. Frequencies at or above IMU Nyquist are
rejected: this model has no analog bandwidth/anti-aliasing stage and cannot
identify a real vibration spectrum from a sampled sine. Noise density uses
`sigma_sample = density / sqrt(sensor_period)`; independent sensor streams
preserve paired draws when another error source changes.

The candidate's 100 preflight barometer readings assume a stationary reference
available before the simulated cruise segment. Exact reference and initially
known navigation state remain simulation assumptions unless error fields are
set. Shared errors in preflight reference height and navigation initialization
are not represented by a cross-covariance model. IMU transport latency gives
stale feedback; only delayed-measurement replay is implemented, not a separate
prediction of the output to the current controller time. These limitations
must remain visible in any sensor boundary claim.

## Concrete candidate files

- `configs/arena_sensor_candidate_v6.json`: `arena_v2.json` plus the complete
  optional joint-bias sensor binding, seed 2001. No controller/plant edits.
- `configs/sensors/joint_baro_candidate_v6.json`: the same profile for explicit
  development runs. Values are assumed simulation specifications, not a
  calibrated BMI270/GNSS/ESC hardware guarantee.
- `configs/main_experiment_sensor_candidate_v6.json`: the same team matrix,
  proposed reserved seeds 1000–1009, all five tune7 record hashes still empty,
  and no old truth-feedback carryover. It remains guarded against final runs.

The ten held-out seeds are a planning proposal, not a sufficiently precise
failure-probability study by themselves. Repeating the whole matrix ten times
would schedule up to 7,260 runs before trim exclusions, plus any separately
declared sensor-quality sweeps. Final sample size should follow the precision
required for the paper's sensor/controller boundary, not the old runtime estimate.
Development uses separate seeds; these held-out seeds have not been used here.

## Run and handoff order

1. Check sensor math, replay, gating, vibration, seed pairing, and both execution
   paths. Retain the truth baseline and record startup/domain failures.
2. Screen the candidate on `arena_v2`, including high-speed cases, then test
   drift, outliers, outages, initial-state errors, and sensor/controller filter
   combinations. Change one factor at a time before interaction tests.
3. Freeze the sensor profile, estimator policy, development/tuning/holdout seed
   sets, and controller-matching protocol. Recompute the config/source hashes.
4. Have the team publish the agreed snapshot/tag and replace the two tune7
   placeholders in `DISTRIBUTED_RUN.md`. Run all five equal-budget tunings;
   perform the documented Windows reproduction and worker checks.
5. Populate final tuning record hashes only from completed, validated records.
   Generate final shards, run and resume them, merge with completeness checks,
   and repeat the prescribed cross-machine subset.
6. Report full-run failures, model-domain excursions, no-trim exclusions,
   estimation errors, control activity, and conditional tracking metrics
   separately. Keep all failed trials in the relevant failure denominator.

Development smoke command (repository root, activated environment):

```sh
python -m control.validation_suite --config configs/arena_sensor_candidate_v6.json --smoke --only-controllers V13 F13 --only-cases gust_lateral_p10_VL gust_lateral_p10_VH --sensor-seed 3 --output results/sensor_candidate_check
```

Use `--feedback truth` for its matched truth reference. Use a new output folder
for each recorded check. The original team files remain available for baseline
reproduction. The candidate must not be described as frozen or paper-ready
merely because its software tests pass.
