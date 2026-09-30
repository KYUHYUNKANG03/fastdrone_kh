# V11 preparation verification

This checkpoint prepares development experiments; it does not add new flight
campaign results or release the paper configuration.

- **365 distinct tests passed** across the broad check and targeted follow-ups;
  17 heavy legacy checks were skipped with `ARENA_QUICK=1`. One strict expected
  failure is the team's existing CPID 85 m/s out-of-design-region trim case.
- The initial broad check found two outdated fairness-test mocks. Their sensor
  arguments and truth-feedback command assertion were repaired. Both pass on
  rerun. Original and follow-up JUnit reports are retained; the JSON indexes the
  latest outcome per test, not an invented single all-green invocation.
- Static preflight passes on this Mac: pinned dependencies, single-thread
  settings, controller-model coefficients, arena facts and active reference
  source compatibility. `paper_ready` remains **false**.
- All **114 runtime source files** match the v9 fixture exactly. No aircraft,
  controller gain, estimator equation, truth baseline or acceptance threshold
  changed. Historical fixtures retain their recorded source hashes.
- Reprocessing all **30 v10 paired envelope trials** produces exactly the same
  result records. Reporting now explicitly handles zero-step/nonfinite failures
  and rejects malformed or mismatched inputs.
- The saved plan has **93 pending tasks**, zero executed: 45 all-controller
  integration tasks and 48 GNSS factorial tasks sharing two truth controls.
  The first provisional plan was replaced by a newly named final development
  plan after portability guards changed; the unused provisional directory was
  retained locally.

The portable archive identifies the v9 development config, verifies every
included source/data byte, and excludes environments, credentials, Git metadata
and bulk results. Its receipt and extracted-workspace checks are recorded
alongside this report. Native Windows numerical replay, process lifecycle and
final cross-OS tuning-record transfer remain unverified. No numerical replay
was rerun for this checkpoint; runtime source compatibility is a narrower claim.

Local prepared plan: `results/prepared_sensor_validation_v11_final/plan.json`.
The committed compact plan summary contains task IDs, input hashes and truth
pairings; regenerate the full plan with the [pre-run commands](../../docs/SENSOR_PRE_RUN.md).
The source/configuration contract refuses modified resumes, and a lock prevents
simultaneous writers. Development scripts normalize Windows path separators
without changing byte hashes; LF checkout rules protect those bytes.

Final tuning remains gated on the startup diagnosis, supported sensor/controller
envelope, declared initialization/hardware assumptions, finalized policy and
five valid tuning records. The old v6 paper spec has deliberately not been
promoted. These preparation checks are not a hardware reliability claim.
