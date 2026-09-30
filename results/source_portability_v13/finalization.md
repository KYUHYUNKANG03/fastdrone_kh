# V13 integration and portable package check

The completed [93-task v12 study](../sensor_validation_v12/findings.md) was
published at `06dcb88b91e3330416ca18421dc486ce409e27fa` before integrating the
source-fingerprint patch. Its runtime, results, failures and reference fixtures
remain archived. The merge preserves the original portability commit history.

The updated handoff and package use source revision
`2b54ae2226616de01110381e370bcf3aa1b66bc7`. Flight equations, gains and the v9
engineering sensor configuration remain unchanged. The active v13 references
match all 115 runtime files. The raw reference-capture campaigns were also
copied and byte-checked into the Desktop checkout for preservation.

## Completed checks

- The earlier [portability verification](verification.md) records 89 distinct
  passing tests and two fresh, trajectory-identical numerical captures.
- After integration and updating the pre-tuning checklist, 19 focused tests
  pass with no failures/skips. These repeat a subset of those 89 tests; their
  counts must not be added.
- The source checkout passes all five static preflight checks.
- The portable ZIP contains 312 source/data/document files and a checksum
  manifest, with the v9 development configuration explicitly selected. All
  payload files were verified to be tracked before packaging a clean checkout.
- After extraction into a fresh short path, package byte/dependency/model
  checks pass. All six preflight checks pass there, including package bytes;
  runtime hashes match the source checkout and active reference.

The local archive is `sensor-development-v13.zip` in the Desktop repository.
Its SHA-256 is
`cfb5efd0260d589deb3ddd848f462cb8771a805e61a34ba0050846e46dce19ea`.
The ZIP and raw campaigns are not committed. The
[package receipt](package_receipt.json), [package check](package_check.json),
[relocated preflight](package_preflight.json), [source preflight](final_preflight.json),
[test results](finalization_tests.xml) and [verification index](finalization.json)
are retained here. No extra flight simulation was needed for the relocation check.

## Remaining boundaries

This is a Mac relocation check, not native Windows execution. The
[Windows workflow](../../docs/SENSOR_WINDOWS_CI.md) is a documented draft;
the extra GitHub authorization expired, so no active workflow was published or
job run. School-machine worker and full tuning-record checks remain separate.

Final tuning and held-out experiments have not started. Before final tuning,
declare the treatment of model-domain/settling failures in the objective,
investigate startup and settling behavior on development seeds, and freeze
initialization, sensor/estimator assumptions and controller-matching comparisons.
These checks do not establish paper readiness or a hardware sensor limit.
