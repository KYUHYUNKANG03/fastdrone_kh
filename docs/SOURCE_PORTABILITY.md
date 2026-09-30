# Source fingerprints across operating systems

V13 changes the relative names used by runtime, tuning and arena source
manifests. A file named `control/example.py` previously appeared with native
backslashes on Windows and slashes on Mac. A direct dictionary comparison
could reject identical source bytes solely because of that spelling.

`control/source_manifest.py` now emits sorted relative POSIX names on both
platforms. SHA-256 still covers the exact file bytes: line-ending changes,
modified files, missing files and extra runtime files remain differences.
Duplicate source names are rejected. `.gitattributes` specifies LF checkout
bytes for the covered source/configuration formats.

This patch changes no plant equations, controller gains, estimator equations,
sensor distributions or trajectory acceptance criteria. Nevertheless, the
source guards correctly see a changed runtime: two existing files changed and
one helper was added. Historical plans, tuning records and reference fixtures
remain bound to their original sources. Do not rewrite their hashes to pass
the new guards or resume an old experiment under the new runtime.

## Numerical reference transition

The archived v9 fixtures remain unchanged. The new default projected-observer
reference is `scripts/data/sensor_reproduction_v13.json`; the separate legacy
reference is `scripts/data/sensor_legacy_reproduction_v13.json`. Each is captured
by running its original controller, case, configuration and development seed
under the new source tree. The capture requires matching recorded verdicts,
numeric metrics and exact trajectory hash before writing a new fixture.

The capture script and receipts in `results/source_portability_v13/` distinguish
this declared source transition from a normal replay under identical sources.
The legacy reference retains its model-domain failure. Reproduction agreement
does not turn that failure into a successful flight result.

The integration/GNSS study used the old runtime, available at source commit
`4d30e025679c56b3acb72edb112d67c706b2cbeb`. Keep its source-bound plan and raw
records with that revision when reanalyzing or reproducing the study. Create a
new output folder and plan for any new study on the portable runtime.

## What this verifies

Path-flavour tests exercise Windows and Posix path objects on the Mac, and the
existing tuning/distributed tests check source mismatch rejection. These are
software checks, not native Windows execution. The school machines still need
the [Windows numerical and worker checks](SENSOR_WINDOWS_DEVELOPMENT.md) and the
team's full tuning-record reproduction procedure before final tuning.
