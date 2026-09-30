# Hosted Windows sensor check

The [workflow](https://github.com/leo11dk/fast-drone-sensor-fusion/blob/main/.github/workflows/windows-sensor-validation.yml)
is published after the user approved GitHub CLI's additional `workflow` scope.
The first native Windows run is pending. A workflow file or passing Mac checks
do not establish Windows verification. The portable source ZIP omits GitHub
workflow files; run this check from the repository's Actions page.

`Windows sensor development validation` checks
the portable runtime on a GitHub-hosted `windows-2022` x64 machine. The dedicated
`validation/windows-sensors` branch also triggers it on push. It does not launch
the 93-task study or final tuning. Official actions are pinned to commit hashes,
Python is 3.13.7, dependencies use `requirements-lock.txt`, and numerical thread
counts are one. The job has a 45-minute limit.

It runs the same 89 targeted tests selected for the Mac portability check, then
static preflight and the projected/legacy V13 numerical references. The short
tuning-path unit tests are software checks, not final tuning records. Reference
comparison keeps `rtol=1e-3` and `atol=1e-9`, exact recorded verdicts, and a separate
trajectory-bit-identity field. Do not loosen tolerances to make a job green.

Artifacts retain test XML, audit JSON, comparisons, logs and raw traces even
when a step fails. Download them before the 30-day retention period expires.
Record the actual run URL, source commit, environment and outcome before claiming
Windows verification. A workflow file alone is not a passed check.

This hosted machine does not certify the four school computers. The school's
dependency setup, full tuning-record reproduction, forced-parent-exit/orphan
worker procedure and compute-capacity checks still need to run there. Native
hosted Windows numerical agreement and school-machine readiness are separate.

Use the manual dispatch on a reviewed source revision and preserve the actual
result before deciding whether it is suitable for the school-machine handoff.
The earlier draft and expired authorization recorded in the Mac package receipt
describe that earlier checkpoint, not the current workflow status.

Primary references: [GitHub's Python workflow guide](https://docs.github.com/en/actions/tutorials/build-and-test-code/python),
[hosted runner definitions](https://docs.github.com/en/actions/reference/runners/github-hosted-runners),
[artifact upload action](https://github.com/actions/upload-artifact/tree/v7.0.1),
[expression context availability](https://docs.github.com/en/actions/reference/workflows-and-actions/contexts#context-availability).
