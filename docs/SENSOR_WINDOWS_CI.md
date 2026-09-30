# Hosted Windows sensor check — prepared draft

The reviewable [workflow draft](https://github.com/leo11dk/fast-drone-sensor-fusion/blob/main/docs/examples/windows-sensor-validation.yml) is saved
under `docs/examples/` in Git and is **not active**. The portable source ZIP
includes this guide but omits YAML files; retrieve the draft from GitHub when
needed. No native Windows job has run.
Publishing it to `.github/workflows/windows-sensor-validation.yml` requires the
GitHub CLI `workflow` OAuth scope; the earlier device authorization expired.
Do not treat the draft or Mac checks as Windows evidence.

Once published with that permission, `Windows sensor development validation` checks
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

The workflow is intentionally a documented draft while the additional GitHub
permission is unavailable. Ordinary repository pushes remain authorized. After
publishing the active workflow, dispatch it once and preserve the actual result
before deciding whether it is suitable for the school-machine handoff.

Primary references: [GitHub's Python workflow guide](https://docs.github.com/en/actions/tutorials/build-and-test-code/python),
[hosted runner definitions](https://docs.github.com/en/actions/reference/runners/github-hosted-runners),
[artifact upload action](https://github.com/actions/upload-artifact/tree/v7.0.1).
