# Working on fast-drone-sensor-fusion

The user authorized ongoing commits and pushes to
`https://github.com/leo11dk/fast-drone-sensor-fusion` on 2026-09-30, and explicitly
requested public visibility. This is the publication destination for this
sensor-fusion project; do not push sensor work to the team's upstream repository.

- Complete and verify a coherent change, review the staged diff, then commit and
  push it to this repository during the active task. Verify the remote commit.
  This authorization is not a request for an unattended scheduled job.
- Preserve the aircraft, original controller defaults, and truth-state baseline.
  Make experimental changes explicit in separate configurations.
- Keep tuning, development, and held-out seeds separate. Preserve failed and
  incomplete trials; distinguish tracking from propulsion-domain validity.
- Do not claim development results or software tests establish paper readiness.
- Never commit credentials, virtual environments, caches, local backups, or bulk
  generated traces. Add compact reviewed result summaries intentionally.
- Preserve attribution and upstream provenance. Do not invent a final tuning
  record, source revision, or research result.
- On this Mac, `.venv-paper/bin/python` is the verified local interpreter. Set
  OMP_NUM_THREADS, OPENBLAS_NUM_THREADS, VECLIB_MAXIMUM_THREADS, and MKL_NUM_THREADS
  to 1 before simulation runs. A fresh clone can use `.venv` with the lock file.
- If `/usr/bin/git` invokes an unrelated Xcode license prompt, the installed
  `/Library/Developer/CommandLineTools/usr/bin/git` works directly. Do not accept
  legal agreements on the user's behalf.
