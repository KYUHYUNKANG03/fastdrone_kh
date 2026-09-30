# V13 source portability verification

The source-manifest patch uses the same relative `/` path names on Windows and
Mac while continuing to hash exact file bytes. It does not change the aircraft,
controller gains, sensor/estimator equations, configurations or acceptance rules.
See [the source-transition note](../../docs/SOURCE_PORTABILITY.md).

## Verification performed on the Mac

- **89 distinct targeted tests passed**, with no failures or skips: 42 runtime,
  tuning and distributed-source tests; 47 preparation, preflight, reproduction,
  envelope and package tests. The JUnit files are retained here.
- Two fresh 12-second V13 reference captures preserve their original numeric
  metrics, verdicts and exact trajectory hashes. Both use development seeds.
- Windows/Posix path-flavour tests agree on source names; altered source bytes
  and duplicate normalized names are still rejected.

| Reference | Speed | Seed | Tracking | Propeller domain | Trajectory vs v9 |
|---|---:|---:|---|---|---|
| Projected rotor observer | 85 m/s | 3 | Pass | Pass | Bit-identical |
| Legacy rotor observer | 20 m/s | 4 | Pass | Fail | Bit-identical |

These are newly executed captures under changed source fingerprints, not old
fixtures with replaced hashes. The capture script checks the declared three-file
runtime transition and refuses a reference if trajectory or metric/verdict
comparison differs. Existing v9 fixtures retain their original hashes.

The compact [verification record](verification.json) links the exact fixture,
test and comparison hashes. Raw run manifests, logs and trajectories remain in
the local checkout and are excluded from Git. The capture script is an archived
one-time transition procedure; use `scripts/sensor_reproduce.py` for ordinary
fresh-folder replays of the published references.

Native Windows execution and tuning-record transfer remain unverified. This
checkpoint does not freeze a sensor/controller matching policy, start final
tuning, use held-out research seeds or establish paper/hardware readiness.
