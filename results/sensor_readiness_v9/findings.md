# What the v9 comparisons show

Both V13 and F13 pass the full 12-second, 85 m/s lateral-gust case with the
timestamp-aware observer, its declared 20 ms motor model, and 0.1 s of noisy
steady-trim rotor measurement history. They pass at development seeds 3, 4 and 5,
with nominal sensor errors and the original 4 ms telemetry delay. No controller
gains or aircraft parameters were retuned. This is a bounded development result,
not a failure probability or a final hardware requirement.

| Condition | V13 | F13 |
|---|---|---|
| Projected, warm, nominal model (seeds 3/4/5) | Full pass 3/3 | Full pass 3/3 |
| Projected, cold (seed 3) | Tracking passes; domain fails | Tracking/domain fail |
| Legacy 20 ms smoothing, warm (seed 3) | Tracking/domain fail | Stops at 3.92 s; fails |
| Unfiltered 4 ms delayed telemetry, warm (seed 3) | Tracking/domain fail | Full pass |
| Observer assumes 10 ms motor response, warm (seed 3) | Tracking passes; domain fails | Stops at 1.38 s; fails |
| Observer assumes 40 ms motor response, warm (seed 3) | Tracking/domain fail | Full pass |

The physical motor retains its nominal 20 ms response in both mismatch tests.
The 10/40 ms changes affect only the observer assumption. Full pass means both
tracking acceptance and propulsion-model domain validity; it is not a stability
proof. One accepted F13 unfiltered run still contains an optimizer failure, which
remains in [the complete numerical report](report.md). Short-run RMSE covers only
the recorded prefix and cannot be ranked against full-duration RMSE.

These results support separating startup policy, measurement smoothing, telemetry
delay and actuator-model assumptions. Projection is useful for V13 in this case;
F13 also tolerates a simpler unfiltered delayed observation with the same startup
history. Their different responses to the 40 ms observer model are relevant to
sensor–controller matching. They do not establish general superiority of either
architecture or show that motor-model uncertainty is solved.

The warm condition uses past noisy sensor packets and known applied commands,
not a direct true-rotor-state initialization. It assumes the aircraft was at its
initial trim before the trial and retains the existing exact initial navigation
state assumption. It is not a takeoff or full preflight simulation. The observer
has no rotor covariance or finite-outlier gate; its stale-data flag does not
automatically change the controller law with the current default settings.

Next validation should cover other maneuvers and speeds, telemetry rate/latency/
loss, initialization errors, and perturbed plant motor response while retaining
a fixed observer assumption. Define and compare an explicit stale-feedback policy
before treating the system as robust to outages. Equal-budget matching, native
Windows checks and held-out paper experiments remain outstanding.

[Observer implementation and assumptions](../../docs/ROTOR_OBSERVER.md) ·
[Verification and reproduction](verification.json) · [Recorded contracts and diagnostics](outcomes.json)
