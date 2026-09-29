# High-speed sensor diagnosis — v8

This checkpoint isolates the feedback paths before changing the team's gains.
It uses the corrected aircraft, the same joint 16D navigation estimator, the
85 m/s lateral-gust scenario, a 12-second trial, and development seed 3.
The [recorded report](../results/sensor_readiness_v8/report.md) includes every
completed or failed comparison and its configuration/source hashes.

## What the controlled comparison establishes

Both V13 and F13 pass tracking and the propulsion-domain check with quiet
sampled sensors. Both also pass when only nominal IMU errors are restored,
and when only nominal GNSS/barometer/magnetometer errors are restored.
Both fail when only the nominal rotor telemetry/observer path is restored.
The complete nominal sensor configuration fails as before. These are paired
development results for one scenario/seed, not failure probabilities.

The rotor-only V13 run completes 12 seconds without optimizer failures but has
velocity RMSE 27.51 m/s. The F13 rotor-only run stops at 2.36 seconds; its
prefix RMSE must not be ranked against a full-duration run. The rotor path
can therefore reproduce the failure without nominal navigation noise. That
does not prove that other sensor groups or their interactions never matter.

Both controllers pass with all nominal sensor noise when telemetry transport
delay and additional rotor measurement smoothing are removed (`rotor_direct`).
This is a diagnostic configuration, not an achievable zero-latency ESC claim.
The accompanying `rotor_unfiltered` and `rotor_no_latency` trials separate
the two changes; inspect their individual outcomes in the report.

## Why navigation fusion alone cannot resolve this

The navigation estimator supplies position, velocity, attitude and body rate.
Rotor speed is supplied by a separate observer. In telemetry mode the existing
observer low-passes the delivered measurement and then holds that value.
It does not propagate delayed telemetry to the current control time.

Schematically, the INDI calculation uses

\[
u_k=\hat n_k+\Delta n_k,
\qquad
\Delta n_k\approx G_n(\hat n_k,\hat v_k,\hat q_k)^{-1}
\bigl(\nu_k-\hat\nu_k\bigr).
\]

The actual implementation includes allocation choices, clipping and filtering;
this equation only shows where rotor feedback enters. An old or smoothed
rotor estimate affects both the command baseline and control effectiveness.
Improving the navigation covariance does not make that rotor estimate current.

The candidate's 20 ms `rotor_observer.tau_s` is additional telemetry smoothing.
It is distinct from the plant's 20 ms motor time constant. The equal numerical
values do not mean the measurement filter correctly models the actuator.
V13's S1 force/acceleration filtering also does not, by itself, compensate this
upstream transport delay and rotor smoothing.

## Startup is part of the experiment

With 4 ms telemetry latency, the rotor observer initially has no arrived rotor
measurement. The controller can allocate before that first measurement arrives;
the saved startup diagnostics expose those commands. With zero latency the
first measurement is available immediately. Thus the latency comparison changes
startup availability as well as continuing delay. It cannot establish a physical
4 ms sensor limit or prove that steady-flight delay is the only cause.

Keep the existing cold-start case as a regression test. Add an explicitly
declared, causal pre-roll/warm-start condition before interpreting latency as
a steady-flight sensor requirement. A warm start must obtain its estimate from
past measurements or a declared model prior, not copy the current true rotor
state into the controller.

## Next implementation and validation boundary

The next observer candidate should correct an estimate at the telemetry sample
timestamp and propagate it through recorded applied commands to the current
time. For the declared first-order actuator model over a constant-command interval:

\[
\hat n(t+\Delta t)=u+\bigl(\hat n(t)-u\bigr)e^{-\Delta t/\hat\tau_m}.
\]

The observer must use a declared nominal motor model, never hidden perturbed
plant parameters. Tests should cover delayed/out-of-order packets, dropouts,
time-constant mismatch, bounded history and startup availability. This proposed
observer is **not implemented by this checkpoint**; the recorded runs use the
existing telemetry observer.

After that change, repeat the paired comparisons at additional development
seeds, then extend to vertical gusts and acceleration/braking. Keep the exact
truth-feedback references and distinguish tracking, solver, estimator and
propulsion-domain failures. A passing single-seed diagnosis is not a basis for
freezing a final sensor profile, choosing hardware limits, or filling the final
tuning hashes. The equal-budget five-controller tuning and held-out protocol
remain separate stages.

## Software and reproduction

The optional estimator IMU-density overrides hold process covariance fixed while
sensor noise changes. Defaults still follow the sensor as before. The group
controls also preserve nominal preflight covariance. Unit tests check these
contracts and preserve paired sensor random streams.

The v8 low-speed reference is a fresh executed trial. It is bit-identical to the
v7 reference on the pinned Mac environment. Old evidence and source hashes are
preserved; the active reproduction command uses the new v8 fixture. Historical
reports must be regenerated with their own recorded runtime revision. Native
Windows verification and final tuning remain outstanding.
