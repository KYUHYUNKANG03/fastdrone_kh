# Sensor–controller matching: development protocol

Paper topic: **High-Speed Multicopter Control Architecture Robust to Disturbances
and Sensor Uncertainty: NMPC–INDI Interface Separation and Sensor–Controller
Matching.** This is a working protocol, not a registered final experiment.

## Three questions, with separate comparisons

1. **Feedback sensitivity:** freeze the controller gains, architecture, plant,
   mission, acceptance limits, and estimator policy; vary one sensor property.
   This estimates sensitivity of that specific closed loop.
2. **Estimator matching:** use the same simulated packets and controller settings
   to compare estimator assumptions/filtering. Declare whether covariance tracks
   the actual noise level or remains nominal. Otherwise better fusion and better
   sensors become confounded.
3. **Controller matching:** compare a declared matched filter/interface/controller
   variant under the same sensor distribution and equal tuning budget. Use fresh
   held-out seeds after selecting that variant. A setting selected from a screen
   cannot be evaluated as if it were chosen independently of that screen.

Truth feedback remains a separate reference. Ideal sampled sensors are also
useful because their sampling and interface can differ from instantaneous truth.
The aircraft dynamics and original controller defaults are preserved.

## Feedback variables and error model

The controller consumes `x_hat = [p_hat(3), v_hat(3), q_hat(4), omega_hat(3), n_hat(4)]`.
The plant integrates truth `x`. Only scoring/debugging may read truth directly.
Navigation error covariance uses a three-component attitude error, not additive
noise on four quaternion components. Sensor measurement models include:

```text
gyro:         y_g = omega_body + b_g + noise + resolved_vibration
accelerometer:y_a = R(q)^T (a_world - g_world) + b_a + noise
GNSS:         y_p = p + b_p + noise; y_v = v + b_v + noise
barometer:    y_h = p_z + b_h + pressure_drift + noise
magnetometer: y_m = R(q)^T magnetic_field_world + b_m + noise
rotor:        y_n = n + b_n + noise
```

Packets carry sample and arrival times. The navigation filter applies delayed
measurements at their sample times and replays retained history. Packet loss,
latency, persistent biases, random noise, and outliers are separate parameters.
Plant wind/force/moment disturbances change physical dynamics and are different
from sensor corruption. Both may be tested jointly after isolated checks.

The implemented white-noise convention is per-sample standard deviation
`noise_density / sqrt(sample_period)`; bias random walk increments scale with
`sqrt(elapsed_sample_time)`. A datasheet's one-sided noise density and real analog
bandwidth must be converted consistently before claiming a hardware match.
The current simulation has no analog anti-aliasing model. Its deterministic
vibration tones must lie below the sampled Nyquist frequency.

The rotor state/telemetry fields are mechanical **rad/s**, despite the `rpm`
configuration key. Convert actual ESC electrical RPM using motor pole pairs,
then convert mechanical RPM using `2*pi/60`; record the exact hardware convention.
Telemetry transport and an observer LPF introduce separate delays. They are not
the motor's physical response time or automatically canceled by the INDI filter.

The [specification-basis review](SENSOR_SPECIFICATION_BASIS.md) compares the
active engineering profile with primary manufacturer references and derives
the bandwidth/variance conversion. Its hardware mapping remains provisional;
the current development profile has not been replaced with datasheet numbers.

## Simulation assumptions still requiring qualification

The candidate initializes navigation at the cruise truth unless explicit initial
errors are configured. Preflight barometer averaging assumes an available
stationary height reference. Rotor initialization is a model estimate until
telemetry arrives. IMU output is not separately predicted forward to compensate
for transport latency. Sensor rates use plant-step scheduling. These assumptions
must accompany any claimed sensor requirement.

The 15-error-state ESKF remains available; `joint_baro` has 16 error states after
adding correlated barometer bias. Neither changes the 17-value controller
interface. Rotor uncertainty remains outside that navigation covariance.

## Required outcomes

For each planned case, retain completion/timeout/solver-failure status, tracking
verdict, and propulsion-domain verdict separately. Report no-trim exclusions
and planned-but-missing trials explicitly. Do not describe an out-of-map
simulation as proof of physical aircraft failure.

Report estimation errors against truth (position, velocity, attitude angle,
body rate, rotor speed), innovation rejection/NIS diagnostics where logged,
tracking errors, saturation, command variation, solver failures, and recovery.
Partial-trajectory RMSE is conditional on the observed duration; compare it
alongside completion, not as evidence that early termination improved accuracy.

For repeated trials use paired sensor seeds across controller variants, with
independent per-sensor streams. Resuming must preserve the configuration and
source fingerprint. For discrete faults also declare onset, duration, and size.

## Freeze order

1. Finish development screens on the corrected aircraft and characterize high-
   speed failure modes. Do not silently tune to held-out main seeds.
2. Select a provisional sensor/estimator and controller-matching design. Record
   each changed setting and the evidence used to choose it.
3. Freeze configuration/source, equal tuning budgets, tuning seed policy, and
   held-out scenarios/seeds before final comparisons. The current inline binding
   uses seed 2001 for tuning; broader tuning-noise coverage would require a
   declared extension, not an undocumented change to objective evaluations.
4. Retune all five controllers through the team's normal runner; complete the
   full design and cross-machine checks and populate actual record hashes.
5. Run the held-out main matrix and a separately specified sensor-quality grid.
   The current ten held-out seeds are only a proposal. Select the count from
   required uncertainty in the failure-rate/boundary claim and available compute.

The target is a supported operating envelope for each sensor/controller pairing,
with uncertainty and model limitations, rather than one universal sensor cutoff
at which NMPC–INDI becomes “useless.”

## Precision planning for the eventual held-out grid

Choose the failure event, operating cell and confidence rule before examining
held-out results. For illustration, if independent identically distributed
trials in one fixed cell have zero failures in `n` runs, the exact one-sided
95% binomial upper bound is `1 - 0.05^(1/n)`. It follows directly by solving
`P(zero failures | p) = (1-p)^n = 0.05`.

| Zero-failure runs in one cell | One-sided 95% upper failure-probability bound |
|---:|---:|
| 3 | 63.2% |
| 10 | 25.9% |
| 59 | 4.95% |
| 299 | 0.997% |

These examples are sample-size planning calculations, not confidence claims
about adaptively selected development results. A two-sided interval, nonzero
failures, dependent repetitions, multiple-cell coverage or sequential stopping
requires the corresponding analysis rule. Repeating deterministic controls
does not increase the effective sample count. Paired controller comparisons
reuse seeds across variants; different operating conditions must not be pooled
as if they were repeated trials of one failure probability.

Track execution incompleteness, tracking failure and model-domain exclusion
separately. A domain crossing establishes a limit of the simulation's supported
model, not the probability of a real aircraft failure. The final sensor-quality
grid and repeat count are still unselected; reserved seeds remain unused.
