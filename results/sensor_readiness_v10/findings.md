# V10: paired envelope validation and navigation-startup diagnosis

**Development evidence with fixed gains and seed 3. This is not final tuning,
a measured hardware limit, or a statistical reliability claim.**

The paired screen extends v9 beyond one nominal 85 m/s lateral gust. It contains
30 distinct trials: 14 truth controls and 16 fused-feedback trials across V13/F13.
Eight additional V13 sensor-group trials follow up the observed low-speed failure.
Those eight are adaptive diagnosis, not held-out confirmation. The physics,
controllers, estimator runtime, original acceptance thresholds and v9 reproduction
fixtures remain unchanged.

All 38 trials produced recorded outcomes without execution timeouts. Three
terminated early after consecutive optimizer failures; their results and traces
are retained. The deterministic full-profile replay is a repeatability check,
not an additional independent statistical sample.

See [all paired outcomes](report.md), [startup group outcomes](startup_groups/report.md),
[verification](verification.json), and [commands/assumptions](../../docs/SENSOR_ENVELOPE_SCREEN.md).

## What the broader cases establish

Both controllers pass the 85 m/s vertical-gust case under truth and fused
feedback. At 20 m/s lateral gust, acceleration to 85 m/s, and the +1 m altitude
step at 20 m/s, both truth controls pass, while both fused runs pass tracking but
leave the assumed propulsion-model domain. Thus tracking completion and model
validity give different answers; neither should replace the other in the paper.

In the low-speed lateral gust, minimum actual rotor speed reaches approximately
7,988 RPM for V13 with full sensor feedback. The nominal model's assumed lower
working limit is 10,000 RPM. Most of that run's low-RPM samples occur in the first
0.5 s, before the gust begins at 3 s. Rotor telemetry is ready throughout. The
[trace figure](low_speed_domain.png) shows the corresponding truth controls and
both fused-feedback trajectories. These are model-domain exits, not proof of a
physical aircraft failure or a general statement that the controller is unusable.

## GNSS is a concrete startup sensitivity to investigate

The first five follow-up trials remove generated errors and restore one sensor
group at a time. Nominal estimator process/measurement covariance and preflight
covariance assumptions remain fixed. The next three restore individual navigation
sensors on that same quiet sampled control. These comparisons do not provide truth
states to the controller.

| V13, 20 m/s lateral gust, seed 3 | Tracking | Model domain | Minimum actual rotor speed [RPM] |
|---|---|---|---:|
| Quiet sampled measurements | Pass | Pass | 11,071 |
| IMU errors only | Pass | Pass | 10,475 |
| Navigation errors only | Pass | Fail | 8,330 |
| Rotor errors only | Pass | Pass | 11,046 |
| Full sensor profile | Pass | Fail | 7,988 |
| GNSS errors only | Pass | Fail | 9,451 |
| Barometer errors only | Pass | Pass | 10,392 |
| Magnetometer errors only | Pass | Pass | 11,059 |

The GNSS-only trial restores the candidate's 0.8 m position noise standard
deviation per axis, 0.15 m/s velocity noise standard deviation per axis, and 120 ms latency. Its
first model-domain crossing is at 0.138 s; the full profile crosses at 0.136 s.
This timing follows the first delayed GNSS arrival at 0.12 s. The ablation shows
that the GNSS block alone is sufficient to reproduce a crossing **in this case**.
It does not establish that GNSS is the only possible cause, nor separate its
noise from its transport delay. Other sensor combinations and seeds remain open.

The full-profile follow-up is trajectory-bit-identical to the paired screen's
V13 low-speed fused run. The diagnostic is repeatable on the reference machine.

The next targeted work is to separate GNSS noise and delay, repeat development
seeds, and compare the present navigation startup with a declared causal
measurement warm-up. A tighter prior must have a physical initialization basis;
merely reducing covariance because the simulator happens to initialize at truth
would not establish a realistic improvement.

## Physical motor mismatch exposes a separate baseline limit

Unlike v9's observer-assumption experiments, these trials change the **physical
motor time constant** to 10 or 40 ms. The observer and controller assumptions
remain nominal. Reports reconstruct the physical parameters from each trial and
verify their hashes; the observer stays at 20 ms.

| Actual motor tau | V13 truth / fusion | F13 truth / fusion |
|---|---|---|
| 10 ms | Truth passes tracking but fails domain; fusion fails both | Both pass tracking but fail domain |
| 40 ms | Truth stops at 3.22 s; fusion runs 12 s but fails tracking and domain | Truth stops at 1.86 s; fusion stops at 1.02 s |

All three early stops in this table follow five consecutive optimizer failures.
The complete solver counts, acceptance failures and partial-run durations remain
in the recorded outcomes. These comparisons do not justify blaming every motor
mismatch failure on sensing: some fail with perfect state feedback as well.
They sample two perturbation points, not a measured motor-tolerance boundary.

## Availability is not an accuracy or fallback guarantee

The rotor outage removes samples during [3.5, 3.7) s. Both controllers continue
for the full 12 s and pass tracking and domain. Each has feedback not ready for
0.154 s, during which it makes 77 allocations; the maximum sample age is 0.204 s. Prediction
continues with the nominal motor model, and readiness recovers on new arrivals.
The unchanged controller has no new stale-feedback fallback policy.

This single-factor case must not be generalized to simultaneous motor-model error
and telemetry loss. The combined conditions are untested here. Likewise, V13's
explicit initial-estimate error passes tracking but leaves the propulsion domain
for five samples at the start; those samples remain failures under the original
criterion. F13's initial-error trial completes 12 s but fails both tracking and
domain, with nine optimizer failures and velocity RMSE 5.126 m/s. Its truth
control passes. The initial-error bundle changes position, velocity and attitude
together; it does not isolate which component drives F13's sensitivity.

## Reproducibility and remaining gates

- The paired runner records a deterministic truth control once per controller and
  physical scenario. Multiple sensor conditions can reference it without turning
  it into multiple independent observations.
- Configurations, seeds, cases, runtime hashes, driver hashes, profile identities,
  actual physical parameters, failures and interrupted-run records are retained.
  Reports reject mismatched input/trace provenance and duplicate trials.
- Forty-four targeted tests cover scenario isolation, actual motor-factor mapping,
  seed guards, partial resume, timeout retention, trace/metadata checks and the
  new navigation screen. The outage test confirms exactly 100 missing rotor
  samples at 500 Hz while preserving subsequent noise draws and the other sensors.
- The first launch was rejected by the arena's configuration check before flight
  simulation: the draft driver omitted required mission definitions. All 30
  rejected attempts remain locally archived separately. The corrected driver
  retains the full arena and selects cases through the existing runner; a regression
  test covers the arena facts. Rejected setup attempts are not flight outcomes.
- All eight paired conditions use one development seed. Braking, the integrated
  mission, broader sensor rate/latency grids, other controllers, native Windows
  reproduction, equal-budget final tuning and held-out trials remain necessary.

The useful next milestone is a repeated, causally explained navigation-startup
comparison, followed by combined sensor/model-error tests in a declared operating
region. The main-paper configuration and final tuning hashes remain unchanged.
