# Sensor campaign status — 2026-09-30, experiment preparation

## Current checkpoint

V11 adds a [pre-run workflow](SENSOR_PRE_RUN.md) with static environment/model
checks, a source/configuration-bound plan, a single-writer resume mechanism,
and reports that preserve pending cases and failures. The default 93-task plan
has not been executed. It covers all five controllers and a fixed-covariance
GNSS noise/latency factorial. The runtime and v9 reproduction fixtures remain
unchanged. Final paper readiness remains false; the old v6 paper candidate is
not silently promoted or overwritten.

## Latest executed development evidence

V10 adds [paired truth/fusion screens](SENSOR_ENVELOPE_SCREEN.md) while keeping
the v9 aircraft, controllers and estimator runtime unchanged. The physical motor
mismatch cases now change the plant time constant, keeping the observer at 20 ms.
Telemetry-outage diagnostics count allocations made while feedback is stale;
the controller fault policy is unchanged. Read [the recorded findings](../results/sensor_readiness_v10/findings.md)
and [all paired outcomes](../results/sensor_readiness_v10/report.md) before choosing
the next sensor/interface changes. Final tuning and held-out evaluation remain unstarted.

The checkpoint records 30 paired trials plus eight adaptive V13 startup trials,
with no execution timeouts. Forty-four targeted tests pass. GNSS-only errors
reproduce a low-speed propulsion-domain crossing while tracking still passes;
both controllers pass the isolated rotor outage with a nominal motor model.
All results use development seed 3. Physical motor mismatch and initial-estimate
error expose additional failures, so these results do not establish a full envelope.

## Previous v9 checkpoint

V9 implements timestamp-aware rotor projection and an independent steady-trim
measurement-history startup. Both controllers pass the nominal 85 m/s case with
the original 4 ms rotor latency. Cold-start and motor-model-mismatch conditions
retain their failures. The full source/seed/configuration contracts and outcomes
are in [the v9 report](../results/sensor_readiness_v9/report.md); see
[the observer assumptions](ROTOR_OBSERVER.md) before interpreting sensor limits.

V9 passed 277 distinct tests, with eight long legacy tests skipped. The default
reproduction fixture exercises the new observer. A separate fresh legacy fixture
and local replay remain bit-identical to v8. Controller defaults, aircraft, main
configuration and held-out seeds remain unchanged; final tuning has not started.

## Previous v8 checkpoint

V8 adds fixed-covariance group controls and explicit rotor timing comparisons.
At 85 m/s, both controllers pass quiet/IMU-only/navigation-only feedback but
fail rotor-only and nominal full feedback. Both pass with full nominal sensor
noise when extra rotor smoothing and transport latency are removed. That is a
diagnostic zero-latency condition, not a hardware specification. Delayed cases
also start without an available rotor measurement. See
[the interpretation and next observer boundary](SENSOR_GROUP_DIAGNOSIS.md) and
[all v8 outcomes](../results/sensor_readiness_v8/report.md).

V8 passed 250 distinct software tests (eight long legacy tests skipped). A fresh
low-speed reference and local replay are bit-identical to v7. Controller gains,
aircraft, truth configurations and held-out seeds are unchanged. Final tuning
remains unstarted.

## Previous v7 checkpoint

V7 establishes the team's exact Python 3.13.7 environment with pinned packages,
and adds strict source/configuration resume checks to a paired development
screen. It tests 85 m/s V13/F13 feedback/interface variants and 20 m/s V13
barometer drift, GNSS outlier, and GNSS outage cases. All eleven noisy
high-speed variants fail tracking and model-domain checks; the four low-speed
fault/baseline trials complete and pass tracking but fail model-domain checks.
The two matched high-speed truth references pass. No controller defaults or
gains were changed. Final tuning and held-out runs remain unstarted.

The [v7 report](../results/sensor_readiness_v7/report.md) contains the current
outcomes, ideal-sensor references, estimation/feedback diagnostics, and software
verification. The [Windows development handoff](SENSOR_WINDOWS_DEVELOPMENT.md)
provides source-checksum verification, sensor-inclusive numerical replay,
resumable screens, and result collection without deleting a checkout. The
[paper protocol](SENSOR_PAPER_PROTOCOL.md) defines the distinct sensitivity,
estimator-matching, and controller-matching comparisons.

## V6 team integration checkpoint

The latest team handoff confirms that all five controllers wait for the sensor
configuration before `tune-final-7`. The pasted `tune-final-5` Windows recipe and
old truth-feedback carryover are superseded. The current generator schedules
726 runs before trim checks and before adding repeated sensor seeds.

The sensor profile can now be embedded in the arena configuration and inherited
by tuning and distributed main runs. Sensor seeds are separated between tuning
and held-out main runs, recorded in trial IDs, and included in provenance.
Source/hash guards cover resume and merging. Table 7 observation delays now
apply to the sensor path as well. Added error inputs include gyro vibration,
barometer drift, GNSS/barometer outliers, and initialization/reference errors.

The optional joint barometer-bias ESKF retains covariance between navigation and
pressure bias and gates updates before changing either. The legacy 15D path is
still the default for old profiles. Stationary consistency and fault/replay unit
checks pass; that is not a flight-performance guarantee.

With the corrected `arena_v2` model, both V13/F13 truth references pass the
85 m/s lateral gust. The candidate sensor profile fails high-speed tracking
for development seed 3 (F13 stops early); both 20 m/s runs pass tracking but
violate the assumed propulsion domain. Do not run the reserved holdout seeds
or present these smoke checks as final paper evidence.

See [the current report](../results/joint_fusion_followup_v6/report.md) and
[the concrete handoff](SENSOR_TEAM_HANDOFF.md). The candidate main specification
keeps the five tuning hashes empty until the sensor freeze and actual tuning.

## Earlier preflight calibration implementation

Preflight barometer calibration now has an opt-in independent random stream,
so changing the averaging count preserves all in-flight sensor noise draws.
The default shared stream retains earlier behavior. The experiment runner
compares first-flight-reading initialization with 1-, 25-, and 100-reading
preflight averages, plus a 100-reading case with ongoing GNSS-based bias learning
disabled. All controller settings and estimator covariance remain fixed.

The [preflight calibration study](PREFLIGHT_CALIBRATION_STUDY.md) records the
sampling model, replay behavior, two-seed V13/F13 outcomes, and limitations.
The focused suite passes **125 tests**, including legacy-stream regression and
packet equality across averaging counts. Four repeated baselines match the
earlier trajectories bit for bit. Detailed results are in the
[generated comparison](../results/preflight_followup_v5/preflight_report.md).

Preflight calibration survives the initial delayed GNSS replay without
reinitializing its bias. That does not prevent navigation-state corrections or
later GNSS-based bias learning. Improving calibration accuracy does not by
itself guarantee improved closed-loop tracking or model-domain validity.

All **20 calibration trials** completed without optimizer failures or timeouts
and passed tracking; **none passed the propulsion-domain check**. A 100-reading
average reduces the mean peak startup height error from about 0.212 m to
0.092 m, but with bias learning enabled it worsens full-trial mean altitude
RMSE by 11–13%. Freezing the same calibration improves altitude RMSE by about
46–47% relative to baseline, while low-RPM excursions remain. No new default
is selected. The next fusion experiment should address uncertainty and gating
in continuing bias learning and include reference error and pressure drift.

## Earlier navigation update diagnosis

Optional navigation traces now record each barometer, GNSS, and magnetometer
packet's first-processing innovation, covariance, NIS, acceptance, sample-time
state injection, and arrival timing. Separate batch records retain the total
current-state change after propagation and replay. The report links these to
the next actual NMPC solve and requested thrust. Tracing defaults to off.

The [navigation diagnosis](NAVIGATION_UPDATE_DIAGNOSIS.md) identifies a concrete
startup interaction: the first noisy barometer sample becomes its bias
reference, and delayed GNSS replay later changes that reference. V13 seed 1
shows a 13.4 cm barometer height injection at 40 ms and a thrust request falling
from 10.0 N to 5.2 N at that solve. This is timing evidence, with individual
sample-time corrections kept separate from total arrival-batch changes.

The new comparison fixes the original controller settings and estimator
covariance while changing measurement noise, latency, or bias. A +1 m constant
barometer bias is absorbed by initialization; a +1 m GNSS height bias fails
tracking for both tested seeds. See the [generated comparison](../results/navigation_update_followup_v4/navigation_report.md)
for the full outcomes. Four repeated baselines are bit-identical to the saved
earlier trajectories, and **109 focused tests pass**. Controller gains,
estimator covariance, and initialization policy remain unchanged.

All **24 trials** completed 12 seconds without optimizer failures or timeouts.
**22 passed tracking; four passed both tracking and model domain.** The four
full passes are the two-seed V13/F13 runs with both barometer and GNSS noise
removed, while IMU, magnetometer, and RPM noise remain nominal. All other runs
have low-RPM map-domain excursions. This isolates a navigation-noise effect;
it does not establish a usable hardware specification or a fusion fix.

The next comparison should use the existing optional preflight barometer
averaging installed in the replay baseline, then assess calibration uncertainty
and GNSS bias-learning behavior before expanding sensor specifications.

## Earlier INDI filter experiments

INDI cutoff and startup availability handling are now explicit experiment
options. V13/F13 accept `indi_cutoff_hz` and `rotor_startup_guard`; F13 also
supports the existing S1 midpoint/force-filter path. The bundled defaults remain
50 Hz, V13 S1, F13 S0, and startup guard off. Sensor noise and ESKF tuning stay
fixed during the filter experiments.

The added telemetry/INDI traces identified an all-zero V13 motor command at
2 ms, before the first RPM packet arrives at 4 ms. The optional guard removes
that command by using the model fallback until rotor feedback is available.
It does not resolve the later low-RPM excursions associated with navigation
corrections and changing NMPC thrust requests.

The [filter experiment report](SENSOR_FILTER_EXPERIMENT.md) explains the
implementation, delay equations, startup trace, outcomes, and remaining limits.
The [generated comparison](../results/sensor_filter_followup_v3/filter_report.md)
contains per-variant results. The focused sensor, filter, allocation, F13, and
validation suite passes **93 tests**.

All 25 new trials completed without optimizer failures or timeouts and passed
tracking. All 22 sensor trials still violated the assumed propulsion-domain
range; the three truth trials passed both checks. Nine repeated baseline
trajectories match the earlier results bit-for-bit. At 25 Hz, command slew fell
47.6% for V13 and 34.0% for F13 relative to their defaults, while tracking error
changed little. A 12.5 Hz cutoff reduced chatter further but did not improve
the domain result consistently. These remain experimental settings.

## Earlier feedback study

The latest experiments use corrected sensor timing and sample-rate noise scaling.
That study contains 36 matched controller-screen trials across lateral and vertical
gusts, plus eight feedback-ablation trials. All 44 completed with zero optimizer
failures and zero timeouts. Its earlier focused regression suite passed 37 tests.
The earlier five-seed GSLQR grids are retained in
[the historical status](history/SENSOR_CAMPAIGN_STATUS_20260929_PRE_TIMING.md).
Their numerical boundaries do **not** transfer to this revised pipeline.

## What changed

RPM telemetry now obeys arrival time. Predictor mode ignores telemetry. IMU
noise is scaled by sensor period, bias drift by elapsed sample time, and
navigation measurements update the state at their sample timestamp. The ideal
profile has no hidden bias random walk. The saved estimated trajectory starts
with the actual feedback state rather than a truth-state copy.

See [the timing review](SENSOR_TIMING_REVIEW.md) for equations, units,
regression cases, and remaining assumptions.

## Matched controller screens

The bounded controller screens compare GSLQR, V13, and F13 on
`gust_lateral_p10_VL` and `gust_vertical_m5_VL`, with the following runs per case:

- Truth feedback and ideal sensor feedback: one deterministic run each.
- Nominal IMU and matched nominal GNSS: sensor seeds 1 and 2.
- Nominal IMU and GNSS severity 0.25: the same two seeds.

Within each case, all points keep the same plant, controller settings, and acceptance
rules. Both GNSS levels match estimator covariance to the generated measurement
sigma. This differs from the raw nominal profile's default estimator tuning.

| GNSS level | Rate [Hz] | Position sigma [m] | Velocity sigma [m/s] | Latency [ms] | Dropout |
|---:|---:|---:|---:|---:|---:|
| 0 | 10 | 0.8 | 0.15 | 120 | 0% |
| 0.25 | 8.75 | 1.35 | 0.3125 | 177.5 | 2.5% |

The generated [lateral-gust report](../results/sensor_timing_v2/controller_screen.md)
contains all outcomes, solver failures, and measured state-estimation errors.
Its [JSON checkpoint](../results/sensor_timing_v2/controller_screen.json)
records whether the campaign is complete. A
[comparison figure](../results/sensor_timing_v2/controller_screen.png) shows
the errors and observed seed ranges. The matching
[vertical-gust report](../results/sensor_vertical_v2/controller_screen.md) and
[figure](../results/sensor_vertical_v2/controller_screen.png) contain the second
case, with the same sensor profiles, controllers, and seeds.

The lateral-gust GSLQR truth-feedback trajectory is bit-identical before and after the
changes, verified by the saved trajectory hash.

Each case has 18 completed trials. The following counts apply separately to
**both** cases; matching pass counts do not imply identical trajectories.

| Feedback | GSLQR tracking / full pass | V13 tracking / full pass | F13 tracking / full pass |
|---|---|---|---|
| Truth | 1/1 · 1/1 | 1/1 · 1/1 | 1/1 · 1/1 |
| Ideal sensors | 1/1 · 1/1 | 1/1 · 1/1 | 1/1 · 1/1 |
| Matched GNSS level 0 | 2/2 · 2/2 | 2/2 · 0/2 | 2/2 · 0/2 |
| Matched GNSS level 0.25 | 1/2 · 1/2 | 2/2 · 0/2 | 2/2 · 0/2 |

For the lateral gust at GNSS level 0.25 the mean velocity RMSE is 0.445 m/s for GSLQR, 0.408 m/s
for V13, and 0.407 m/s for F13. All nine domain failures in this screen are due
to rotor speed falling below the propeller map's assumed lower RPM bound.
This is not proof that the physical aircraft loses control; it limits what
the current propulsion model can validate. For the vertical gust, the respective
mean velocity RMSE values are 0.389, 0.319, and 0.319 m/s. GSLQR seed 2 fails
gust recovery and the propulsion-domain check at GNSS level 0.25; seed 1 passes.

## Feedback-path findings

Four single-factor V13 runs varied gyro noise, RPM noise, RPM latency, and the
rotor-observer filter. Removing gyro white noise reduced motor-command slew by
97.3% in the nominal-GNSS seed-1 lateral case, but the startup model-domain
excursion remained. The three rotor-feedback changes did not remove it either.

Four additional V13/F13 runs tested quarter nominal gyro-noise density with
degraded GNSS and seeds 1 and 2. Filtered angular-acceleration error fell by
about 75% and command slew by about 74.5%. Tracking error changed little, and
all four runs still violated the propulsion-domain check. This is a lower-noise
sensor-profile experiment; no additional gyro filtering is implemented yet.

See [the feedback follow-up](SENSOR_FEEDBACK_FOLLOWUP.md) for the equations and
comparisons, [per-run diagnostics](../results/sensor_feedback_followup_v2/feedback_diagnostics.md)
for all 44 traces, and [verification metadata](../results/sensor_feedback_followup_v2/verification.json)
for source-hash and controller-setting checks. Across these traces, every
propulsion-domain excursion was due to rotor speed below the assumed
10,000 mechanical RPM map bound; other domain conditions were not violated.

## Interpretation and next work

Keep tracking success separate from propulsion-model validity. The nominal
V13/F13 runs show short excursions below the propeller map's assumed minimum
RPM; some occur after startup. Successful trajectory tracking does not make
those excursions validated propulsion data.

These experiments cover two gust cases. They do not establish a hardware sensor
tolerance or a probability of controller failure. The model still assumes a
known initial navigation state, plant-tick sampling, and simplified sensor
bandwidth. It holds the last navigation event state between updates.

The next useful work is to:

1. Isolate the barometer and GNSS update transients seen after startup, and
   check estimator initialization, covariance consistency, and barometer-bias learning before
   treating the GNSS transition as a hardware limit.
2. Extend filter comparisons to vertical gusts, acceleration,
   deceleration, and more noise seeds.
3. Validate the actual ESC telemetry rate, latency, and rad/s conversion, and
   investigate matching the rotor-command baseline timing as well as the force filter.
4. Include M17 with a separately budgeted run; it has not been retested in this
   timing revision.

Reproduction commands are in [SENSOR_FUSION.md](SENSOR_FUSION.md).
