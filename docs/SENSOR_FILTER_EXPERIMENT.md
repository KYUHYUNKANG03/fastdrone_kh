# INDI filtering and startup diagnosis — 2026-09-29

The simulation now supports explicit INDI cutoff experiments, F13 S1 filtering,
and an optional guard against allocating from unavailable rotor telemetry.
The controller's existing gyro-derived angular-acceleration filter is tunable;
the ESKF's IMU input and NMPC's observed body rate are unchanged. Sensor profiles
remain fixed during the filter comparison.

The bundled arena configuration retains its original behavior: 50 Hz, V13 S1,
F13 S0, startup guard off. New settings are written into separate experiment
configs and recorded from the actual constructed controller in every manifest.

All **25 new trials** completed with zero optimizer failures and zero timeouts.
All passed tracking; only the three truth-feedback trials passed the complete
acceptance check. Every one of the 22 sensor-feedback runs still violated the
assumed propulsion-domain range through low rotor RPM. The focused suite passes
**93 tests**. All nine repeated baselines (three truth and six sensor runs) are
bit-identical to their previously saved trajectories.

## Measured filter tradeoff

Means over seeds 1 and 2, lateral gust, matched GNSS severity 0.25:

| Controller / variant | Filter | Command slew [rad/s²] | Velocity RMSE [m/s] | Outside propulsion domain [%] |
|---|---|---:|---:|---:|
| V13 baseline | S1, 50 Hz | 301472 | 0.408 | 5.192 |
| V13 lower cutoff | S1, 25 Hz | 157869 | 0.408 | 4.950 |
| V13 lower cutoff | S1, 12.5 Hz | 81636 | 0.408 | 5.383 |
| V13 guard + filter | S1, 25 Hz | 157869 | 0.408 | 4.867 |
| F13 baseline | S0, 50 Hz | 238849 | 0.407 | 2.750 |
| F13 alignment only | S1, 50 Hz | 301215 | 0.407 | 3.158 |
| F13 lower cutoff | S1, 25 Hz | 157558 | 0.407 | 3.133 |
| F13 lower cutoff | S1, 12.5 Hz | 80982 | 0.407 | 3.350 |
| F13 guard + filter | S1, 25 Hz | 157560 | 0.407 | 2.783 |

Relative to their original defaults, 25 Hz reduces command slew by **47.6% for
V13** and **34.0% for F13**; 12.5 Hz reduces it by 72.9% and 66.1%. These are
command-variation measurements, not reductions in tracking error. All tracking
verdicts remain successful, but lower noise does not consistently reduce time
outside the propeller model's assumed range.

F13's S1/50 Hz control isolates the alignment change. At that cutoff it increases
chatter compared with S0; switching S1 from 50 to 25 Hz then reduces it by about
48%. The S0-to-S1 change therefore must not be counted as a noise-reduction
improvement by itself. The combined guard/25 Hz option removes unavailable-RPM
allocation in all six guarded trials (including the two standalone guard runs),
but still does not make the sensor runs fully pass.

The evidence supports keeping these as explicit experimental settings. It does
not establish a new default cutoff or a sensor specification limit.

## Startup finding

At initialization the nominal telemetry observer has no RPM packet and returns
four zero estimates. The plant itself starts at cruise trim, near 11,300–11,650
mechanical RPM. The first packet is sampled at 0 ms and arrives at 4 ms.

| Time | Original V13 path | Guarded V13 path |
|---:|---|---|
| 0 ms | Initialization fallback | Rotor-unavailable fallback |
| 2 ms | A1 allocation from zero rotor estimates; commands all four motors to zero | Rotor-unavailable fallback; finite nonzero model-based command |
| 4 ms | Normal allocation after first telemetry arrival | Initialize derivative history from available feedback; fallback for this sample |
| 6 ms | Normal allocation | Normal allocation |

The guard changes availability handling, not the estimated RPM values. It does
not receive the true rotor state. In the tested nominal profile it removes the
all-zero command at 2 ms. F13's original A0 solver already falls back when the
zero-rotor effectiveness matrix is singular, so it does not exhibit the same
all-zero allocation.

This fix does not remove the later startup transient. In the V13 seed-1 nominal
GNSS run, estimated height increases by approximately 0.134 m at 40 ms while
true height changes negligibly. The NMPC thrust request drops from about 10.0 N
to 5.2 N. GNSS first arrives at 120 ms, when the height error increases to about
0.30 m and the thrust request falls to about 3.6 N. Similar navigation corrections
and thrust changes persist with the guard. Their timing points to navigation
update transients as a separate issue; a sensor/update ablation is needed to
isolate their contributions quantitatively.

With the guard, the minimum startup rotor speed rises from 6,878 to 6,992 RPM
for V13 and from 7,740 to 7,839 RPM for F13. Both remain below the propulsion
map's assumed 10,000 RPM lower bound. The full-trial outside-domain fraction
changes from 1.333% to 1.350% for V13 and from 0.983% to 1.000% for F13. This
is a correction to the missing-feedback behavior, not a demonstrated cure for
the model-validity failures.

![Startup telemetry, motor commands, navigation error, and requested thrust](../results/sensor_filter_followup_v3/startup_transient.png)

## Filter equations and delay

For the existing S1 path, at each control interval T:

\[
\alpha=1-e^{-2\pi f_cT},\qquad
\widehat{\dot\omega}^{f}_k=(1-\alpha)\widehat{\dot\omega}^{f}_{k-1}
+\alpha\frac{\hat\omega_k-\hat\omega_{k-1}}{T},
\]

\[
f_{mid,k}=\tfrac12(f_k+f_{k-1}),\qquad
f^{f}_k=(1-\alpha)f^{f}_{k-1}+\alpha f_{mid,k}.
\]

Here `f` contains the four rotor-force estimates. Initialization sets the force
filter to its first available midpoint value, not zero. The optional guard
defers initialization until feedback is available. The derivative filter starts
at zero. S0 uses `alpha=T/(T+1/(2*pi*fc))` on angular acceleration and no matching
rotor-force LPF, so an S0/S1 comparison also changes the discrete coefficient.

For constant T, the S1 low-pass transfer is
`L(z)=alpha/(1-(1-alpha)z^-1)`. At angular frequency w, the gyro path normalized
by an ideal differentiator is `L(exp(j*w*T))*(1-exp(-j*w*T))/(j*w*T)`;
the rotor-force path is `L(exp(j*w*T))*(1+exp(-j*w*T))/2`. These have equal phase
below Nyquist, although their magnitudes differ. Lower cutoffs suppress more
gyro noise and increase phase delay.

At the 2 ms simulation interval, the analytically calculated equivalent phase
delay at **5 Hz** is:

| S1 cutoff | Controller-path equivalent delay | Expected filtered gyro white-noise RMS norm |
|---:|---:|---:|
| 50 Hz | 3.277 ms | 20.634 rad/s² |
| 25 Hz | 6.336 ms | 11.225 rad/s² |
| 12.5 Hz | 12.138 ms | 5.846 rad/s² |

The noise calculation assumes independent per-axis gyro samples with density
0.002 rad/s/sqrt(Hz), and excludes bias-estimation error. These are analytical
controller-filter quantities, not measured end-to-end delays. ESC transport,
the rotor-observer filter, and estimator timing add other effects. INDI still
linearizes at the current observed rotor state and adds its increment there;
matching the force/acceleration filters is not complete actuator-state time
alignment.

## Experiment artifacts

- [Filter and startup report](../results/sensor_filter_followup_v3/filter_report.md)
- [Filter sweep checkpoint](../results/sensor_filter_v3/experiment.json)
- [Startup-guard checkpoint](../results/sensor_startup_v3/experiment.json)
- [Combined guard and 25 Hz checkpoint](../results/sensor_filter_combined_v3/experiment.json)
- [Truth regression campaign](../results/sensor_filter_truth_v3/campaign.json)
- [Analytical delay values](../results/sensor_filter_followup_v3/filter_delay_analysis.json)
- [Verification results](../results/sensor_filter_followup_v3/verification.json)
- [Saved-data verification script](../results/sensor_filter_followup_v3/verify.py)
- [Configuration and reproduction commands](SENSOR_FUSION.md#indi-filter-and-startup-experiments)

The filter sweep uses lateral gust `gust_lateral_p10_VL`, nominal gyro noise,
matched GNSS level 0.25, and sensor seeds 1 and 2. The standalone startup test
uses matched GNSS level 0 and seed 1. The combined guard/25 Hz test uses the same
degraded GNSS and seeds as the filter sweep. These are exploratory comparisons,
not hardware requirements or flight qualification. No cutoff is promoted to the
default automatically.

The next work is to isolate barometer and GNSS update transients, evaluate
estimator initialization/covariance consistency, and test the useful filter
candidates on vertical gusts, acceleration/deceleration, and more seeds. The
startup guard does not detect inaccurate/stale telemetry after initialization;
that needs a separate policy. These cutoffs were tested with a 500 Hz IMU and
500 Hz inner loop; different sampling rates and sensor bandwidth require a new
comparison. M17 is outside this experiment.
