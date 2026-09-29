# Sensor feedback follow-up — 2026-09-29

Reducing gyro noise substantially reduces motor-command chatter in the tested
V13 and F13 controllers. It does not by itself resolve the propulsion-model
validity failures or materially improve trajectory-tracking error. The new
tools diagnose this distinction without changing the truth baseline, controller
settings, estimator implementation, or acceptance rules.

This follow-up adds a vertical-gust controller screen, four single-factor
diagnostic runs, and four lower-gyro-noise runs. Full traces and generated sensor
profiles are retained with each campaign. The focused regression suite passes
37 tests.

All 26 new trials completed with zero optimizer failures and zero timeouts.
Together with the previous 18 lateral-gust runs, the diagnostics cover 44
traces. Saved source hashes confirm unchanged core sensor, estimator, plant,
controller, and validation code across these campaigns and the Desktop copy;
controller settings also match across campaigns.

## Vertical-gust comparison

The 18-run screen uses `gust_vertical_m5_VL`. All truth and ideal-sensor runs
pass tracking and model-domain checks. Under matched nominal GNSS, GSLQR passes
both checks in both seeds; V13/F13 pass tracking in both seeds but fail the
domain check. At GNSS severity 0.25, GSLQR seed 1 passes while seed 2 fails gust
recovery and the domain check. V13/F13 again pass tracking in both seeds and
fail the domain check.

These pass-count patterns match the lateral-gust screen. The mean velocity
RMSE under degraded GNSS is 0.389 m/s for GSLQR and 0.319 m/s for each of V13
and F13. Across all 44 traces, only the low-RPM condition accounts for
propulsion-domain excursions; maximum RPM, advance ratio, and reverse-flow
conditions are not violated.

## What the feedback equations explain

The navigation estimate feeds NMPC, while the INDI correction uses measured body
rate and estimated rotor speed. With bias-corrected gyro feedback,

\[
\hat\omega_k=\omega_k+e_{\omega,k},\qquad
e_{\omega,k}=b_{g,k}-\hat b_{g,k}+\eta_{g,k}.
\]

The differentiated measurement contributes

\[
e_{\dot\omega,k}^{raw}
=\frac{e_{\omega,k}-e_{\omega,k-1}}{\Delta t_k},\qquad
e_{\dot\omega,k}^{f}
=(1-\alpha_k)e_{\dot\omega,k-1}^{f}
+\alpha_k e_{\dot\omega,k}^{raw}.
\]

At the current 50 Hz cutoff, V13 uses
`alpha = 1-exp(-2*pi*50*dt)` and F13 uses
`alpha = dt/(dt+1/(2*pi*50))`. The implementation clips differentiation/filter
intervals to [0.0001, 0.2] s; these trials use 0.002 s. A locally linear INDI
correction has the form

\[
\Delta u\simeq G^{\dagger}(\nu-\widehat{\dot\omega}^{f}),
\qquad
\delta\Delta u\simeq-G^{\dagger}e_{\dot\omega}^{f},
\]

before allocation, actuator limits, and other feedback errors. Here `G` is the
local control-effectiveness matrix; this equation describes the noise path, not
the complete implementation. Sensor noise is interpreted as a change in measured
angular acceleration and therefore creates corrective actuator commands.

Under this simulator's white-noise convention, the nominal gyro sample sigma is
`0.002/sqrt(0.002) = 0.0447 rad/s` per axis. Independent sample noise then gives
raw derivative sigma `sqrt(2)*0.0447/0.002 = 31.6 rad/s²` per axis. The
three-axis RMS norm is approximately 54.8 rad/s² before the controller filter.
This amplification explains why modest rate noise can produce large command
slew even when position/velocity tracking looks acceptable.

The diagnostic reconstructs estimated-minus-true differentiated body rate on
the **same trajectory**. It does not subtract two different closed-loop runs.
Command slew is the RMS norm of the four commanded rotor accelerations, in
rad/s², not measured rotor acceleration. Both diagnostic RMS values exclude the
first 0.5 s; all acceptance checks still include startup.

## Isolating possible causes

The first experiment uses V13, lateral gust `gust_lateral_p10_VL`, matched GNSS
level 0, and sensor seed 1. Each row changes exactly one field from that profile.
The gyro-density field also sets the corresponding ESKF process-noise term;
these tests do not hold that covariance fixed when changing gyro density.

| Profile change | Filtered angular-acceleration error [rad/s²] | Command slew [rad/s²] | Minimum mechanical RPM | Outside domain [%] |
|---|---:|---:|---:|---:|
| Baseline | 20.643 | 296299 | 6878 | 1.333 |
| Zero gyro white-noise density | 0.138 | 7887 | 7303 | 1.133 |
| Zero RPM telemetry noise | 20.643 | 296315 | 6892 | 1.333 |
| Zero RPM transport latency | 20.643 | 294993 | 6891 | 1.350 |
| Rotor-observer time constant 20 → 2 ms | 20.643 | 296846 | 6734 | 1.300 |

All five runs pass tracking and fail the existing propulsion-domain check. For
these particular runs, every low-RPM sample is within the first 0.5 s. Removing
gyro white noise reduces command slew by 97.3%, but a startup excursion remains.
The three rotor-feedback changes tested individually do not resolve the
excursion. This does not establish that rotor estimation is irrelevant; it
rules out those particular standalone changes as sufficient fixes here.

## A lower-noise candidate under degraded GNSS

The second experiment changes gyro white-noise density from 0.002 to 0.0005
rad/s/sqrt(Hz), with V13/F13, lateral gust, GNSS severity 0.25, and sensor seeds
1 and 2. The ESKF process covariance follows that configured density. This is a
sensor-specification experiment; no additional filtering algorithm is enabled.

| Controller / gyro density | Filtered angular-acceleration error [rad/s²] | Command slew [rad/s²] | Velocity RMSE [m/s] | Outside domain [%] |
|---|---:|---:|---:|---:|
| V13 / nominal | 20.709 | 301472 | 0.4078 | 5.192 |
| V13 / quarter | 5.178 | 76765 | 0.4087 | 3.767 |
| F13 / nominal | 16.702 | 238849 | 0.4072 | 2.750 |
| F13 / quarter | 4.177 | 60676 | 0.4080 | 1.867 |

Entries are means over two seeds. The candidate reduces filtered acceleration
error by about 75% and command slew by about 74.5%. All four candidate runs pass
tracking, but all four retain propulsion-domain violations, including samples
after 0.5 s. The violating condition is rotor speed below the map's **assumed
10,000 mechanical RPM minimum**. This is a model-validity boundary, not an
established physical minimum RPM or evidence of loss of control.

The similar tracking errors show that improving the gyro alone does not remove
the other feedback errors under this GNSS profile. Two seeds cannot establish
a hardware noise requirement or controller failure probability.

## Artifacts and next experiments

- [Vertical-gust comparison](../results/sensor_vertical_v2/controller_screen.md)
- [Single-factor campaign checkpoint](../results/sensor_feedback_ablation_v2/ablation.json)
- [Lower-noise candidate checkpoint](../results/sensor_gyro_candidate_v2/ablation.json)
- [Per-run feedback diagnostics](../results/sensor_feedback_followup_v2/feedback_diagnostics.md)
- [Reproduction commands](SENSOR_FUSION.md#feedback-path-diagnosis)

The next implementation experiment should expose an explicit, optional gyro
signal-conditioning path and sweep its cutoff with matched timing on the INDI
rotor/acceleration paths. A lower cutoff trades noise reduction against phase
lag; this must be tested in the closed loop rather than inferred from this
lower-noise sensor experiment. Preserve the current settings as the reference.

Separately, instrument the startup rotor estimate, actual rotor state, and
allocation commands to explain the residual transient. Validate or extend the
propeller map using supporting data before treating out-of-range trajectories
as validated results. Do not relax acceptance rules to make these runs pass.

Remaining coverage includes uncertain navigation initialization, estimator
covariance consistency, sensor bandwidth/aliasing, realistic ESC sampling and
unit conversion, acceleration/deceleration cases, more seeds, and M17. These
runs still start navigation at the known trim state and sample sensors on plant
ticks. They provide simulator evidence, not flight qualification.
