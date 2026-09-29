# Timestamp-aware rotor feedback — v9

The optional `telemetry_predictor` projects delayed rotor telemetry to the
current controller time using recorded applied commands and a declared nominal
motor model. Legacy `telemetry` and command-only `predictor` defaults remain.
No controller gain, plant equation or truth-feedback mode changes.

## Measurement correction and command history

A packet contains rotor speed, sample time and arrival time. It is unavailable
before arrival. The newest usable measurement anchors the estimate at its sample
time. Each recorded constant-command interval from there to now applies

\[
\hat n(t+\Delta t)=u+(\hat n(t)-u)\exp(-\Delta t/\hat\tau_m).
\]

Speeds and commands are mechanical **rad/s**, including fields named `rpm`.
The command passed to an endpoint acted over the preceding interval. Intervals
are clipped to the sample timestamp. Later commands never affect earlier
estimates. The observer receives no true rotor state or actual plant parameters.

This is direct telemetry correction with unit gain plus model prediction, not a
Kalman filter. It applies no extra telemetry low-pass filter. Measurement noise,
bias and motor-model error remain; there is no rotor covariance or guaranteed
error bound. Old/duplicate packets cannot replace a newer anchor. Invalid packets
and measurements outside retained command history are rejected. Packets that
cannot fit history upon arrival are not queued indefinitely. Between arrivals,
prediction continues using commands. Diagnostics count accepted and rejected packets.

```json
{
  "source": "telemetry_predictor",
  "motor_tau_s": 0.02,
  "history_s": 0.5,
  "max_age_s": 0.05,
  "prehistory_s": 0.1
}
```

The motor time constant, history and maximum ready age must be explicitly positive
and finite. History must cover ready age and nominal telemetry latency. `ready`
requires a usable measurement with sample age at most `max_age_s`; it does not
guarantee accuracy. Expiry does not stop prediction. Controllers with the existing
`rotor_startup_guard=false` do not automatically switch control law when readiness
expires. Readiness is logged each control tick; fault-policy changes must be explicit.

Legacy telemetry `tau_s` is extra measurement smoothing; command-only prediction
uses that field as a motor time constant. The new mode uses `motor_tau_s` and
ignores `tau_s`. Its nominal 20 ms assumption is not read from perturbed plant truth.

## Causal startup history

With `prehistory_s=0` (default), the estimate starts from the declared `initial_rpm`
prior or zero. Delayed telemetry is initially unavailable. This cold case is retained.

A positive duration supplies a **steady-initial-trim rotor prehistory** before
flight time zero. The sensor layer generates noisy, biased, possibly missing
measurements at negative times with the configured arrival latency. The observer
receives those packets and the known command maintaining the simulated trim,
not the true rotor state used to generate them. Packets in transit at zero remain
queued. A history shorter than latency does not falsely make telemetry ready.

This models an already flying aircraft entering the trial with telemetry running.
It does not simulate takeoff, full aircraft/navigation warm-up, or measured ESC
startup. State the steady-trim assumption in paper methods. Metrics cover the
complete trial from zero; failures are not cropped from cold runs. Both controllers
receive the same initialization policy in a comparison.

A separate seeded stream generates past rotor measurements, preserving all five
flight streams and independent barometer calibration. The existing exact initial
navigation-state assumption is unchanged and remains a separate limitation.

## Reproduce the comparisons

`configs/arena_rotor_projected_development_v9.json` changes only the sensor binding
relative to `arena_v2.json`. It is an explicit development configuration; the
existing v6 candidate and final-tuning guards remain unchanged.

```sh
python scripts/sensor_rotor_screen.py --output results/observer_diagnosis --controllers V13 F13 --seeds 3 --conditions rotor_projected_warm rotor_projected_cold rotor_telemetry_warm rotor_unfiltered_warm
python scripts/sensor_rotor_screen.py --output results/observer_repeats --controllers V13 F13 --seeds 4 5 --conditions rotor_projected_warm
python scripts/sensor_rotor_screen.py --output results/observer_model_mismatch --controllers V13 F13 --seeds 3 --conditions rotor_projected_warm_tau10ms rotor_projected_warm_tau40ms
```

`projected_warm` versus `projected_cold` isolates prehistory. `telemetry_warm`
retains 20 ms measurement smoothing; `unfiltered_warm` removes that smoothing
while preserving the same 4 ms delay and prehistory. Comparing the latter with
`projected_warm` tests projection against an unfiltered, delayed observation.
The 10/40 ms variants change only the observer model; the physical motor retains
its 20 ms response. Controller gains and other sensor errors stay fixed.

See [all v9 outcomes](../results/sensor_readiness_v9/report.md) and
[verification](../results/sensor_readiness_v9/verification.json). These small
development comparisons do not establish hardware limits or failure probabilities.
Wider maneuvers, rate/latency/dropout sweeps, plant mismatch, equal-budget matching
and held-out evaluation remain necessary.
