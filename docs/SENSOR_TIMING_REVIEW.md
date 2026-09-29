# Sensor feedback timing review — 2026-09-29

The earlier sensor campaigns exposed limits in one GSLQR gust case. Before
extending those results to NMPC-INDI, regression tests found errors in the
measurement pipeline. Historical thresholds need rerunning with these fixes.

| Issue reproduced by a regression test | Corrected behavior |
|---|---|
| Rotor packets were passed directly to the observer before arrival | A separate queue releases telemetry only when arrival time is reached |
| Predictor mode still fused telemetry | Predictor mode ignores all RPM packets |
| Motor prediction used an extra old command | An endpoint update uses the command applied during the preceding interval |
| Rotor low-pass gain used plant dt at all sensor rates | Gain uses the elapsed time between accepted rotor samples |
| A first telemetry packet was blended with a fabricated zero measurement | The first arrived reading initializes the measured rotor state |
| IMU noise used plant dt instead of sensor period | Density is divided by the square root of the sensor period |
| Bias random walk used plant dt even for a slower IMU | Bias increments use elapsed IMU sample time; no walk occurs before t=0 |
| GNSS/barometer updates between IMU samples used an older state | Propagate state/covariance to the measurement timestamp before updating |
| Ideal IMU inherited nonzero bias random walk | Both random-walk terms are explicitly zero in the ideal profile |
| The first saved estimated state was a copy of truth | Save the actual controller feedback state at t=0 |
| A failed campaign rerun could read an older successful output | Only directories created by the current subprocess are eligible |

## Measurement and feedback equations used here

The plant state is `[p(3), v(3), q(4), omega(3), n(4)]`. The navigation
filter uses a 15-dimensional error state; the four rotor states are handled
separately. This is not a 17-dimensional navigation filter.

- Accelerometer: `a_m = R(q)^T*(a_world-g_world) + b_a + eta_a`.
- Gyroscope: `omega_m = omega + b_g + eta_g`.
- GNSS: `p_m = p+b_p+eta_p`, `v_m = v+b_v+eta_v`.
- Barometer: `z_m = p_z+b_baro+eta_baro`.
- Magnetometer: `m_m = R(q)^T*m_world+b_mag+eta_mag`.
- Rotor telemetry: `n_m = n+b_n+eta_n`, with `n` in rad/s.

Every packet has `t_arrival = t_sample + latency`. The filter can only use
arrived packets. Delayed navigation updates replay the history from the sample
time; delayed rotor telemetry is filtered when it arrives and is not replayed
through the navigation filter.

For the simulation's continuous-white-noise convention, sample standard
deviation is `D/sqrt(T_sensor)`, while a bias increment is
`B*sqrt(delta_t)*N(0,I)`. These parameters must be converted and calibrated
from real sensor specifications; an output data rate alone does not establish
the hardware's noise bandwidth.

The controller receives
`[p_hat,v_hat,q_hat,omega_m-b_g_hat,n_hat]`. In telemetry mode,
`n_hat += (1-exp(-delta_t_sample/tau))*(n_m-n_hat)`; in predictor mode,
`n_hat += (1-exp(-delta_t/tau))*(u_applied-n_hat)`.
The plant always integrates from its truth state.

## Verification and remaining assumptions

The GSLQR truth-feedback trajectory on `gust_lateral_p10_VL` is bit-identical
before and after these changes. The saved check is
`results/sensor_timing_v2/truth_baseline_check.json`.

The regression suite covers arrival causality, predictor isolation, sample-rate
noise scaling, bias drift through loss, stale RPM rejection, in-order versus
delayed replay equivalence, and measurement updates between IMU ticks.

Known assumptions remain: exact initial navigation trim state, plant-tick sensor
scheduling, no analog sensor bandwidth model, and no navigation extrapolation
from the last event to current controller time. Barometer-bias adaptation and
ESKF covariance consistency also need separate validation before treating any
screening result as a hardware requirement. Real RPM telemetry must be converted
to the plant's rad/s units.

The old five-seed tables are preserved in
`history/SENSOR_CAMPAIGN_STATUS_20260929_PRE_TIMING.md`; they must not be mixed
with results from this corrected pipeline.
