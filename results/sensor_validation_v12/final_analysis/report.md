# Sensor integration and GNSS startup development study

Fixed gains/covariances, development seeds only. Three seeds are not a hardware reliability estimate. Quiet repetitions may have identical trajectories. No startup exclusion from acceptance.

Counts retain all planned cases and executed failures. Execution errors and early simulation stops are different outcomes. The detailed validated report keeps duration, partial RMSE, stop reasons, source hashes, estimation errors and model-domain diagnostics.

| Stage | Controller | Case | Condition | Recorded / planned | Track / domain / full pass | Early stops | Unique trajectories | Acceptance / domain failures unpenalized by legacy rule |
|---|---|---|---|---:|---|---:|---:|---:|
| gnss_startup | F13 | gust_lateral_p10_VL | gnss_full_noise0_delay0 | 3 / 3 | 3 / 2 / 2 | 0 | 3 | 1 / 1 |
| gnss_startup | F13 | gust_lateral_p10_VL | gnss_full_noise0_delay1 | 3 / 3 | 3 / 2 / 2 | 0 | 3 | 1 / 1 |
| gnss_startup | F13 | gust_lateral_p10_VL | gnss_full_noise1_delay0 | 3 / 3 | 3 / 0 / 0 | 0 | 3 | 3 / 3 |
| gnss_startup | F13 | gust_lateral_p10_VL | gnss_full_noise1_delay1 | 3 / 3 | 3 / 0 / 0 | 0 | 3 | 3 / 3 |
| gnss_startup | F13 | gust_lateral_p10_VL | gnss_isolated_noise0_delay0 | 3 / 3 | 3 / 3 / 3 | 0 | 1 | 0 / 0 |
| gnss_startup | F13 | gust_lateral_p10_VL | gnss_isolated_noise0_delay1 | 3 / 3 | 3 / 3 / 3 | 0 | 1 | 0 / 0 |
| gnss_startup | F13 | gust_lateral_p10_VL | gnss_isolated_noise1_delay0 | 3 / 3 | 3 / 2 / 2 | 0 | 3 | 1 / 1 |
| gnss_startup | F13 | gust_lateral_p10_VL | gnss_isolated_noise1_delay1 | 3 / 3 | 3 / 2 / 2 | 0 | 3 | 1 / 1 |
| gnss_startup | V13 | gust_lateral_p10_VL | gnss_full_noise0_delay0 | 3 / 3 | 3 / 1 / 1 | 0 | 3 | 2 / 2 |
| gnss_startup | V13 | gust_lateral_p10_VL | gnss_full_noise0_delay1 | 3 / 3 | 3 / 1 / 1 | 0 | 3 | 2 / 2 |
| gnss_startup | V13 | gust_lateral_p10_VL | gnss_full_noise1_delay0 | 3 / 3 | 3 / 0 / 0 | 0 | 3 | 3 / 3 |
| gnss_startup | V13 | gust_lateral_p10_VL | gnss_full_noise1_delay1 | 3 / 3 | 3 / 0 / 0 | 0 | 3 | 3 / 3 |
| gnss_startup | V13 | gust_lateral_p10_VL | gnss_isolated_noise0_delay0 | 3 / 3 | 3 / 3 / 3 | 0 | 1 | 0 / 0 |
| gnss_startup | V13 | gust_lateral_p10_VL | gnss_isolated_noise0_delay1 | 3 / 3 | 3 / 3 / 3 | 0 | 1 | 0 / 0 |
| gnss_startup | V13 | gust_lateral_p10_VL | gnss_isolated_noise1_delay0 | 3 / 3 | 3 / 1 / 1 | 0 | 3 | 2 / 2 |
| gnss_startup | V13 | gust_lateral_p10_VL | gnss_isolated_noise1_delay1 | 3 / 3 | 3 / 1 / 1 | 0 | 3 | 2 / 2 |
| integration | CPID | gust_lateral_p10_VL | nominal | 3 / 3 | 3 / 3 / 3 | 0 | 3 | 0 / 0 |
| integration | CPID | gust_lateral_p10_VL | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | CPID | gust_lateral_p10_VL | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | F13 | gust_lateral_p10_VH | nominal | 3 / 3 | 3 / 3 / 3 | 0 | 3 | 0 / 0 |
| integration | F13 | gust_lateral_p10_VH | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | F13 | gust_lateral_p10_VH | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | F13 | gust_lateral_p10_VL | nominal | 3 / 3 | 3 / 0 / 0 | 0 | 3 | 3 / 3 |
| integration | F13 | gust_lateral_p10_VL | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | F13 | gust_lateral_p10_VL | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | GSLQR | gust_lateral_p10_VH | nominal | 3 / 3 | 2 / 3 / 2 | 0 | 3 | 1 / 0 |
| integration | GSLQR | gust_lateral_p10_VH | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | GSLQR | gust_lateral_p10_VH | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | GSLQR | gust_lateral_p10_VL | nominal | 3 / 3 | 3 / 2 / 2 | 0 | 3 | 1 / 1 |
| integration | GSLQR | gust_lateral_p10_VL | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | GSLQR | gust_lateral_p10_VL | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | M17 | gust_lateral_p10_VH | nominal | 3 / 3 | 1 / 3 / 1 | 0 | 3 | 2 / 0 |
| integration | M17 | gust_lateral_p10_VH | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | M17 | gust_lateral_p10_VH | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | M17 | gust_lateral_p10_VL | nominal | 3 / 3 | 3 / 0 / 0 | 0 | 3 | 3 / 3 |
| integration | M17 | gust_lateral_p10_VL | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | M17 | gust_lateral_p10_VL | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | V13 | gust_lateral_p10_VH | nominal | 3 / 3 | 3 / 3 / 3 | 0 | 3 | 0 / 0 |
| integration | V13 | gust_lateral_p10_VH | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | V13 | gust_lateral_p10_VH | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | V13 | gust_lateral_p10_VL | nominal | 3 / 3 | 3 / 0 / 0 | 0 | 3 | 3 / 3 |
| integration | V13 | gust_lateral_p10_VL | quiet_sampled | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |
| integration | V13 | gust_lateral_p10_VL | truth | 1 / 1 | 1 / 1 / 1 | 0 | 1 | 0 / 0 |

The final column applies the current tuning failure rule `stop_reason or paper_failed` to these development records. It is not an actual tuning evaluation, and these cases are not moved into the tuning set. Its first count covers all failed acceptance verdicts; the second is the domain-failure subset and must not be added to the first. These trials still incur their ordinary RMSE cost; the counts concern the binary failure penalty only. Model-domain validity and some tracking acceptance checks, such as pre-gust settling, are separate from the inherited paper-failure rule. Declare their intended treatment before final tuning; this report does not change the objective.

## Nominal repeat checks

These full-context cells retain nominal noise, delay, covariance and controller settings, while enabling detailed update logging. Compare them with the corresponding serial integration trials; unavailable comparisons are not passes.

| Factorial trial | Comparison available | Trajectory bit-identical | Verdicts identical |
|---|---|---|---|
| gnss_startup/V13/gust_lateral_p10_VL/gnss_full_noise1_delay1/seed_3 | True | True | True |
| gnss_startup/V13/gust_lateral_p10_VL/gnss_full_noise1_delay1/seed_4 | True | True | True |
| gnss_startup/V13/gust_lateral_p10_VL/gnss_full_noise1_delay1/seed_5 | True | True | True |
| gnss_startup/F13/gust_lateral_p10_VL/gnss_full_noise1_delay1/seed_3 | True | True | True |
| gnss_startup/F13/gust_lateral_p10_VL/gnss_full_noise1_delay1/seed_4 | True | True | True |
| gnss_startup/F13/gust_lateral_p10_VL/gnss_full_noise1_delay1/seed_5 | True | True | True |

## Startup diagnostics

Errors and minima below cover the first 0.5 s, or the recorded prefix if it stopped sooner. The first GNSS update is logged at its first processing; its injection occurs at the historical sample time. Arrival also triggers propagation/replay, and several measurements can affect the next controller solve. Timing alone does not isolate a causal effect.

| Controller | Condition | Seed | Peak position / velocity error | First GNSS processing [s] | First domain crossing [s] | Minimum actual RPM |
|---|---|---:|---|---:|---:|---:|
| V13 | gnss_isolated_noise0_delay0 | 3 | 5.971e-06 m / 0.0001147 m/s | 0 | — | 1.134e+04 |
| V13 | gnss_isolated_noise0_delay0 | 4 | 5.971e-06 m / 0.0001147 m/s | 0 | — | 1.134e+04 |
| V13 | gnss_isolated_noise0_delay0 | 5 | 5.971e-06 m / 0.0001147 m/s | 0 | — | 1.134e+04 |
| V13 | gnss_isolated_noise0_delay1 | 3 | 1.44e-05 m / 0.0001147 m/s | 0.12 | — | 1.134e+04 |
| V13 | gnss_isolated_noise0_delay1 | 4 | 1.44e-05 m / 0.0001147 m/s | 0.12 | — | 1.134e+04 |
| V13 | gnss_isolated_noise0_delay1 | 5 | 1.44e-05 m / 0.0001147 m/s | 0.12 | — | 1.134e+04 |
| V13 | gnss_isolated_noise1_delay0 | 3 | 0.4816 m / 0.2265 m/s | 0 | 0.018 | 9522 |
| V13 | gnss_isolated_noise1_delay0 | 4 | 0.3184 m / 0.2084 m/s | 0 | 0.212 | 9008 |
| V13 | gnss_isolated_noise1_delay0 | 5 | 0.4161 m / 0.1865 m/s | 0 | — | 1.062e+04 |
| V13 | gnss_isolated_noise1_delay1 | 3 | 0.3106 m / 0.2254 m/s | 0.12 | 0.138 | 9451 |
| V13 | gnss_isolated_noise1_delay1 | 4 | 0.2976 m / 0.2069 m/s | 0.12 | 0.332 | 8904 |
| V13 | gnss_isolated_noise1_delay1 | 5 | 0.3717 m / 0.1865 m/s | 0.12 | — | 1.067e+04 |
| V13 | gnss_full_noise0_delay0 | 3 | 0.1877 m / 0.03037 m/s | 0 | 0.18 | 9927 |
| V13 | gnss_full_noise0_delay0 | 4 | 0.3756 m / 0.03668 m/s | 0 | 0.026 | 5896 |
| V13 | gnss_full_noise0_delay0 | 5 | 0.1118 m / 0.03809 m/s | 0 | — | 1.012e+04 |
| V13 | gnss_full_noise0_delay1 | 3 | 0.195 m / 0.04145 m/s | 0.12 | 0.18 | 9795 |
| V13 | gnss_full_noise0_delay1 | 4 | 0.4052 m / 0.04137 m/s | 0.12 | 0.026 | 5709 |
| V13 | gnss_full_noise0_delay1 | 5 | 0.1153 m / 0.04644 m/s | 0.12 | — | 1.003e+04 |
| V13 | gnss_full_noise1_delay0 | 3 | 0.4859 m / 0.2359 m/s | 0 | 0.008 | 8897 |
| V13 | gnss_full_noise1_delay0 | 4 | 0.4793 m / 0.2211 m/s | 0 | 0.038 | 6077 |
| V13 | gnss_full_noise1_delay0 | 5 | 0.4199 m / 0.189 m/s | 0 | 0.106 | 8666 |
| V13 | gnss_full_noise1_delay1 | 3 | 0.3234 m / 0.2371 m/s | 0.12 | 0.136 | 7988 |
| V13 | gnss_full_noise1_delay1 | 4 | 0.4052 m / 0.2321 m/s | 0.12 | 0.026 | 5709 |
| V13 | gnss_full_noise1_delay1 | 5 | 0.3782 m / 0.1762 m/s | 0.12 | 0.132 | 9342 |
| F13 | gnss_isolated_noise0_delay0 | 3 | 1.086e-05 m / 0.000207 m/s | 0 | — | 1.133e+04 |
| F13 | gnss_isolated_noise0_delay0 | 4 | 1.086e-05 m / 0.000207 m/s | 0 | — | 1.133e+04 |
| F13 | gnss_isolated_noise0_delay0 | 5 | 1.086e-05 m / 0.000207 m/s | 0 | — | 1.133e+04 |
| F13 | gnss_isolated_noise0_delay1 | 3 | 1.584e-05 m / 0.0002069 m/s | 0.12 | — | 1.133e+04 |
| F13 | gnss_isolated_noise0_delay1 | 4 | 1.584e-05 m / 0.0002069 m/s | 0.12 | — | 1.133e+04 |
| F13 | gnss_isolated_noise0_delay1 | 5 | 1.584e-05 m / 0.0002069 m/s | 0.12 | — | 1.133e+04 |
| F13 | gnss_isolated_noise1_delay0 | 3 | 0.4816 m / 0.2266 m/s | 0 | — | 1.019e+04 |
| F13 | gnss_isolated_noise1_delay0 | 4 | 0.3184 m / 0.2082 m/s | 0 | 0.216 | 9497 |
| F13 | gnss_isolated_noise1_delay0 | 5 | 0.4161 m / 0.1864 m/s | 0 | — | 1.076e+04 |
| F13 | gnss_isolated_noise1_delay1 | 3 | 0.3106 m / 0.2256 m/s | 0.12 | — | 1.005e+04 |
| F13 | gnss_isolated_noise1_delay1 | 4 | 0.2976 m / 0.2068 m/s | 0.12 | 0.338 | 9519 |
| F13 | gnss_isolated_noise1_delay1 | 5 | 0.3717 m / 0.1865 m/s | 0.12 | — | 1.081e+04 |
| F13 | gnss_full_noise0_delay0 | 3 | 0.1876 m / 0.03047 m/s | 0 | — | 1.03e+04 |
| F13 | gnss_full_noise0_delay0 | 4 | 0.3756 m / 0.03687 m/s | 0 | 0.026 | 7213 |
| F13 | gnss_full_noise0_delay0 | 5 | 0.1118 m / 0.03811 m/s | 0 | — | 1.048e+04 |
| F13 | gnss_full_noise0_delay1 | 3 | 0.195 m / 0.04141 m/s | 0.12 | — | 1.03e+04 |
| F13 | gnss_full_noise0_delay1 | 4 | 0.4052 m / 0.04162 m/s | 0.12 | 0.026 | 7157 |
| F13 | gnss_full_noise0_delay1 | 5 | 0.1152 m / 0.04646 m/s | 0.12 | — | 1.052e+04 |
| F13 | gnss_full_noise1_delay0 | 3 | 0.4859 m / 0.2358 m/s | 0 | 0.016 | 9720 |
| F13 | gnss_full_noise1_delay0 | 4 | 0.4793 m / 0.2215 m/s | 0 | 0.044 | 8230 |
| F13 | gnss_full_noise1_delay0 | 5 | 0.4199 m / 0.1888 m/s | 0 | 0.132 | 9667 |
| F13 | gnss_full_noise1_delay1 | 3 | 0.3234 m / 0.2373 m/s | 0.12 | 0.152 | 9266 |
| F13 | gnss_full_noise1_delay1 | 4 | 0.4052 m / 0.2313 m/s | 0.12 | 0.026 | 7157 |
| F13 | gnss_full_noise1_delay1 | 5 | 0.3782 m / 0.1759 m/s | 0.12 | 0.156 | 9998 |

![Startup actual rotor speeds](startup_rotor.png)

Each thin curve is one development seed; colors identify the GNSS noise/delay cell. The dashed line is the unchanged assumed 10,000 RPM propeller-model boundary. Isolated and full contexts differ in other sensor errors and rotor policy; compare factorial axes within each context. A model-domain crossing is not proof of hardware instability.

[All trial records and separate execution statuses](validated/report.md)
