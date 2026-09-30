# Prepared sensor validation

**Development only; fixed team gains.**

Planned: 93. Recorded: 93. Execution incomplete: 0. Pending: 0.

Pending cases are not successes or executed failures. Failures remain in the executed denominator. Shared deterministic controls and quiet repeats are not independent trials. Tracking and propulsion-domain verdicts remain separate. No hardware limit or reliability claim.

RMSE covers only recorded time. Read duration and stop reason before comparing failures. Diagnostic availability never changes an original trial verdict.

| Stage | Controller | Case / condition | Seed | Execution | Track / domain | Duration [s] | Pair |
|---|---|---|---:|---|---|---:|---|
| integration | V13 | gust_lateral_p10_VL / truth | 0 | recorded | True / True | 12.0 | control |
| integration | V13 | gust_lateral_p10_VL / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | V13 | gust_lateral_p10_VL / nominal | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | V13 | gust_lateral_p10_VL / nominal | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | V13 | gust_lateral_p10_VL / nominal | 5 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | V13 | gust_lateral_p10_VH / truth | 0 | recorded | True / True | 12.0 | control |
| integration | V13 | gust_lateral_p10_VH / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | V13 | gust_lateral_p10_VH / nominal | 3 | recorded | True / True | 12.0 | both_pass |
| integration | V13 | gust_lateral_p10_VH / nominal | 4 | recorded | True / True | 12.0 | both_pass |
| integration | V13 | gust_lateral_p10_VH / nominal | 5 | recorded | True / True | 12.0 | both_pass |
| integration | F13 | gust_lateral_p10_VL / truth | 0 | recorded | True / True | 12.0 | control |
| integration | F13 | gust_lateral_p10_VL / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | F13 | gust_lateral_p10_VL / nominal | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | F13 | gust_lateral_p10_VL / nominal | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | F13 | gust_lateral_p10_VL / nominal | 5 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | F13 | gust_lateral_p10_VH / truth | 0 | recorded | True / True | 12.0 | control |
| integration | F13 | gust_lateral_p10_VH / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | F13 | gust_lateral_p10_VH / nominal | 3 | recorded | True / True | 12.0 | both_pass |
| integration | F13 | gust_lateral_p10_VH / nominal | 4 | recorded | True / True | 12.0 | both_pass |
| integration | F13 | gust_lateral_p10_VH / nominal | 5 | recorded | True / True | 12.0 | both_pass |
| integration | M17 | gust_lateral_p10_VL / truth | 0 | recorded | True / True | 12.0 | control |
| integration | M17 | gust_lateral_p10_VL / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | M17 | gust_lateral_p10_VL / nominal | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | M17 | gust_lateral_p10_VL / nominal | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | M17 | gust_lateral_p10_VL / nominal | 5 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | M17 | gust_lateral_p10_VH / truth | 0 | recorded | True / True | 12.0 | control |
| integration | M17 | gust_lateral_p10_VH / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | M17 | gust_lateral_p10_VH / nominal | 3 | recorded | False / True | 12.0 | fusion_only_failure |
| integration | M17 | gust_lateral_p10_VH / nominal | 4 | recorded | True / True | 12.0 | both_pass |
| integration | M17 | gust_lateral_p10_VH / nominal | 5 | recorded | False / True | 12.0 | fusion_only_failure |
| integration | GSLQR | gust_lateral_p10_VL / truth | 0 | recorded | True / True | 12.0 | control |
| integration | GSLQR | gust_lateral_p10_VL / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | GSLQR | gust_lateral_p10_VL / nominal | 3 | recorded | True / True | 12.0 | both_pass |
| integration | GSLQR | gust_lateral_p10_VL / nominal | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| integration | GSLQR | gust_lateral_p10_VL / nominal | 5 | recorded | True / True | 12.0 | both_pass |
| integration | GSLQR | gust_lateral_p10_VH / truth | 0 | recorded | True / True | 12.0 | control |
| integration | GSLQR | gust_lateral_p10_VH / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | GSLQR | gust_lateral_p10_VH / nominal | 3 | recorded | True / True | 12.0 | both_pass |
| integration | GSLQR | gust_lateral_p10_VH / nominal | 4 | recorded | True / True | 12.0 | both_pass |
| integration | GSLQR | gust_lateral_p10_VH / nominal | 5 | recorded | False / True | 12.0 | fusion_only_failure |
| integration | CPID | gust_lateral_p10_VL / truth | 0 | recorded | True / True | 12.0 | control |
| integration | CPID | gust_lateral_p10_VL / quiet_sampled | 0 | recorded | True / True | 12.0 | both_pass |
| integration | CPID | gust_lateral_p10_VL / nominal | 3 | recorded | True / True | 12.0 | both_pass |
| integration | CPID | gust_lateral_p10_VL / nominal | 4 | recorded | True / True | 12.0 | both_pass |
| integration | CPID | gust_lateral_p10_VL / nominal | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay0 | 3 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay0 | 4 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay0 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay1 | 3 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay1 | 4 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay1 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay0 | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay0 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay0 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay1 | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay1 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay1 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise0_delay0 | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise0_delay0 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise0_delay0 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise0_delay1 | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise0_delay1 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise0_delay1 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise1_delay0 | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise1_delay0 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise1_delay0 | 5 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise1_delay1 | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise1_delay1 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | V13 | gust_lateral_p10_VL / gnss_full_noise1_delay1 | 5 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay0 | 3 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay0 | 4 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay0 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay1 | 3 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay1 | 4 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise0_delay1 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay0 | 3 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay0 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay0 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay1 | 3 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay1 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_isolated_noise1_delay1 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise0_delay0 | 3 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise0_delay0 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise0_delay0 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise0_delay1 | 3 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise0_delay1 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise0_delay1 | 5 | recorded | True / True | 12.0 | both_pass |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise1_delay0 | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise1_delay0 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise1_delay0 | 5 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise1_delay1 | 3 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise1_delay1 | 4 | recorded | True / False | 12.0 | fusion_only_failure |
| gnss_startup | F13 | gust_lateral_p10_VL / gnss_full_noise1_delay1 | 5 | recorded | True / False | 12.0 | fusion_only_failure |
