# D5 effect check summary (generated from effect_table.json)

Off = prehistory off (low speed: E1 records, reused after 5/5 bit-identical samples; 85 m/s: this branch). On = 30 s prehistory.

## Low speed gust_lateral_p10_VL, seeds 3/4/5

| sensors | metric | V13 | F13 | M17 | GSLQR | CPID |
|---|---|---|---|---|---|---|
| navigation_only | domain-exit seeds (off -> on) | 3 -> 0 | 2 -> 0 | 3 -> 0 | 1 -> 0 | 0 -> 0 |
| navigation_only | exit steps total (off -> on) | 132 -> 0 | 92 -> 0 | 106 -> 0 | 1 -> 0 | 0 -> 0 |
| navigation_only | passed (off -> on) | 0 -> 3 | 1 -> 3 | 0 -> 3 | 2 -> 3 | 3 -> 3 |
| navigation_only | min RPM 0-0.5 s, worst seed (off -> on) | 6102 -> 10424 | 7282 -> 10794 | 6185 -> 10287 | 9945 -> 11212 | 10454 -> 11190 |
| navigation_only | max est z err 0-0.2 s, worst seed m (off -> on) | 0.405 -> 0.075 | 0.405 -> 0.075 | 0.405 -> 0.075 | 0.405 -> 0.075 | 0.405 -> 0.075 |
| full | domain-exit seeds (off -> on) | 3 -> 3 | 3 -> 1 | 3 -> 0 | 1 -> 0 | 0 -> 0 |
| full | exit steps total (off -> on) | 149 -> 3 | 111 -> 1 | 115 -> 0 | 1 -> 0 | 0 -> 0 |
| full | passed (off -> on) | 0 -> 0 | 0 -> 2 | 0 -> 3 | 2 -> 3 | 3 -> 3 |
| full | min RPM 0-0.5 s, worst seed (off -> on) | 5709 -> 9944 | 7157 -> 10314 | 6132 -> 10035 | 9936 -> 11141 | 10352 -> 10866 |
| full | max est z err 0-0.2 s, worst seed m (off -> on) | 0.405 -> 0.096 | 0.405 -> 0.096 | 0.405 -> 0.096 | 0.405 -> 0.096 | 0.405 -> 0.096 |

## 85 m/s, full sensors, seed 3 (CPID excluded: design region 0-20 m/s)

| case | ctrl | passed off -> on | reasons off | domain exits off -> on | min RPM 0-0.5 s off -> on | max est z err 0-0.2 s off -> on |
|---|---|---|---|---|---|---|
| gust_lateral_p10_VH | V13 | True -> True | [] | 0 -> 0 | 26967 -> 27029 | 0.196 -> 0.082 |
| gust_lateral_p10_VH | F13 | True -> True | [] | 0 -> 0 | 26961 -> 27012 | 0.196 -> 0.082 |
| gust_lateral_p10_VH | M17 | False -> True | ['pre_gust_not_settled'] | 0 -> 0 | 28045 -> 28191 | 0.195 -> 0.082 |
| gust_lateral_p10_VH | GSLQR | True -> True | [] | 0 -> 0 | 28026 -> 28279 | 0.196 -> 0.082 |
| gust_vertical_m5_VH | V13 | True -> True | [] | 0 -> 0 | 26967 -> 27029 | 0.196 -> 0.082 |
| gust_vertical_m5_VH | F13 | True -> True | [] | 0 -> 0 | 26961 -> 27012 | 0.196 -> 0.082 |
| gust_vertical_m5_VH | M17 | False -> True | ['pre_gust_not_settled'] | 0 -> 0 | 28045 -> 28191 | 0.195 -> 0.082 |
| gust_vertical_m5_VH | GSLQR | True -> True | [] | 0 -> 0 | 28026 -> 28279 | 0.196 -> 0.082 |

## Remaining exits with the prehistory on

- gust_lateral_p10_VL full F13 s4: first exit 5.534 s, steps 0-0.5/0.5-3/3- = 0/0/1, min RPM 0-0.5 s 10403
- gust_lateral_p10_VL full V13 s3: first exit 6.848 s, steps 0-0.5/0.5-3/3- = 0/0/1, min RPM 0-0.5 s 10051
- gust_lateral_p10_VL full V13 s4: first exit 5.534 s, steps 0-0.5/0.5-3/3- = 0/0/1, min RPM 0-0.5 s 10121
- gust_lateral_p10_VL full V13 s5: first exit 0.036 s, steps 0-0.5/0.5-3/3- = 1/0/0, min RPM 0-0.5 s 9944
