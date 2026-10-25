| 제어기 | 게이트 시나리오 | 구분 | 결과 | 속도 RMSE (m/s) |
|---|---|---|---:|---:|
| V13 | gate_cruise_VL | 필수 | 통과 | 0.082 |
| V13 | gate_cruise_VH | 필수 | 통과 | 0.096 |
| V13 | gate_cruise_VH__state_delay_5ms | 참고 | 통과 | 0.101 |
| F13 | gate_cruise_VL | 필수 | 통과 | 0.071 |
| F13 | gate_cruise_VH | 필수 | 통과 | 0.083 |
| F13 | gate_cruise_VH__state_delay_5ms | 참고 | 실패 | 0.114 |
| M17 | gate_cruise_VL | 필수 | 통과 | 0.070 |
| M17 | gate_cruise_VH | 필수 | 통과 | 0.123 |
| M17 | gate_cruise_VH__state_delay_5ms | 참고 | 통과 | 0.100 |
| GSLQR | gate_cruise_VL | 필수 | 통과 | 0.077 |
| GSLQR | gate_cruise_VH | 필수 | 통과 | 0.093 |
| GSLQR | gate_cruise_VH__state_delay_5ms | 참고 | 통과 | 0.114 |
| CPID | gate_cruise_VL | 필수 | 통과 | 0.067 |
| CPID | gate_cruise_VH | 필수 | skip |  |
| CPID | gate_cruise_VH__state_delay_5ms | 참고 | skip |  |
