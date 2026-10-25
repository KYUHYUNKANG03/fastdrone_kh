| 조건 | 시나리오 | 비교 | 앞쪽만 통과 | 뒤쪽만 통과 | 부호 검정 p |
|---|---|---|---:|---:|---:|
| nominal | gust_lateral_+10_V_H | V13-0 → V13-1 (A1 추가) | 0 | 1 | 1 |
| nominal | gust_lateral_+10_V_H | V13-0 → V13 (A1+C1+S1) | 0 | 1 | 1 |
| nominal | ref_accel_VH_rho1 | V13-0 → V13-1 (A1 추가) | 3 | 0 | 0.25 |
| nominal | ref_accel_VH_rho1 | V13-1 → V13-2 (C1 추가) | 3 | 0 | 0.25 |
| nominal | ref_accel_VH_rho1 | V13-2 → V13 (S1 추가) | 0 | 2 | 0.5 |
| nominal | ref_accel_VH_rho1 | V13-0 → V13 (A1+C1+S1) | 4 | 0 | 0.12 |
| nominal | ref_brake_VH_rho1 | V13-0 → V13-1 (A1 추가) | 1 | 11 | 0.0063 |
| nominal | ref_brake_VH_rho1 | V13-1 → V13-2 (C1 추가) | 2 | 2 | 1 |
| nominal | ref_brake_VH_rho1 | V13-2 → V13 (S1 추가) | 4 | 3 | 1 |
| nominal | ref_brake_VH_rho1 | V13-0 → V13 (A1+C1+S1) | 2 | 11 | 0.022 |
| imu_latency_5ms | gust_lateral_+10_V_H | V13-0 → V13-1 (A1 추가) | 0 | 1 | 1 |
| imu_latency_5ms | gust_lateral_+10_V_H | V13-0 → V13 (A1+C1+S1) | 0 | 1 | 1 |
| imu_latency_5ms | ref_accel_VH_rho1 | V13-0 → V13-1 (A1 추가) | 11 | 0 | 9.8e-04 |
| imu_latency_5ms | ref_accel_VH_rho1 | V13-1 → V13-2 (C1 추가) | 4 | 0 | 0.12 |
| imu_latency_5ms | ref_accel_VH_rho1 | V13-2 → V13 (S1 추가) | 1 | 2 | 1 |
| imu_latency_5ms | ref_accel_VH_rho1 | V13-0 → V13 (A1+C1+S1) | 14 | 0 | 1.2e-04 |
| imu_latency_5ms | ref_brake_VH_rho1 | V13-0 → V13-1 (A1 추가) | 0 | 9 | 0.0039 |
| imu_latency_5ms | ref_brake_VH_rho1 | V13-1 → V13-2 (C1 추가) | 2 | 3 | 1 |
| imu_latency_5ms | ref_brake_VH_rho1 | V13-2 → V13 (S1 추가) | 0 | 2 | 0.5 |
| imu_latency_5ms | ref_brake_VH_rho1 | V13-0 → V13 (A1+C1+S1) | 0 | 12 | 4.9e-04 |
