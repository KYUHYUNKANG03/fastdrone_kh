| 조건 | 시나리오 | V13 | F13 | V13만 통과한 시드 | F13만 통과한 시드 | 부호 검정 p |
|---|---:|---:|---:|---:|---:|---:|
| imu_latency_5ms | gust_lateral_+10_V_H | 20/20 | 0/20 | 20 | 0 | 1.9e-06 |
| imu_latency_5ms | gust_vertical_-5_V_H | 20/20 | 0/20 | 20 | 0 | 1.9e-06 |
| imu_latency_5ms | ref_accel_VH_rho1 | 5/20 | 0/20 | 5 | 0 | 0.062 |
| imu_latency_10ms | ref_brake_VH_rho1 | 17/20 | 0/20 | 17 | 0 | 1.5e-05 |
| gyro_noise_x3 | gust_lateral_+10_V_H | 12/20 | 4/20 | 9 | 1 | 0.021 |
| gyro_noise_x5 | ref_brake_VH_rho1 | 2/20 | 16/20 | 1 | 15 | 5.2e-04 |
| nominal | ref_brake_VH_rho1 | 14/20 | 19/20 | 0 | 5 | 0.062 |
