# fast-drone 재현 및 추가 실험

원본: protkjj/fast-drone (팀 제어소년단)
로컬 재현 + 실험 추가 사본. 원본에는 push하지 않음.

## 환경
Ubuntu / Python venv (.venv 미포함)
numpy 2.5.2, scipy 1.18.1, casadi 3.8.0, matplotlib 3.11.1
NMPC = CasADi + IPOPT. acados 미설치 (향후 과제).

## 추가분
- sensors.py: GPS 지연/결측 모델 (기본값에서 원본과 동일)
- sensor_sweep.py: 센서 격자 17종 x 제어기 3종 + MC 모드
- show_grid.py: 격자 CSV 요약
- omega_plot.py: 각속도 시계열 그림
- results/: 산출물 (csv, png)

## 주요 결과
실기 판정 (|w| > 35 rad/s, 또는 25 rad/s 200ms 지속):
- 분리형 단독 0/17, 안전계층 17/17, LQR 16/17

참값 65s 미션:
- 단독: RMSE z 1.633 m, |w|max 62.08 rad/s, FAIL
- 안전계층: RMSE z 2.676 m, |w|max 15.54 rad/s, PASS (전환 t=46.5s)
