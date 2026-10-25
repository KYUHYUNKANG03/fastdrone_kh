| 실험 | 내용 | 행 수 | 실행된 시행 | 실행 환경 | 파일 |
|---|---|---:|---:|---:|---:|
| `main8` | 예비 비교: 5개 제어기 × 114 시나리오 | 11400 | 10020 | Windows PC-61~64 | `data/main8_trials.csv` |
| `main8_ladder` | V13 구성요소 사다리(명목·섭동 3종, 시드 10개) | 2250 | 2250 | Windows PC-63 | `data/main8_ladder_trials.csv` |
| `sensor8` | 센서 조건 27개 × 핵심 시나리오 7개, V13/F13/GSLQR(+CPID) | 7542 | 7542 | Windows PC-63 | `data/sensor8_trials.csv` |
| `ablation8` | V13-0/1/2/V13 × 명목·IMU 5 ms × 돌풍·가속·감속 | 480 | 480 | Linux 클라우드 | `data/ablation8_trials.csv` |
| `weightcheck8` | F13↔V13-0 비용 가중치 맞바꿈(IMU 5 ms) — 논문 미인용 | 46 | 46 | Linux 클라우드 | `data/weightcheck8_trials.csv` |
