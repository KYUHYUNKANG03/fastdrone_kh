# tune8 재튜닝 — 학교 PC 4대 시작 절차 (2026-10-10)

tune7 게인으로 돌린 본 실험에서 V13이 V_H(85 m/s)에 닿는 시나리오 대부분에서 발산했다.
원인은 제어기 구조가 아니라 **튜닝 집합이 고속에서 짧았던 것**이다. 이 문서는 고친 튜닝(tune8)을
같은 학교 PC 4대에서 다시 돌리는 절차다.

## 무엇이 왜 바뀌었나

| 확인한 것 | 근거 |
|---|---|
| 튜닝된 V13은 85 m/s 순항에서 발산, 초기값은 통과 | 같은 시드·같은 시나리오, 게인만 바꿈 |
| 불안정 경계는 약 80.6 m/s, 82 m/s에서 진폭이 두 배 되는 데 약 2.8 s | 75~85 m/s 속도 스윕 |
| tune7 튜닝 집합은 최고 82 m/s, 고속 돌풍 4 s | `configs/arena_tune7.json` |
| 탐색이 `w_v` 5→10, `w_omega` 1.0→0.5로 옮겨 고속 감쇠를 깎음 | 게인 하나씩 되돌린 시행 |
| F13도 같은 방향으로 깎여 4 Hz 진동이 4~5배 | 85 m/s 순항, 초기값 대 튜닝값 |

tune8의 변경(`configs/arena_tune8.json`, 근거 전문은 그 파일의 `scenario_design_note_3`):

| 묶음 | tune7 | tune8 |
|---|---|---|
| 저속 돌풍 4개(14·26 m/s) | 4 s | 그대로(id·시드 동일) |
| 고속 돌풍 4개 | 75·82 m/s, 4 s | 83·85 m/s, 12 s (큰 돌풍은 83, 작은 돌풍은 85) |
| 고속 순항 | 없음 | 83 m/s 15 s |
| 가속·제동 | 꼬리 1 s | 꼬리 5 s |
| 정지 출발 가속 | 없음 | 0→26 ρ1.2, 0→83 ρ0.8 |
| 약한 섭동 | 없음 | 83 m/s 순항 + 상태 지연 5 ms, 질량 1.15배 |
| 계단 8개 | 8 s | 그대로(id·시드 동일) |
| 실패 판정 | 시뮬 중단·논문 기준 | + 본시험 합격 기준(Acceptance) 미달 |
| 예산 | 120(→180) | 60 |

- 시나리오 23개, 시뮬 시간 합 132.7 s → 233.6 s(1.76배). 60회 × 233.6 s는 tune7 120회의 0.88배다.
- 87 m/s는 팀 기체 확인 범위(0~85 m/s) 밖이라 쓰지 않았다. 위쪽 끝은 V_H(85) 자체이고, 본시험과 크기(0.7배)·시드가 다르다.
- 코드 변경은 `control/arena_tune.py`의 `objective.acceptance_failure` 하나다. 꺼져 있으면(tune7 이하) 예전과 똑같이 돈다.

## PC 배정 (tune7 본 실험과 같다)

| PC | 제어기 | 한 줄 명령 끝에 붙일 것 |
|---|---|---|
| PC-61 | M17 | `-Controller M17` |
| PC-62 | F13 | `-Controller F13` |
| PC-63 | V13, CPID | `-Controller V13,CPID` |
| PC-64 | GSLQR | `-Controller GSLQR` |

## 시작 (PC마다 PowerShell에 한 줄)

```powershell
irm https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school/tune8_all.ps1 -OutFile $HOME\tune8_all.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\tune8_all.ps1 -Controller M17
```

`-Controller` 뒤만 PC에 맞게 바꾼다. 휴대폰 알림을 받으려면 끝에 `-Topic <본인 ntfy 주제>`를 붙인다.

스크립트가 하는 일:
1. 새 폴더 `$HOME\fds8`에 코드를 받는다. **tune7 폴더 `$HOME\fds`(튜닝 기록·본 실험 결과)는 건드리지 않는다.**
2. 점검 약 15분: 환경, 센서 기준 2개, tune7 설정의 V13 튜닝 경로 재현(학교 Windows 값과 비트 일치), 고아 작업자.
3. 통과하면 튜닝을 숨은 창으로 시작한다. 창을 닫아도 계속 돈다.
4. 제어기마다 감시 프로세스가 PC가 잠들지 않게 하고, 튜닝이 끝나면 **본 실험 전 관문**을 돌린다.

재부팅·로그오프로 멈췄으면 같은 한 줄을 다시 붙여 넣는다. 끝난 평가는 건너뛰고 이어 간다.

## 진행 확인

```powershell
Select-String '"spent"|"status"' $HOME\fds8\results\arena\tuning\tune8\*.record.json
```

## 예상 시간 (추정)

| 제어기 | 추정 | 근거 |
|---|---|---|
| M17 | 3.5~4일 | tune7 120회가 약 4일 × 0.88 |
| F13 | 1~2일 | |
| V13 | 하루 안쪽 | |
| GSLQR, CPID | 몇 시간 | |

## 본 실험 전 관문 (자동)

튜닝이 끝나면 `school/tune8_gate.py`가 그 제어기의 최선값으로 다음을 돌린다(튜닝 시드 2901~2903, 본시험 시드는 쓰지 않는다).

| 사례 | 판정 |
|---|---|
| V_L 순항 15 s, 섭동 없음 | 필수 |
| V_H 순항 15 s, 섭동 없음 | 필수 (CPID는 설계 영역 밖이라 제외) |
| V_H 순항 15 s + 상태 지연 5 ms | 참고(기록만) |

결과는 `fds8\results\arena\tuning\tune8\<제어기>.gate.json`, 로그는 `fds8\tune8_gate_<제어기>.log`.
관문은 통과/실패만 보고한다. **본 실험은 자동으로 시작하지 않는다** — 다섯 제어기의 관문 결과를 보고 사람이 정한다.

```powershell
Select-String '"gate_pass"' $HOME\fds8\results\arena\tuning\tune8\*.gate.json
```

## 결과 꺼내기

```powershell
tar -a -c -f "$HOME\tune8_$env:COMPUTERNAME.zip" -C $HOME\fds8 results/arena/tuning/tune8
```

## 아직 정하지 않은 것

- 본 실험 설정(spec)과 시드. 이 개정은 본 실험 결과를 본 뒤에 나왔으므로 새 시드 구간으로 돌리는 것이 깨끗하다.
- 60회 예산은 M17 일정에 맞춘 값이다. I-4에 따라 다섯 제어기 모두 같은 예산이어야 한다.
- 초기값을 새 집합으로 미리 돌려 본 결과(리눅스 진단 환경): V13은 23개 전부 통과(목적함수 0.348), GSLQR은 정지 출발 가속 2개만 실패(87.4), CPID는 설계 영역(0~20 m/s) 밖 13개 실패(565.4). 그대로 둔 12개 시나리오의 V13 점수는 학교 tune7 기록과 1e-12 안에서 같다. F13도 23개 전부 통과(0.314). M17은 한 평가가 길어 사전 확인 없이 시작한다.
