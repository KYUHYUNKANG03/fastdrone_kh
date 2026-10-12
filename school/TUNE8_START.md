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
   센서 기준 2개는 tune-final-7 코드 해시에 묶여 있어서 `school/tune8_sensor_reproduce.py`로 돌린다 — 바뀐 파일이 `control/arena_tune.py` 하나뿐인지 확인한 뒤 수치 비교는 원래대로 한다.
3. 통과하면 튜닝을 숨은 창으로 시작한다. 창을 닫아도 계속 돈다.
4. 제어기마다 감시 프로세스가 PC가 잠들지 않게 하고, 튜닝이 끝나면 **본 실험 전 관문**을 돌린다.

재부팅·로그오프로 멈췄으면 같은 한 줄을 다시 붙여 넣는다. 끝난 평가는 건너뛰고 이어 간다.

M17 작업자는 시작부터 하나에 약 3.7 GiB를 쓴다(PC-61 실측, 스크립트 가정은 2.0 GiB). 16 GiB PC에서 3개는 들어가지 않으므로 스크립트가 M17을 2개로 제한한다.

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

## 본 실험 (main8)

관문을 통과한 제어기부터 PC마다 따로 시작한다. tune7 본 실험(main7)과 같은 겹치기 실행이다 — 시행 하나는
(시나리오, 제어기, 시드) 단위로 독립이라 PC마다 자기 제어기만 돌리고 나중에 합친다.

```powershell
irm "https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school/main8_chain.ps1" -OutFile $HOME\main8_chain.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\main8_chain.ps1 -Controllers F13 -Topic <ntfy 주제>
```

`-Controllers`는 PC-61 `M17`, PC-62 `F13`, PC-63 `V13,CPID`, PC-64 `GSLQR`.

- 튜닝이 60회에 끝날 때까지 기다리고(튜닝을 새로 시작하지는 않는다), 관문 결과를 확인한 뒤 시작한다.
  관문이 FAIL이면 그 PC는 본 실험을 시작하지 않고 멈춘다.
- 설정은 `school/main8_make_spec.py`가 만든다: 기준 설정 `configs/arena_tune8.json`, 기록 `tune8_final`,
  시드 1020~1039(사다리 1020~1029). tune7 본 실험이 쓴 1000~1019는 다시 쓰지 않는다.
- 시나리오 구성은 main7과 같다(V_H 85, 섭동 표, 돌풍, 임무, V13 사다리). 시행 수도 같다(V13+CPID 6810, 나머지 2280).
- 결과는 `fds8\results\main8\<PC>_<제어기>`. M17은 작업자 2개로 제한한다.

### 한 제어기를 여러 PC에 나눠 돌리기

`-Shards`(전체 조각 수)와 `-Index`(이 PC가 맡을 조각)를 주면 된다. 모든 PC가 같은 설정과 같은 조각 파일을
만들고(둘 다 결정적이다) 자기 조각만 돌린 뒤, 결과 폴더를 나중에 합친다(`control.main_distributed merge`).

```powershell
irm "https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school/main8_chain.ps1" -OutFile $HOME\main8_chain.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\main8_chain.ps1 -Controllers M17 -Shards 12 -Index 0,1 -Topic <ntfy 주제>
```

- 튜닝한 PC(M17은 PC-61)가 `-Index 0,1`, 나머지 PC가 `2,3` / `4,5` / ... 를 맡는다. 같은 번호를 두 PC가 맡으면 안 된다.
- 새 PC는 처음에 fds8 설치와 점검(약 15분)을 스스로 한 뒤, 튜닝 기록이 들어올 때까지 기다린다.
- 튜닝 기록(`<제어기>.record.json`, `<제어기>.gate.json`)을 찾는 곳: 바탕화면, 다운로드, USB 등 드라이브 맨 위,
  GitHub `tune8` 브랜치의 `records/`. 튜닝한 PC는 관문을 통과하면 이 두 파일을 ntfy 알림으로 보낸다.
- 들어온 기록은 쓰기 전에 검사한다: 60회 완료, `arena_tune8` 설정, 그 기록으로 돈 관문 PASS.
