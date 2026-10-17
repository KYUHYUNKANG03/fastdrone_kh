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

### 아무것도 없는 새 PC

Git과 Python이 없는 PC에서도 같은 한 줄이면 된다. 관리자 권한은 필요 없다.
- git이 없으면 MinGit 2.51.0(설치 없는 압축본, SHA256 고정)을 `$HOME\mingit`에 풀어 쓴다.
- Python 3.13이 없으면 3.13.7을 이 사용자에게만 조용히 설치한다(py 런처는 설치하지 않는다).
- 그다음 venv, 패키지, 점검(약 15분)은 다른 PC와 같다. 설치까지 합쳐 30분쯤 걸린다.

M17 작업자는 하나에 약 3.7 GiB를 쓰므로 메모리 16 GB PC여야 한다(작업 관리자 > 성능 > 메모리).

### M17 3대 분할 (2026-10-14)

`-Shards 6`으로 PC-61 `-Index 0,1`, PC-64 `-Index 2,3`, PC-62 `-Index 4,5 -Queue`. 튜닝 실측으로 잡은 추정은
모든 시행이 끝까지 돌 때 4.3~5.3일이다. 일찍 멈추는 시행이 있으면 그만큼 짧아진다.

- PC-60(i7-11700)도 시도했지만 tune7 V13 경로 재현이 학교 i7-10700 값과 비트 단위로 맞지 않아(`no match`)
  점검에서 멈췄다. AVX-512가 있는 CPU라 numpy·OpenBLAS의 계산 경로가 다른 것으로 보인다. M17은 i7-10700에서만 돌린다.
- PC-62는 F13 본실험이 아직 돌고 있어서 `-Queue`를 붙인다. 이 PC의 다른 main8 체인·조각 작업자·튜닝이 모두 끝날
  때까지 아무것도 하지 않고 기다렸다가 시작한다(5분마다 확인, 대기 시작 때 ntfy 알림 한 번).

## 센서 실험 (sensor8, PC-63, 2026-10-17)

tune8 게인을 그대로 두고 센서 조건만 하나씩 바꿔 V13·F13·GSLQR(V_L 시나리오는 CPID 포함)을 비교한다.
PC-63 한 대에서 돈다. PC-63의 main8(V13·CPID)이 쓴 fds8를 그대로 쓰고, F13·GSLQR 기록은 GitHub `records/`에서 받는다.

```powershell
irm "https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school/sensor8_chain.ps1" -OutFile $HOME\sensor8_chain.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\sensor8_chain.ps1 -Topic <ntfy 주제>
```

중간 결과(언제든, 돌고 있는 실험은 건드리지 않는다): 바탕화면에 `sensor8_PC-63_rows_<날짜>.zip`이 생긴다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\sensor8_chain.ps1 -Collect
```

- 무엇을 돌리나(`school/sensor8_run.py`에 고정): 시나리오 5개(V_H 횡돌풍 +10, V_H 수직돌풍 −5, V_H 가속 ρ1,
  V_H 감속 ρ1, V_L 횡돌풍 +10)와 모델 오차 2개(V_H 순항 moment_25, mass_1.3), 시드 1020~1039, 조건 27개.
  - 0순위: truth(센서 없음, 시드 없이 1회), nominal(main8 프로파일 그대로 — main8 재현 확인 겸 기준선)
  - 1순위(INDI 경로): IMU 지연 2/5/10 ms, rpm 지연 10/20/40 ms, rpm 100 Hz, rpm 끊김 200 ms, gyro 잡음 ×3/×5,
    gyro 진동 150 Hz, rpm 잡음 ×4
  - 2순위(항법): GNSS 잡음 ×3, GNSS 지연 0.3 s, GNSS 끊김 3 s, GNSS 높이 튐 3 m, 기압계 잡음 ×4, 기압계 드리프트, 초기 추정 오차
  - 3순위: degraded(중간 수준 묶음), stress(`configs/sensors/stress.json`의 센서 값)
  - 4순위: 모델 오차 시나리오 × {기준, gyro ×3, rpm 20 ms, IMU 2 ms}
- 추정기 설정(ESKF 공분산 등)은 기준 프로파일 그대로다. 센서가 가정보다 나빠졌을 때의 민감도를 본다.
- 7,542 시행, 작업자 7개 기준 main8 실측으로 약 3일, 여유를 두면 4일. 조건 순서대로 돌고, 레인마다 조건 하나의
  시행을 고르게 나눠 받아서 중간에 끊어도 끝난 조건은 제어기 비교가 완결돼 있다. 조건 하나가 끝날 때마다 ntfy 알림.
- 결과는 `fds8\results\sensor8\PC-63\lane*\trials\*.json`(시행마다 한 줄 요약 + 추정 오차), 그림용 궤적은
  시드 1020과 truth만 `traces\*.npz`(zip에는 안 넣는다, 필요하면 `-Collect -Traces`).
- 끝나면 `summary.md`를 ntfy로 보내고 zip을 바탕화면에 만든다. 재부팅·로그오프 뒤에는 같은 한 줄로 이어 돈다.
- 다른 main8·튜닝이 돌고 있으면 시작하지 않는다(`-Queue`를 붙이면 끝날 때까지 기다렸다 시작).
