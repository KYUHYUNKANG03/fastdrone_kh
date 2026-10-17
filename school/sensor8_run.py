"""sensor8 — tune8 게인 그대로, 센서 조건만 바꿔 네 제어기를 비교하는 센서 실험(PC-63 한 대).

main8과 같은 것: 기준 설정 configs/arena_tune8.json, tune8 최종 기록 4개(V13·F13·GSLQR·CPID — main8이 쓴
바로 그 파일, sha256 고정), 시나리오 정의(main8 spec 템플릿에서 그대로 만든다), 센서 시드 1020~1039,
시행 하나를 돌리는 함수(control.validation_suite.run_trial)와 판정(Acceptance·PaperCriteria).
main8과 다른 것: run_trial에 **센서 프로파일만** 바꿔 넘긴다(feedback='sensors', sensor_profile=...).
추정기 설정(ESKF 공분산 등)은 기준 프로파일 그대로 둔다 — '센서가 가정보다 나빠졌을 때의 민감도'를 본다.
nominal 조건은 기준 프로파일을 손대지 않고 넘기므로 main8 시행과 같은 궤적이 나와야 한다(같은 PC면 비트 동일).

control/·configs/·models/는 읽기만 한다. 결과는 <fds8>/results/sensor8/ 아래에만 쓴다.

    python sensor8_run.py plan     --root <fds8> --out <fds8>/results/sensor8
    python sensor8_run.py lanes    --out <fds8>/results/sensor8 --pc PC-63 --n 7
    python sensor8_run.py run-lane --root <fds8> --out <fds8>/results/sensor8 --pc PC-63 --index 0
    python sensor8_run.py status   --out <fds8>/results/sensor8 --pc PC-63 [--done-conditions]
    python sensor8_run.py summary  --out <fds8>/results/sensor8 --pc PC-63

조건은 아래 CONDITIONS 순서(우선순위)대로 돈다. 레인(작업자)마다 조건 하나의 시행을 고르게 나눠 받으므로
모든 레인이 거의 같은 조건을 같이 지나간다 — 중간에 끊어도 끝난 조건은 제어기 비교가 완결돼 있다.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import glob
import hashlib
import json
import os
from pathlib import Path
import sys

PLAN_SCHEMA = 'sensor8-plan/1'
LANES_SCHEMA = 'sensor8-lanes/1'
BASE = 'configs/arena_tune8.json'
BASE_SHA = 'f0bb414658361f68e29b838425c0485074c6c32b339c83921162485dc8118767'
TEMPLATE = 'configs/main_experiment_sensor_candidate_v6.json'
STRESS = 'configs/sensors/stress.json'
RUN_DIR = 'results/arena/tuning/tune8_final'
CONTROLLERS = ('V13', 'F13', 'GSLQR', 'CPID')
# main8이 쓴 tune8 최종 기록(PC-63 V13·CPID, PC-62 F13, PC-64 GSLQR의 main8 spec에 적힌 값)
RECORDS = {
    'V13': '81bee33b2cc36352fa185e2000fa429497beed050b5380424f9034542546180f',
    'F13': '3d553662c5be20258a4672dd6f9851270f51b0e9c934733959c706f9efe29553',
    'GSLQR': '00c00d9ffdfc3d3f7c756d622ee4dfa1e64b0e20b0ed7048ed34ef9e42536ccf',
    'CPID': '61c9c555f798adac356fc8640faf8a4c1948b0e9606f33d82610aa3d9ee9ff2a',
}
SEEDS = list(range(1020, 1040))     # main8 비교 시드 — 같은 시드 = 같은 잡음 수열이라 조건끼리·main8과 짝이 된다
TRACE_SEED = 1020                   # 이 시드(와 truth)만 궤적 npz를 남긴다 — 그림용, 디스크 절약

CORE = ['main_gust_lateral_+10_V_H', 'main_gust_vertical_-5_V_H', 'main_ref_accel_VH_rho1',
        'main_ref_brake_VH_rho1', 'main_gust_lateral_+10_V_L']
MISMATCH = ['main_t7_cruise_VH__moment_25', 'main_t7_cruise_VH__mass_1.3']

# 조각 나누기용 예상 시간(s) — main8 Windows 실측 평균(V13·CPID PC-63, F13 PC-62, GSLQR PC-64)
EST = {
    'main_gust_lateral_+10_V_H': dict(V13=126, F13=296, GSLQR=38),
    'main_gust_vertical_-5_V_H': dict(V13=126, F13=287, GSLQR=37),
    'main_ref_accel_VH_rho1': dict(V13=141, F13=280, GSLQR=32),
    'main_ref_brake_VH_rho1': dict(V13=516, F13=1420, GSLQR=92),
    'main_gust_lateral_+10_V_L': dict(V13=126, F13=301, GSLQR=37, CPID=41),
    'main_t7_cruise_VH__moment_25': dict(V13=151, F13=368, GSLQR=41),
    'main_t7_cruise_VH__mass_1.3': dict(V13=151, F13=368, GSLQR=41),
}

# 돌풍·기동은 모두 3.0 s에 시작한다(gust times_s [3,2,7], reference lead_s 3, 표7 disturbance_start_s 3).
# 끊김·튐 구간은 그 0.5 s 뒤부터(센서 envelope 화면과 같은 위치).
VIB = [dict(frequency_hz=150.0, amplitude_rad_s=[0.1, 0.1, 0.1], phase_rad=[0.0, 2.0944, 4.1888])]

# (이름, 묶음, 시나리오, 바꾸는 값 [(경로, 값)], 설명). 경로는 센서 프로파일 안의 'imu.latency_s' 식.
# 'feedback': 'truth'는 센서 없이 참 상태 — 결정적이라 시드 없이 한 번만.
CONDITIONS = [
    dict(name='truth', tier=0, scenarios=CORE+MISMATCH, feedback='truth', changes=[],
         note='참 상태 피드백(센서·추정기 없음). 제어기 자체 한계와 센서 영향을 가른다.'),
    dict(name='nominal', tier=0, scenarios=CORE, changes=[],
         note='main8 기준 프로파일 그대로. main8 재현 확인 겸 같은 PC 기준선.'),
    # 1순위 — INDI 경로(IMU·rpm)
    dict(name='imu_latency_2ms', tier=1, scenarios=CORE, changes=[('imu.latency_s', 0.002)],
         note='IMU 전송 지연 2 ms(보상 없음)'),
    dict(name='imu_latency_5ms', tier=1, scenarios=CORE, changes=[('imu.latency_s', 0.005)],
         note='IMU 전송 지연 5 ms(보상 없음)'),
    dict(name='imu_latency_10ms', tier=1, scenarios=CORE, changes=[('imu.latency_s', 0.010)],
         note='IMU 전송 지연 10 ms(보상 없음, table7 state_delay_10ms와 비교)'),
    dict(name='rpm_latency_10ms', tier=1, scenarios=CORE, changes=[('rpm.latency_s', 0.010)],
         note='rpm 텔레메트리 지연 4 -> 10 ms(observer가 타임스탬프로 보상)'),
    dict(name='rpm_latency_20ms', tier=1, scenarios=CORE, changes=[('rpm.latency_s', 0.020)],
         note='rpm 텔레메트리 지연 4 -> 20 ms(보상)'),
    dict(name='rpm_latency_40ms', tier=1, scenarios=CORE, changes=[('rpm.latency_s', 0.040)],
         note='rpm 텔레메트리 지연 4 -> 40 ms(보상, observer max_age 50 ms 안)'),
    dict(name='rpm_rate_100hz', tier=1, scenarios=CORE, changes=[('rpm.rate_hz', 100.0)],
         note='rpm 텔레메트리 500 -> 100 Hz'),
    dict(name='rpm_outage_200ms', tier=1, scenarios=CORE, changes=[('rpm.outage_windows', [[3.5, 3.7]])],
         note='rpm 끊김 3.5~3.7 s(observer는 모델 예측만)'),
    dict(name='gyro_noise_x3', tier=1, scenarios=CORE, changes=[('imu.gyro_noise_density', 0.006)],
         note='gyro 백색잡음 밀도 0.002 -> 0.006'),
    dict(name='gyro_noise_x5', tier=1, scenarios=CORE, changes=[('imu.gyro_noise_density', 0.010)],
         note='gyro 백색잡음 밀도 0.002 -> 0.010(stress 프로파일 값)'),
    dict(name='gyro_vibration_150hz', tier=1, scenarios=CORE, changes=[('imu.gyro_vibration', VIB)],
         note='gyro 진동 150 Hz, 축마다 0.1 rad/s(크기는 임의값 — 데이터시트 근거 아님)'),
    dict(name='rpm_noise_x4', tier=1, scenarios=CORE, changes=[('rpm.sigma', 32.0)],
         note='rpm 잡음 8 -> 32 rad/s'),
    # 2순위 — 항법 센서(외부 루프)
    dict(name='gnss_noise_x3', tier=2, scenarios=CORE,
         changes=[('gnss.pos_sigma', 2.4), ('gnss.vel_sigma', 0.45)],
         note='GNSS 잡음 0.8 m / 0.15 m/s -> 2.4 / 0.45(추정기 가정은 그대로)'),
    dict(name='gnss_latency_300ms', tier=2, scenarios=CORE, changes=[('gnss.latency_s', 0.30)],
         note='GNSS 지연 0.12 -> 0.30 s'),
    dict(name='gnss_outage_3s', tier=2, scenarios=CORE, changes=[('gnss.outage_windows', [[3.5, 6.5]])],
         note='GNSS 끊김 3.5~6.5 s'),
    dict(name='gnss_outlier_3m', tier=2, scenarios=CORE,
         changes=[('gnss.outlier_windows', [dict(start_s=3.5, end_s=4.0, pos_offset_m=[0.0, 0.0, 3.0])])],
         note='GNSS 높이 튐 +3 m, 3.5~4.0 s(크기는 임의값)'),
    dict(name='baro_noise_x4', tier=2, scenarios=CORE, changes=[('barometer.sigma', 1.0)],
         note='기압계 잡음 0.25 -> 1.0 m'),
    dict(name='baro_drift_5cm_s', tier=2, scenarios=CORE, changes=[('barometer.bias_rate_m_s', 0.05)],
         note='기압계 바이어스 드리프트 0.05 m/s'),
    dict(name='initial_error', tier=2, scenarios=CORE,
         changes=[('estimator.initial_position_error_m', [0.2, -0.1, 0.3]),
                  ('estimator.initial_velocity_error_m_s', [0.1, -0.1, 0.05]),
                  ('estimator.initial_attitude_error_deg', [0.5, -0.5, 1.0])],
         note='초기 추정 오차(센서 envelope 화면과 같은 값)'),
    # 3순위 — 묶음
    dict(name='degraded', tier=3, scenarios=CORE,
         changes=[('imu.gyro_noise_density', 0.006), ('rpm.latency_s', 0.010),
                  ('gnss.pos_sigma', 1.6), ('gnss.vel_sigma', 0.30), ('gnss.latency_s', 0.20),
                  ('barometer.sigma', 0.5)],
         note='중간 수준을 한꺼번에: gyro x3, rpm 10 ms, GNSS 잡음 x2·지연 0.2 s, 기압계 x2'),
    dict(name='stress', tier=3, scenarios=CORE, changes='stress',
         note='configs/sensors/stress.json의 센서 값(imu·gnss·기압계·지자기·rpm). 추정기·observer는 기준 그대로'),
    # 4순위 — 모델 오차 x 센서(V_H 순항 moment_25·mass_1.3, main8에서 셋 다 통과)
    dict(name='mismatch_nominal', tier=4, scenarios=MISMATCH, changes=[],
         note='모델 오차 시나리오의 같은 PC 기준선(기준 프로파일)'),
    dict(name='mismatch_gyro_noise_x3', tier=4, scenarios=MISMATCH, changes=[('imu.gyro_noise_density', 0.006)],
         note='모델 오차 + gyro x3'),
    dict(name='mismatch_rpm_latency_20ms', tier=4, scenarios=MISMATCH, changes=[('rpm.latency_s', 0.020)],
         note='모델 오차 + rpm 지연 20 ms'),
    dict(name='mismatch_imu_latency_2ms', tier=4, scenarios=MISMATCH, changes=[('imu.latency_s', 0.002)],
         note='모델 오차 + IMU 지연 2 ms'),
]


def _utc():
    return datetime.now(timezone.utc).isoformat()


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    return sha256_bytes(Path(path).read_bytes())


def canonical(obj):
    return json.dumps(obj, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False)


def write_atomic(path, text):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(text, encoding='utf-8')
    os.replace(tmp, path)


def safe_name(trial_id):
    return trial_id.replace('/', '__').replace(':', '_')


def runner_sha256():
    return sha256_file(__file__)


def use_root(root):
    root = Path(root).resolve()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    return root


# ══════════════════════════════════════════════════════════════════
# 계획
# ══════════════════════════════════════════════════════════════════

def apply_changes(profile, changes, root):
    p = deepcopy(profile)
    if changes == 'stress':
        stress = json.loads((root/STRESS).read_text(encoding='utf-8'))
        for block in ('imu', 'gnss', 'barometer', 'magnetometer', 'rpm'):
            p[block].update(deepcopy(stress[block]))
        return p
    for path, value in changes:
        block, key = path.split('.')
        if block not in p:
            raise ValueError(f'profile has no block {block}')
        p[block][key] = deepcopy(value)
    return p


def check_records(root):
    problems = []
    for label, sha in RECORDS.items():
        path = root/RUN_DIR/f'{label}.record.json'
        if not path.exists():
            problems.append(f'{label}: {path} missing')
        elif sha256_file(path) != sha:
            problems.append(f'{label}: {path} sha256 {sha256_file(path)[:12]} is not the main8 record {sha[:12]}')
    return problems


def scenario_dicts(root, base, native):
    """main8과 같은 시나리오 정의 — main8 spec 템플릿을 control.main_experiment로 전개해 id로 고른다."""
    from control import main_experiment as me
    spec = json.loads((root/TEMPLATE).read_text(encoding='utf-8'))
    spec['controllers'] = list(CONTROLLERS)
    spec['ladder'] = dict(spec['ladder'], variants={})
    pool = {}
    for batch in me.build_batches(spec, base, native):
        for s in batch.config['scenarios']:
            pool[s['id']] = s
    wanted = CORE + MISMATCH
    missing = [sid for sid in wanted if sid not in pool]
    if missing:
        raise SystemExit(f'scenarios not in the main8 template: {missing}')
    return spec, [deepcopy(pool[sid]) for sid in wanted]


def build_plan(root):
    root = use_root(root)
    from control.arena import load_config, config_sha256, build_scenarios, excluded_from
    from control.arena_factory import ArenaFactory
    from control.arena_sensors import load_sensor_profile
    from control.sensor_binding import resolve_feedback
    from control.validation_suite import baseline_params, source_hashes
    from control import main_experiment as me
    problems = check_records(root)
    if problems:
        raise SystemExit('records:\n  - ' + '\n  - '.join(problems))
    base = load_config(root/BASE)
    if config_sha256(base) != BASE_SHA:
        raise SystemExit(f'{BASE}: hash {config_sha256(base)[:12]} is not the tune8 config')
    lo, hi = base['seeds']['main']
    tlo, thi = base['seeds']['tuning']
    if any(not lo <= s <= hi or tlo <= s <= thi for s in SEEDS):
        raise SystemExit('sensor8 seeds must be main-range seeds outside the tuning range')
    native = baseline_params()
    spec, scen = scenario_dicts(root, base, native)
    config = me._derived(base, scen)
    cp = ArenaFactory(base, native).cp
    built = {s.id: s for s in build_scenarios(config, cp, native)}
    nominal = resolve_feedback(base)
    if nominal.mode != 'sensors':
        raise SystemExit('tune8 config is not a sensor-feedback config')
    conditions, trials = [], []
    names = [c['name'] for c in CONDITIONS]
    if len(names) != len(set(names)):
        raise SystemExit('duplicate condition names')
    for order, cond in enumerate(CONDITIONS):
        entry = dict(name=cond['name'], tier=cond['tier'], order=order, note=cond['note'],
                     scenarios=list(cond['scenarios']), feedback=cond.get('feedback', 'sensors'),
                     changes=cond['changes'])
        seeds = [None]
        if entry['feedback'] == 'sensors':
            profile = apply_changes(nominal.profile, cond['changes'], root)
            if cond['changes']:
                profile['name'] = f"{nominal.profile['name']}+{cond['name']}"
            profile = load_sensor_profile(profile)
            entry['profile'] = profile
            entry['profile_sha256'] = resolve_feedback(base, mode='sensors', profile=profile).metadata['sensor_profile_sha256']
            seeds = SEEDS
        excluded = []
        for sid in cond['scenarios']:
            s = built[sid]
            for label in CONTROLLERS:
                reason = excluded_from(config, label, s)
                if reason:
                    excluded.append(dict(scenario_id=sid, controller=label, reason=reason))
                    continue
                for seed in seeds:
                    tid = f"{cond['name']}/{sid}/{label}" + ('' if seed is None else f'/seed_{seed}')
                    trials.append(dict(trial_id=tid, condition=cond['name'], order=order, scenario_id=sid,
                                       controller=label, sensor_seed=seed, est_seconds=EST[sid][label]))
        entry['excluded'] = excluded
        conditions.append(entry)
    plan = dict(schema=PLAN_SCHEMA,
                purpose='sensor8: tune8 gains, sensor conditions varied one at a time (estimator settings fixed), '
                        'paired seeds 1020-1039, controllers V13 F13 GSLQR (+CPID where its design region allows).',
                base_config=dict(path=BASE, config_sha256=BASE_SHA),
                template=dict(path=TEMPLATE, sha256=sha256_file(root/TEMPLATE)),
                tuned=dict(run_dir=RUN_DIR, records=RECORDS),
                derived_config_sha256=config_sha256(config),
                controllers=list(CONTROLLERS), seeds=SEEDS, trace_seed=TRACE_SEED,
                nominal_profile_sha256=nominal.metadata['sensor_profile_sha256'],
                source_sha256=source_hashes(), runner_sha256=runner_sha256(),
                conditions=conditions, trials=trials)
    return plan


def plan_path(out):
    return Path(out)/'sensor8.plan.json'


def load_plan(out):
    path = plan_path(out)
    plan = json.loads(path.read_text(encoding='utf-8'))
    if plan.get('schema') != PLAN_SCHEMA:
        raise SystemExit(f'{path}: not a sensor8 plan')
    return plan, sha256_file(path)


def cmd_plan(root, out):
    plan = build_plan(root)
    text = json.dumps(plan, ensure_ascii=False, indent=1) + '\n'
    path = plan_path(out)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.read_text(encoding='utf-8') != text:
            raise SystemExit(f'{path} exists and differs from the plan this script makes now '
                             '(script, records or source changed) - use a new output folder')
        print(f'plan unchanged: {path}')
    else:
        write_atomic(path, text)
        print(f'wrote {path}')
    by_tier = {}
    for t in plan['trials']:
        tier = next(c['tier'] for c in plan['conditions'] if c['name'] == t['condition'])
        by_tier.setdefault(tier, [0, 0.0])
        by_tier[tier][0] += 1
        by_tier[tier][1] += t['est_seconds']
    for tier, (n, sec) in sorted(by_tier.items()):
        print(f'tier {tier}: {n} trials, {sec/3600:.1f} worker-hours')
    print(f"total {len(plan['trials'])} trials, {sum(t['est_seconds'] for t in plan['trials'])/3600:.1f} worker-hours, "
          f'plan sha256 {sha256_file(path)}')


# ══════════════════════════════════════════════════════════════════
# 레인(작업자 하나가 맡는 시행 목록)
# ══════════════════════════════════════════════════════════════════

def lanes_path(out, pc):
    return Path(out)/f'{pc}.lanes.json'


def cmd_lanes(out, pc, n):
    plan, plan_sha = load_plan(out)
    path = lanes_path(out, pc)
    loads = [0.0]*n
    lanes = [[] for _ in range(n)]
    for order in sorted({t['order'] for t in plan['trials']}):
        group = [t for t in plan['trials'] if t['order'] == order]
        # 조건 하나씩: 긴 시행부터 지금까지 가장 덜 찬 레인에(LPT). 동률은 번호가 작은 레인 — 결정적.
        for t in sorted(group, key=lambda t: (-t['est_seconds'], t['trial_id'])):
            k = min(range(n), key=lambda i: (loads[i], i))
            lanes[k].append(t['trial_id'])
            loads[k] += t['est_seconds']
    doc = dict(schema=LANES_SCHEMA, plan_sha256=plan_sha, pc=pc, n=n,
               lanes=[dict(index=i, est_hours=loads[i]/3600.0, trials=lanes[i]) for i in range(n)])
    text = json.dumps(doc, ensure_ascii=False, indent=1) + '\n'
    if path.exists():
        old = json.loads(path.read_text(encoding='utf-8'))
        if old.get('plan_sha256') != plan_sha or old.get('n') != n or old.get('lanes') != doc['lanes']:
            raise SystemExit(f'{path} exists for another plan or lane count - keep it, or use a new output folder')
        print(f'lanes unchanged: {path}')
    else:
        write_atomic(path, text)
        print(f'wrote {path}')
    for lane in doc['lanes']:
        print(f"lane {lane['index']}: {len(lane['trials'])} trials, ~{lane['est_hours']:.1f} h")


def load_lanes(out, pc):
    path = lanes_path(out, pc)
    doc = json.loads(path.read_text(encoding='utf-8'))
    if doc.get('schema') != LANES_SCHEMA:
        raise SystemExit(f'{path}: not a sensor8 lanes file')
    return doc


# ══════════════════════════════════════════════════════════════════
# 실행
# ══════════════════════════════════════════════════════════════════

class Runner:
    def __init__(self, root, plan, plan_sha):
        self.root = use_root(root)
        from control.arena import load_config, config_sha256, build_scenarios
        from control.arena_factory import ArenaFactory
        from control.arena_design_check import tuned_overrides
        from control.validation_suite import baseline_params, source_hashes
        from control import main_experiment as me
        self.plan, self.plan_sha = plan, plan_sha
        problems = check_records(self.root)
        base = load_config(self.root/BASE)
        if config_sha256(base) != plan['base_config']['config_sha256']:
            problems.append('base config hash differs from the plan')
        if source_hashes() != plan['source_sha256']:
            problems.append('control/configs source hashes differ from the plan')
        if runner_sha256() != plan['runner_sha256']:
            problems.append('this sensor8_run.py differs from the one that made the plan')
        if problems:
            raise SystemExit('refusing to run:\n  - ' + '\n  - '.join(problems))
        self.base, self.native = base, baseline_params()
        self.spec, scen = scenario_dicts(self.root, base, self.native)
        self.config = me._derived(base, scen)
        if config_sha256(self.config) != plan['derived_config_sha256']:
            raise SystemExit('derived scenario config differs from the plan')
        self.config_sha = config_sha256(self.config)
        cp = ArenaFactory(base, self.native).cp
        self.scenarios = {s.id: s for s in build_scenarios(self.config, cp, self.native)}
        self.overrides, self.used = tuned_overrides(base, self.root/RUN_DIR, list(CONTROLLERS))
        self.factory = ArenaFactory(self.config, self.native, overrides=self.overrides)
        self.conditions = {c['name']: c for c in plan['conditions']}
        self.trials = {t['trial_id']: t for t in plan['trials']}
        self.source = plan['source_sha256']
        self._trim = {}

    def hashes(self, label):
        return dict(plan_sha256=self.plan_sha, base_config_sha256=self.plan['base_config']['config_sha256'],
                    derived_config_sha256=self.config_sha, tuned_record_sha256=RECORDS[label],
                    runner_sha256=self.plan['runner_sha256'])

    def run_trial(self, trial):
        """main_distributed.Campaign.run_trial과 같은 순서·판정. 다른 것은 feedback/sensor_profile 인자뿐."""
        import numpy as np
        from control.arena import excluded_from
        from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
        from control.validation_suite import run_trial as suite_run
        from control import main_experiment as me
        cond = self.conditions[trial['condition']]
        s = self.scenarios[trial['scenario_id']]
        label = trial['controller']
        if s.id not in self._trim:
            self._trim[s.id] = me.trim_status(self.native, s)
        status, detail = self._trim[s.id]
        row = dict(trial_id=trial['trial_id'], condition=cond['name'], tier=cond['tier'],
                   scenario_id=s.id, scenario_type=s.type, controller=label, case=s.cases[0],
                   window=list(s.window), sensor_seed=trial['sensor_seed'], **self.hashes(label),
                   trim_detail={f'{v:g}': d for v, d in detail.items()},
                   authority_boundary=me.authority_boundary(detail, self.spec, self.base))
        reason = excluded_from(self.config, label, s)
        if reason or status:
            row.update(skipped=True, skip_reason=reason or 'vehicle_limit')
            return row, None
        limits = Acceptance(**self.config['acceptance'])
        paper = PaperCriteria(**self.config['paper_criteria'])
        kwargs = (dict(feedback='truth') if cond['feedback'] == 'truth' else
                  dict(feedback='sensors', sensor_profile=cond['profile'], sensor_seed=trial['sensor_seed']))
        try:
            metrics, result, log = suite_run(self.factory, label, s.profile, s.cases[0], limits, **kwargs)
            metrics.update(paper_evaluate(result, s.profile, paper, window=s.window, solve_log=log,
                                          n_max=self.native['n_max']))
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            metrics = dict(passed=False, failure_reasons=['setup_error'], stop_reason=f'{type(exc).__name__}: {exc}')
            result = None
        row.update(skipped=False, skip_reason=None, **metrics)
        row['estimation_error'] = estimation_error(result)
        if (cond['feedback'] == 'sensors' and result is not None
                and row.get('sensor_profile_sha256') != cond['profile_sha256']):
            raise SystemExit(f"{trial['trial_id']}: ran with sensor profile {row.get('sensor_profile_sha256')}, "
                             f"plan says {cond['profile_sha256']}")
        return row, result


def estimation_error(result):
    """추정 상태(제어기가 받은 값) 대 참 상태 오차 — 전 구간 RMS와 최대. 센서 시행에만 있다."""
    import numpy as np
    if result is None or result.get('xs_est') is None:
        return None
    n = min(len(result['xs']), len(result['xs_est']))
    if n == 0:
        return None
    x, e = np.asarray(result['xs'][:n], dtype=float), np.asarray(result['xs_est'][:n], dtype=float)

    def norms(a):
        return np.linalg.norm(a, axis=1)
    qx = x[:, 6:10]/np.linalg.norm(x[:, 6:10], axis=1, keepdims=True)
    qe = e[:, 6:10]/np.linalg.norm(e[:, 6:10], axis=1, keepdims=True)
    angle = np.degrees(2.0*np.arccos(np.clip(np.abs(np.sum(qx*qe, axis=1)), 0.0, 1.0)))
    out = dict(samples=int(n))
    for name, part in (('position_m', norms(x[:, 0:3]-e[:, 0:3])), ('velocity_m_s', norms(x[:, 3:6]-e[:, 3:6])),
                       ('attitude_deg', angle), ('body_rate_rad_s', norms(x[:, 10:13]-e[:, 10:13])),
                       ('rotor_rad_s', norms(x[:, 13:17]-e[:, 13:17])/2.0)):
        out[name] = dict(rms=float(np.sqrt(np.mean(part**2))), max=float(np.max(part)))
    out['note'] = 'rotor_rad_s is the 4-rotor norm / 2 (RMS over rotors); all over the whole run'
    return out


def lane_dir(out, pc, index):
    return Path(out)/pc/f'lane{index}'/'trials'


def cmd_run_lane(root, out, pc, index, max_trials=None):
    import numpy as np
    plan, plan_sha = load_plan(out)
    lanes = load_lanes(out, pc)
    if lanes['plan_sha256'] != plan_sha:
        raise SystemExit('lanes file was made for another plan')
    runner = Runner(root, plan, plan_sha)
    from control.main_distributed import machine_fingerprint
    from control.validation_suite import json_safe, git_state
    folder = lane_dir(out, pc, index)
    folder.mkdir(parents=True, exist_ok=True)
    traces = Path(out)/pc/'traces'
    traces.mkdir(parents=True, exist_ok=True)
    for stale in glob.glob(str(folder/'*.tmp')) + glob.glob(str(traces/f'*.lane{index}.tmp')):
        os.remove(stale)
        print(f'removed interrupted file {Path(stale).name}', flush=True)
    revision, dirty = git_state()
    machine = machine_fingerprint()
    ran = 0
    for trial_id in lanes['lanes'][index]['trials']:
        trial = runner.trials[trial_id]
        path = folder/(safe_name(trial_id)+'.json')
        if path.exists():
            done = json.loads(path.read_text(encoding='utf-8'))
            want = runner.hashes(trial['controller'])
            if {k: done.get(k) for k in want} != want or done.get('source_sha256') != runner.source:
                raise SystemExit(f'{path}: finished with different hashes - refusing to mix results')
            continue
        if max_trials is not None and ran >= max_trials:
            break
        started = _utc()
        try:
            row, result = runner.run_trial(trial)
        except Exception as exc:        # 한 시행의 예외로 레인 전체가 멈추지 않게 — 실패 행으로 남기고 다음으로
            import traceback
            row = dict(trial_id=trial_id, condition=trial['condition'], scenario_id=trial['scenario_id'],
                       controller=trial['controller'], sensor_seed=trial['sensor_seed'],
                       **runner.hashes(trial['controller']), skipped=False, skip_reason=None, passed=False,
                       tracking_pass=False, model_domain_valid=False, failure_reasons=['runner_exception'],
                       stop_reason=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc()[-4000:])
            result = None
        if result is not None and (trial['sensor_seed'] in (None, TRACE_SEED)):
            npz = traces/(safe_name(trial_id)+'.npz')
            tmp = traces/(safe_name(trial_id)+f'.lane{index}.tmp')
            with open(tmp, 'wb') as stream:
                np.savez_compressed(stream, **{k: v for k, v in result.items()
                                               if k not in ('error', 'navigation_trace')})
            os.replace(tmp, npz)
        row.update(lane=index, machine=machine, git_revision=revision, git_dirty=dirty,
                   source_sha256=runner.source, started_utc=started, finished_utc=_utc())
        write_atomic(path, json.dumps(json_safe(row), ensure_ascii=False, allow_nan=False))
        ran += 1
        verdict = 'SKIP ' + row['skip_reason'] if row['skipped'] else row.get('passed')
        print(f'[{index}] {trial_id}: {verdict}', flush=True)


# ══════════════════════════════════════════════════════════════════
# 진행·요약
# ══════════════════════════════════════════════════════════════════

def done_rows(out, pc):
    rows = {}
    for path in glob.glob(str(Path(out)/pc/'lane*'/'trials'/'*.json')):
        row = json.loads(Path(path).read_text(encoding='utf-8'))
        rows[row['trial_id']] = row
    return rows


def cmd_status(out, pc, done_conditions=False):
    plan, _ = load_plan(out)
    have = {safe_name(Path(p).stem) for p in glob.glob(str(Path(out)/pc/'lane*'/'trials'/'*.json'))}
    total = {}
    for t in plan['trials']:
        n = total.setdefault(t['condition'], [0, 0])
        n[0] += 1
        n[1] += safe_name(t['trial_id']) in have
    if done_conditions:
        print(' '.join(c['name'] for c in plan['conditions'] if total[c['name']][1] == total[c['name']][0]))
        return
    for c in plan['conditions']:
        n, d = total[c['name']]
        print(f"tier {c['tier']} {c['name']}: {d}/{n}")
    print(f"all: {sum(v[1] for v in total.values())}/{len(plan['trials'])}")


def cmd_summary(out, pc):
    plan, plan_sha = load_plan(out)
    rows = done_rows(out, pc)
    counts = {}
    for t in plan['trials']:
        key = (t['order'], t['condition'], t['scenario_id'], t['controller'])
        c = counts.setdefault(key, dict(planned=0, done=0, tracking=0, domain=0, full=0, optimizer_stop=0,
                                        diverged=0, setup_error=0))
        c['planned'] += 1
        row = rows.get(t['trial_id'])
        if row is None or row.get('skipped'):
            continue
        c['done'] += 1
        c['tracking'] += bool(row.get('tracking_pass'))
        c['domain'] += bool(row.get('model_domain_valid'))
        c['full'] += bool(row.get('passed'))
        reason = row.get('stop_reason') or ''
        c['optimizer_stop'] += reason.startswith('5 consecutive optimizer failures')
        c['diverged'] += reason.startswith('body-rate or altitude divergence')
        c['setup_error'] += bool({'setup_error', 'runner_exception'} & set(row.get('failure_reasons') or []))
    lines = [f'# sensor8 summary ({pc})', '',
             f'plan sha256 {plan_sha[:12]}, {len(rows)}/{len(plan["trials"])} trials done, {_utc()[:16]} UTC', '',
             'Cell: tracking pass / trials done, "of N" while unfinished, "-" not started. Truth runs once.',
             'Full pass (tracking and propulsion-model domain), stops and divergences are in summary.csv.', '']
    for sid in CORE + MISMATCH:
        labels = [l for l in CONTROLLERS if any(k[2] == sid and k[3] == l for k in counts)]
        lines += [f'## {sid.replace("main_", "")}', '', '| condition | ' + ' | '.join(labels) + ' |',
                  '|---|' + '---:|'*len(labels)]
        for c in plan['conditions']:
            if sid not in c['scenarios']:
                continue
            cells = []
            for label in labels:
                v = counts.get((c['order'], c['name'], sid, label))
                if v is None:
                    cells.append('')
                elif v['done'] == 0:
                    cells.append('-')
                else:
                    cells.append(f"{v['tracking']}/{v['done']}" + ('' if v['done'] == v['planned'] else f" of {v['planned']}"))
            lines.append(f"| {c['name']} | " + ' | '.join(cells) + ' |')
        lines.append('')
    csv = ['order,condition,scenario_id,controller,planned,done,tracking_pass,model_domain_valid,full_pass,'
           'optimizer_stop,diverged,setup_error']
    for (order, cond, sid, label), v in sorted(counts.items()):
        csv.append(f"{order},{cond},{sid},{label},{v['planned']},{v['done']},{v['tracking']},{v['domain']},"
                   f"{v['full']},{v['optimizer_stop']},{v['diverged']},{v['setup_error']}")
    folder = Path(out)/pc
    folder.mkdir(parents=True, exist_ok=True)
    write_atomic(folder/'summary.md', '\n'.join(lines) + '\n')
    write_atomic(folder/'summary.csv', '\n'.join(csv) + '\n')
    print(f'wrote {folder/"summary.md"} and summary.csv ({len(rows)} trials)')


def cmd_collect(out, pc, zip_path, logs=None, traces=False):
    """요약을 새로 쓰고, 끝난 행 전부 + 요약 + 계획 + 레인 + 로그를 zip 하나로. npz는 --traces일 때만."""
    import zipfile
    cmd_summary(out, pc)
    out = Path(out)
    files = [plan_path(out), lanes_path(out, pc), out/pc/'summary.md', out/pc/'summary.csv']
    files += sorted(Path(p) for p in glob.glob(str(out/pc/'lane*'/'trials'/'*.json')))
    if traces:
        files += sorted(Path(p) for p in glob.glob(str(out/pc/'traces'/'*.npz')))
    zip_path = Path(zip_path)
    tmp = zip_path.with_name(zip_path.name + '.tmp')
    with zipfile.ZipFile(tmp, 'w', compression=zipfile.ZIP_DEFLATED) as z:
        for f in files:
            if f.exists():
                z.write(f, arcname=f.relative_to(out).as_posix())
        if logs and Path(logs).exists():
            for f in sorted(Path(logs).iterdir()):
                if f.suffix in ('.log', '.err'):
                    z.write(f, arcname=f'logs/{f.name}')
    os.replace(tmp, zip_path)
    print(f'wrote {zip_path} ({zip_path.stat().st_size/1e6:.1f} MB)')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('plan')
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p = sub.add_parser('lanes')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pc', required=True)
    p.add_argument('--n', type=int, required=True)
    p = sub.add_parser('run-lane')
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pc', required=True)
    p.add_argument('--index', type=int, required=True)
    p.add_argument('--max-trials', type=int, help='tests only')
    p = sub.add_parser('status')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pc', required=True)
    p.add_argument('--done-conditions', action='store_true')
    p = sub.add_parser('summary')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pc', required=True)
    p = sub.add_parser('collect')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pc', required=True)
    p.add_argument('--zip', type=Path, required=True)
    p.add_argument('--logs', type=Path)
    p.add_argument('--traces', action='store_true')
    a = parser.parse_args(argv)
    if a.cmd == 'plan':
        cmd_plan(a.root, a.out)
    elif a.cmd == 'lanes':
        if a.n < 1:
            raise SystemExit('--n must be at least 1')
        cmd_lanes(a.out, a.pc, a.n)
    elif a.cmd == 'run-lane':
        cmd_run_lane(a.root, a.out, a.pc, a.index, a.max_trials)
    elif a.cmd == 'status':
        cmd_status(a.out, a.pc, a.done_conditions)
    elif a.cmd == 'summary':
        cmd_summary(a.out, a.pc)
    elif a.cmd == 'collect':
        cmd_collect(a.out, a.pc, a.zip, a.logs, a.traces)
    return 0


if __name__ == '__main__':
    sys.exit(main())
