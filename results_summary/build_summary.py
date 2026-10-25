"""Build results_summary/ from the raw per-trial JSON rows of main8, sensor8, ablation8 and weightcheck8.

Raw rows (one JSON per trial) are not in git (bulky). This script reads them, writes one compact CSV per
experiment (data/), an aggregate pass-count CSV, markdown tables (tables/), and fills README.md from
README.template.md ({{name}} markers). Standard library only.

    python results_summary/build_summary.py \
        --main8 <dir with results/main8/*/shard*/trials/*.json> [--main8 <dir> ...] \
        --sensor8 <dir with lane*/trials/*.json> \
        --ablation8 results/ablation8/CLOUD --weightcheck8 results/weightcheck8/CLOUD results/weightcheck8b/CLOUD
"""
import argparse
import csv
import glob
import json
from collections import defaultdict
from math import comb
from pathlib import Path
from statistics import median

HERE = Path(__file__).resolve().parent
SEEDS = list(range(1020, 1040))


# ─────────────────────────────────────────────── loading
def load_dir(pattern):
    rows = []
    for f in sorted(glob.glob(pattern, recursive=True)):
        if f.endswith('.navigation.json'):
            continue
        rows.append(json.loads(Path(f).read_text(encoding='utf-8')))
    return rows


def load_all(a):
    exp = {}
    m8 = []
    for d in a.main8:
        m8 += load_dir(str(Path(d)/'results'/'main8'/'*'/'shard*'/'trials'/'*.json'))
    exp['main8'] = [r for r in m8 if not str(r.get('batch', '')).startswith('ladder')]
    exp['main8_ladder'] = [r for r in m8 if str(r.get('batch', '')).startswith('ladder')]
    exp['sensor8'] = load_dir(str(Path(a.sensor8)/'lane*'/'trials'/'*.json'))
    exp['ablation8'] = load_dir(str(Path(a.ablation8)/'lane*'/'trials'/'*.json'))
    wc = []
    for d in a.weightcheck8:
        wc += load_dir(str(Path(d)/'lane*'/'trials'/'*.json'))
    exp['weightcheck8'] = wc
    return exp


# ─────────────────────────────────────────────── per-trial CSV
COLS = ['experiment', 'trial_id', 'group', 'scenario_id', 'scenario_type', 'controller', 'variant', 'sensor_seed',
        'skipped', 'tracking_pass', 'full_pass', 'model_domain_valid', 'prop_domain_outside_fraction',
        'failure_reasons', 'stop_reason', 'window_rmse_velocity', 'window_rmse_z', 'window_max_velocity_error',
        'window_max_z_error', 'window_max_omega', 'max_omega', 'motor_saturation_fraction', 'optimizer_calls',
        'optimizer_failures', 'simulated_seconds', 'wall_seconds', 'host', 'platform', 'git_revision',
        'est_pos_rms_m', 'est_vel_rms_m_s', 'est_att_rms_deg', 'est_rate_rms_rad_s', 'trajectory_sha256']


def group_of(name, r):
    if name in ('main8', 'main8_ladder'):
        return r.get('batch')
    return r.get('condition')


def flat(name, r):
    m = r.get('machine') or {}
    ee = r.get('estimation_error') or {}
    def e(k):
        return (ee.get(k) or {}).get('rms') if isinstance(ee, dict) else None
    reasons = r.get('failure_reasons') or []
    return dict(experiment=name, trial_id=r.get('trial_id'), group=group_of(name, r), scenario_id=r.get('scenario_id'),
                scenario_type=r.get('scenario_type'), controller=r.get('controller'),
                variant=r.get('variant') or r.get('controller'), sensor_seed=r.get('sensor_seed'),
                skipped=bool(r.get('skipped')), tracking_pass=bool(r.get('tracking_pass')),
                full_pass=bool(r.get('passed')), model_domain_valid=bool(r.get('model_domain_valid')),
                prop_domain_outside_fraction=r.get('prop_domain_outside_fraction'),
                failure_reasons=';'.join(reasons), stop_reason=r.get('stop_reason') or '',
                window_rmse_velocity=r.get('window_rmse_velocity'), window_rmse_z=r.get('window_rmse_z'),
                window_max_velocity_error=r.get('window_max_velocity_error'),
                window_max_z_error=r.get('window_max_z_error'), window_max_omega=r.get('window_max_omega'),
                max_omega=r.get('max_omega'), motor_saturation_fraction=r.get('motor_saturation_fraction'),
                optimizer_calls=r.get('optimizer_calls'), optimizer_failures=r.get('optimizer_failures'),
                simulated_seconds=r.get('simulated_seconds'), wall_seconds=r.get('wall_seconds'),
                host=m.get('hostname'), platform=m.get('system'), git_revision=r.get('git_revision'),
                est_pos_rms_m=e('position_m'), est_vel_rms_m_s=e('velocity_m_s'), est_att_rms_deg=e('attitude_deg'),
                est_rate_rms_rad_s=e('body_rate_rad_s'), trajectory_sha256=r.get('trajectory_sha256'))


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        w.writeheader()
        for r in rows:
            w.writerow({k: ('' if r[k] is None else (round(r[k], 6) if isinstance(r[k], float) else r[k]))
                        for k in COLS})


# ─────────────────────────────────────────────── helpers
def ran(rows):
    return [r for r in rows if not r['skipped']]


def npass(rows):
    return sum(r['tracking_pass'] for r in ran(rows))


def cell(rows):
    rs = ran(rows)
    return f'{npass(rs)}/{len(rs)}' if rs else '–'


def sign_p(a, b):
    n, k = a + b, min(a, b)
    return min(1.0, 2*sum(comb(n, i) for i in range(k+1))/2**n) if n else 1.0


def paired(xs, ys):
    """xs, ys: dict seed -> tracking_pass. Returns (x_only, y_only, p)."""
    common = set(xs) & set(ys)
    a = sum(xs[s] and not ys[s] for s in common)
    b = sum(ys[s] and not xs[s] for s in common)
    return a, b, sign_p(a, b), len(common)


def by_seed(rows):
    return {r['sensor_seed']: r['tracking_pass'] for r in ran(rows)}


def md_table(header, body, left=None):
    left = left or 1
    out = ['| ' + ' | '.join(header) + ' |', '|' + '|'.join(['---']*left + ['---:']*(len(header)-left)) + '|']
    out += ['| ' + ' | '.join(str(c) for c in row) + ' |' for row in body]
    return '\n'.join(out)


def fmt_p(p):
    return f'{p:.2g}' if p >= 1e-3 else f'{p:.1e}'


def sel(rows, **kw):
    return [r for r in rows if all(r[k] == v for k, v in kw.items())]


# ─────────────────────────────────────────────── tables
def t_overview(F):
    body = []
    meta = {
        'main8': ('예비 비교: 5개 제어기 × 114 시나리오', 'Windows PC-61~64'),
        'main8_ladder': ('V13 구성요소 사다리(명목·섭동 3종, 시드 10개)', 'Windows PC-63'),
        'sensor8': ('센서 조건 27개 × 핵심 시나리오 7개, V13/F13/GSLQR(+CPID)', 'Windows PC-63'),
        'ablation8': ('V13-0/1/2/V13 × 명목·IMU 5 ms × 돌풍·가속·감속', 'Linux 클라우드'),
        'weightcheck8': ('F13↔V13-0 비용 가중치 맞바꿈(IMU 5 ms) — 논문 미인용', 'Linux 클라우드'),
    }
    for k, rows in F.items():
        rs = ran(rows)
        body.append([f'`{k}`', meta[k][0], len(rows), len(rs), meta[k][1], f'`data/{k}_trials.csv`'])
    return md_table(['실험', '내용', '행 수', '실행된 시행', '실행 환경', '파일'], body, left=2)


MAIN8_GROUPS = [
    ('돌풍 V_L (4종)', lambda r: r['group'] == 'gust' and r['scenario_id'].endswith('_V_L')),
    ('돌풍 V_H (4종)', lambda r: r['group'] == 'gust' and r['scenario_id'].endswith('_V_H')),
    ('가속 0→V_L, ρ 0.5–1.5', lambda r: r['scenario_id'].startswith('main_ref_accel_VL')),
    ('가속 0→V_H, ρ 0.5–1.5', lambda r: r['scenario_id'].startswith('main_ref_accel_VH')),
    ('감속 V_H→V_L, ρ 0.5–1.5', lambda r: r['scenario_id'].startswith('main_ref_brake_VH')),
    ('통합 임무 V_H', lambda r: r['group'] == 'mission'),
    ('섭동 시험 (5 기동 × 18)', lambda r: r['group'] in ('table7', 'table7_base')),
]
CTRL = ['V13', 'F13', 'M17', 'GSLQR', 'CPID']


def t_main8_groups(rows):
    body = []
    for name, f in MAIN8_GROUPS:
        g = [r for r in rows if f(r)]
        body.append([name] + [cell(sel(g, controller=c)) for c in CTRL])
    body.append(['**전체 (114 시나리오)**'] + [f'**{cell(sel(rows, controller=c))}**' for c in CTRL])
    return md_table(['시험'] + CTRL, body)


def t_main8_rho(rows):
    body = []
    for kind, label in (('accel_VH', '가속 0→V_H'), ('brake_VH', '감속 V_H→V_L'), ('accel_VL', '가속 0→V_L')):
        for c in CTRL:
            cells = []
            for rho in ('0.5', '0.8', '1', '1.2', '1.5'):
                cells.append(cell(sel(rows, controller=c, scenario_id=f'main_ref_{kind}_rho{rho}')))
            if all(x == '–' for x in cells):
                continue
            body.append([label, c] + cells)
    return md_table(['기동', '제어기', 'ρ=0.5', '0.8', '1', '1.2', '1.5'], body)


def t_main8_by_scenario(rows):
    sids = sorted({r['scenario_id'] for r in rows})
    body = [[s.replace('main_', '')] + [cell(sel(rows, controller=c, scenario_id=s)) for c in CTRL] for s in sids]
    return md_table(['시나리오'] + CTRL, body)


def t_main8_numeric(rows):
    body = []
    for c in ('V13', 'F13', 'M17', 'GSLQR'):
        rs = ran(sel(rows, controller=c))
        fails = [r for r in rs if not r['tracking_pass']]
        stops = sum(r['stop_reason'].startswith('5 consecutive optimizer') for r in rs)
        div = sum(r['stop_reason'].startswith('body-rate or altitude divergence') for r in rs)
        calls = sum(r['optimizer_calls'] or 0 for r in rs)
        ofail = sum(r['optimizer_failures'] or 0 for r in rs)
        vh = [r for r in rs if ('V_H' in r['scenario_id'] or 'VH' in r['scenario_id'])]
        end = sum(r['stop_reason'] == '' for r in vh)
        dom = sum((not r['model_domain_valid']) for r in rs if r['tracking_pass'])
        body.append([c, f'{npass(rs)}/{len(rs)}', len(fails), stops, div,
                     f'{100*ofail/calls:.2f} %' if calls else '–', f'{end}/{len(vh)} ({100*end/len(vh):.0f} %)',
                     f'{dom}/{npass(rs)}'])
    return md_table(['제어기', '추종 통과', '실패', 'NMPC 5회 연속 실패로 중단', '발산 중단',
                     'NMPC 호출 대비 실패율', 'V_H 시행 끝까지 진행', '통과 중 프로펠러 자료 범위 이탈'], body)


def t_compute(F):
    body = []
    s8 = F['sensor8']
    for c in ('V13', 'F13', 'GSLQR', 'CPID'):
        rs = [r for r in ran(sel(s8, controller=c, group='nominal')) if r['stop_reason'] == '' and r['simulated_seconds']]
        if rs:
            body.append(['sensor8 명목, 같은 PC(PC-63)', c, len(rs), f"{median(r['wall_seconds']/r['simulated_seconds'] for r in rs):.1f}"])
    m8 = F['main8']
    for c in ('V13', 'F13', 'M17', 'GSLQR', 'CPID'):
        rs = [r for r in ran(sel(m8, controller=c, group='gust')) if r['stop_reason'] == '' and r['simulated_seconds']]
        if rs:
            hosts = ','.join(sorted({r['host'] for r in rs}))
            body.append([f'main8 명목 돌풍 ({hosts})', c, len(rs), f"{median(r['wall_seconds']/r['simulated_seconds'] for r in rs):.1f}"])
    return md_table(['측정', '제어기', '시행 수', '시뮬레이션 1 s당 계산 시간(s, 중앙값)'], body)


S8_ORDER = None


def t_sensor8(rows, scen_cols, ctrls_by_scen):
    conds = []
    for r in rows:
        if r['group'] not in conds:
            conds.append(r['group'])
    order = {}
    for r in rows:
        order.setdefault(r['group'], r['trial_id'])
    body = []
    for cnd in conds:
        g = sel(rows, group=cnd)
        if not any(r['scenario_id'] in scen_cols for r in g):
            continue
        cells = []
        for s in scen_cols:
            parts = []
            for c in ctrls_by_scen[s]:
                rs = ran(sel(g, scenario_id=s, controller=c))
                parts.append(f'{npass(rs)}' + ('' if len(rs) in (0, 20) else f'/{len(rs)}') if rs else '–')
            cells.append(' / '.join(parts))
        body.append([cnd] + cells)
    return body


def t_sensor8_core(rows):
    scen = ['main_gust_lateral_+10_V_H', 'main_gust_vertical_-5_V_H', 'main_ref_accel_VH_rho1',
            'main_ref_brake_VH_rho1', 'main_gust_lateral_+10_V_L']
    ctr = {s: ['V13', 'F13', 'GSLQR'] for s in scen}
    ctr['main_gust_lateral_+10_V_L'] = ['V13', 'F13', 'GSLQR', 'CPID']
    body = t_sensor8(rows, scen, ctr)
    head = ['조건', 'V_H 횡돌풍 +10 (V13/F13/GSLQR)', 'V_H 수직돌풍 −5', '가속 0→V_H ρ1', '감속 V_H→V_L ρ1',
            'V_L 횡돌풍 +10 (V13/F13/GSLQR/CPID)']
    return md_table(head, body)


def t_sensor8_mismatch(rows):
    scen = ['main_t7_cruise_VH__moment_25', 'main_t7_cruise_VH__mass_1.3']
    ctr = {s: ['V13', 'F13', 'GSLQR'] for s in scen}
    body = t_sensor8(rows, scen, ctr)
    return md_table(['조건', 'V_H 순항 + 모멘트 25% (V13/F13/GSLQR)', 'V_H 순항 + 질량 1.3배'], body)


def t_sensor8_tests(rows):
    body = []
    tests = [('imu_latency_5ms', 'main_gust_lateral_+10_V_H'), ('imu_latency_5ms', 'main_gust_vertical_-5_V_H'),
             ('imu_latency_5ms', 'main_ref_accel_VH_rho1'), ('imu_latency_10ms', 'main_ref_brake_VH_rho1'),
             ('gyro_noise_x3', 'main_gust_lateral_+10_V_H'), ('gyro_noise_x5', 'main_ref_brake_VH_rho1'),
             ('nominal', 'main_ref_brake_VH_rho1')]
    for cnd, sid in tests:
        v = by_seed(sel(rows, group=cnd, scenario_id=sid, controller='V13'))
        f = by_seed(sel(rows, group=cnd, scenario_id=sid, controller='F13'))
        a, b, p, n = paired(v, f)
        body.append([cnd, sid.replace('main_', ''), f'{sum(v.values())}/{len(v)}', f'{sum(f.values())}/{len(f)}',
                     a, b, fmt_p(p)])
    return md_table(['조건', '시나리오', 'V13', 'F13', 'V13만 통과한 시드', 'F13만 통과한 시드', '부호 검정 p'], body)


def t_estimation(rows):
    conds = []
    for r in rows:
        if r['group'] not in conds:
            conds.append(r['group'])
    body = []
    for cnd in conds:
        rs = [r for r in ran(sel(rows, group=cnd)) if r['est_att_rms_deg'] not in (None, '')]
        if not rs:
            continue
        body.append([cnd, len(rs)] + [f"{median(r[k] for r in rs):.3g}" for k in
                                       ('est_pos_rms_m', 'est_vel_rms_m_s', 'est_att_rms_deg', 'est_rate_rms_rad_s')])
    return md_table(['조건', '시행', '위치 (m)', '속도 (m/s)', '자세 (°)', '각속도 (rad/s)'], body)


VARS = ['V13-0', 'V13-1', 'V13-2', 'V13']


def t_ablation(rows, s8rows):
    scen = [('main_gust_lateral_+10_V_H', '횡돌풍 V_H +10'), ('main_ref_accel_VH_rho1', '가속 0→V_H ρ1'),
            ('main_ref_brake_VH_rho1', '감속 V_H→V_L ρ1')]
    head = ['구성', '배분/피드백/정렬']
    for _, lab in scen:
        head += [f'{lab} 명목', f'{lab} IMU 5 ms']
    flags = {'F13 (PC-63)': 'A0/–/S0 (NMPC: 로터 추력)', 'V13-0': 'A0/–/S0', 'V13-1': 'A1/–/S0', 'V13-2': 'A1/C1/S0',
             'V13': 'A1/C1/S1'}
    body = []
    row = ['F13 (PC-63)', flags['F13 (PC-63)']]
    for sid, _ in scen:
        for cnd in ('nominal', 'imu_latency_5ms'):
            row.append(cell(sel(s8rows, group=cnd, scenario_id=sid, controller='F13')))
    body.append(row)
    for v in VARS:
        row = [v, flags[v]]
        for sid, _ in scen:
            for cnd in ('nominal', 'imu_latency_5ms'):
                row.append(cell(sel(rows, group=cnd, scenario_id=sid, variant=v)))
        body.append(row)
    return md_table(head, body)


def t_ablation_tests(rows, s8rows):
    body = []
    pairs = [('V13-0', 'V13-1', 'A1 추가'), ('V13-1', 'V13-2', 'C1 추가'), ('V13-2', 'V13', 'S1 추가'),
             ('V13-0', 'V13', 'A1+C1+S1')]
    for cnd in ('nominal', 'imu_latency_5ms'):
        for sid in ('main_gust_lateral_+10_V_H', 'main_ref_accel_VH_rho1', 'main_ref_brake_VH_rho1'):
            for x, y, lab in pairs:
                a, b, p, n = paired(by_seed(sel(rows, group=cnd, scenario_id=sid, variant=x)),
                                    by_seed(sel(rows, group=cnd, scenario_id=sid, variant=y)))
                if a + b == 0:
                    continue
                body.append([cnd, sid.replace('main_', ''), f'{x} → {y} ({lab})', a, b, fmt_p(p)])
    return md_table(['조건', '시나리오', '비교', '앞쪽만 통과', '뒤쪽만 통과', '부호 검정 p'], body, left=3)


def t_ladder(rows, m8rows):
    head = ['구성', '조건', '가속 0→V_H (/50)', '감속 V_H→V_L (/50)', '감속 ρ=1.5 (/10)', '가속 0→V_L (/50)']
    body = []
    def agg(rs, kind):
        rs = [r for r in ran(rs) if f'_{kind}_' in r['scenario_id'] or r['scenario_id'].startswith(f'main_ref_{kind}')]
        return rs
    conds = ['nominal', 'motor_tau_2', 'rotor_delay_5ms', 'state_delay_30ms']
    for v in VARS:
        for cnd in conds:
            if v == 'V13' and cnd == 'nominal':
                base = [r for r in m8rows if r['controller'] == 'V13' and r['group'] == 'reference'
                        and r['sensor_seed'] is not None and r['sensor_seed'] <= 1029]
            else:
                base = [r for r in rows if r['variant'] == v and f'_{cnd}__' in r['scenario_id']]
            if not base:
                continue
            def count(prefix, rho=None):
                rs = [r for r in ran(base) if prefix in r['scenario_id'] and (rho is None or r['scenario_id'].endswith(f'rho{rho}'))]
                return f'{npass(rs)}' if rs else '–'
            body.append([v, cnd, count('accel_VH'), count('brake_VH'), count('brake_VH', '1.5'), count('accel_VL')])
    f13 = [r for r in m8rows if r['controller'] == 'F13' and r['group'] == 'reference' and r['sensor_seed'] <= 1029]
    def cf(prefix, rho=None):
        rs = [r for r in ran(f13) if prefix in r['scenario_id'] and (rho is None or r['scenario_id'].endswith(f'rho{rho}'))]
        return f'{npass(rs)}'
    body.insert(0, ['F13 (참고)', 'nominal', cf('accel_VH'), cf('brake_VH'), cf('brake_VH', '1.5'), cf('accel_VL')])
    return md_table(head, body)


def t_weightcheck(rows, s8rows, abl):
    body = []
    def w(rs):
        return cell(rs)
    body.append(['F13 (튜닝값, PC-63)', 'w_v 10, w_ω 0.25, terminal 5', w(sel(s8rows, group='imu_latency_5ms', scenario_id='main_gust_lateral_+10_V_H', controller='F13')),
                 w(sel(s8rows, group='imu_latency_5ms', scenario_id='main_ref_accel_VH_rho1', controller='F13'))])
    for v, lab in (('F13-w1', 'F13 + w_ω 1.0'), ('F13-sV', 'F13 + w_v 5, w_ω 1, terminal 10')):
        body.append([v, lab, w(sel(rows, variant=v, group='imu_latency_5ms', scenario_id='main_gust_lateral_+10_V_H')),
                     w(sel(rows, variant=v, group='imu_latency_5ms', scenario_id='main_ref_accel_VH_rho1'))])
    body.append(['V13-0 (튜닝값 V13)', 'w_v 5, w_ω 1, terminal 10', w(sel(abl, variant='V13-0', group='imu_latency_5ms', scenario_id='main_gust_lateral_+10_V_H')),
                 w(sel(abl, variant='V13-0', group='imu_latency_5ms', scenario_id='main_ref_accel_VH_rho1'))])
    body.append(['V13-0-w025', 'V13-0 + w_ω 0.25', w(sel(rows, variant='V13-0-w025', group='imu_latency_5ms', scenario_id='main_gust_lateral_+10_V_H')),
                 w(sel(rows, variant='V13-0-w025', group='imu_latency_5ms', scenario_id='main_ref_accel_VH_rho1'))])
    return md_table(['구성', '상태 비용 가중치', 'IMU 5 ms 횡돌풍 V_H', 'IMU 5 ms 가속 0→V_H'], body, left=2)


def t_platform(abl, s8rows):
    body = []
    tot = up = down = 0
    for cnd in ('nominal', 'imu_latency_5ms'):
        for sid in ('main_gust_lateral_+10_V_H', 'main_ref_accel_VH_rho1', 'main_ref_brake_VH_rho1'):
            c = by_seed(sel(abl, group=cnd, scenario_id=sid, variant='V13'))
            w = by_seed(sel(s8rows, group=cnd, scenario_id=sid, controller='V13'))
            common = set(c) & set(w)
            a = sum(c[s] and not w[s] for s in common)
            b = sum(w[s] and not c[s] for s in common)
            tot += len(common); up += a; down += b
            body.append([cnd, sid.replace('main_', ''), f'{sum(c.values())}/{len(c)}', f'{sum(w.values())}/{len(w)}', a, b])
    body.append(['**합계**', '', '', '', f'**{up}**', f'**{down}**'])
    return md_table(['조건', '시나리오', 'V13 클라우드(Linux)', 'V13 PC-63(Windows)', '클라우드만 통과', 'PC-63만 통과'], body), tot, up, down


# ─────────────────────────────────────────────── main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--main8', action='append', required=True)
    ap.add_argument('--sensor8', required=True)
    ap.add_argument('--ablation8', required=True)
    ap.add_argument('--weightcheck8', nargs='+', required=True)
    a = ap.parse_args()
    raw = load_all(a)
    F = {k: [flat(k, r) for r in v] for k, v in raw.items()}
    for k, rows in F.items():
        write_csv(HERE/'data'/f'{k}_trials.csv', sorted(rows, key=lambda r: str(r['trial_id'])))
    # aggregate
    agg = defaultdict(list)
    for k, rows in F.items():
        for r in rows:
            agg[(k, r['group'], r['scenario_id'], r['controller'], r['variant'])].append(r)
    with open(HERE/'data'/'aggregate_pass_counts.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['experiment', 'group', 'scenario_id', 'controller', 'variant', 'rows', 'ran', 'tracking_pass',
                    'full_pass', 'optimizer_stop', 'diverged', 'median_window_max_omega'])
        for key in sorted(agg, key=lambda k: tuple(str(x) for x in k)):
            rs = ran(agg[key])
            mo = [r['window_max_omega'] for r in rs if r['window_max_omega'] is not None]
            w.writerow(list(key) + [len(agg[key]), len(rs), npass(rs), sum(r['full_pass'] for r in rs),
                                    sum(r['stop_reason'].startswith('5 consecutive optimizer') for r in rs),
                                    sum(r['stop_reason'].startswith('body-rate or altitude divergence') for r in rs),
                                    round(median(mo), 3) if mo else ''])
    plat, tot, up, down = t_platform(F['ablation8'], F['sensor8'])
    tables = dict(
        overview=t_overview(F), main8_groups=t_main8_groups(F['main8']), main8_rho=t_main8_rho(F['main8']),
        main8_numeric=t_main8_numeric(F['main8']), compute=t_compute(F), sensor8_core=t_sensor8_core(F['sensor8']),
        sensor8_mismatch=t_sensor8_mismatch(F['sensor8']), sensor8_tests=t_sensor8_tests(F['sensor8']),
        estimation=t_estimation(F['sensor8']), ablation=t_ablation(F['ablation8'], F['sensor8']),
        ablation_tests=t_ablation_tests(F['ablation8'], F['sensor8']),
        ladder=t_ladder(F['main8_ladder'], F['main8']),
        weightcheck=t_weightcheck(F['weightcheck8'], F['sensor8'], F['ablation8']), platform=plat,
        platform_total=f'{tot}회 중 {up + down}회 (클라우드만 통과 {up}, PC-63만 통과 {down})')
    (HERE/'tables').mkdir(exist_ok=True)
    for k, v in tables.items():
        (HERE/'tables'/f'{k}.md').write_text(v + '\n', encoding='utf-8')
    (HERE/'tables'/'main8_by_scenario.md').write_text(
        '# main8 시나리오별 추종 통과 수 (시드 20개, – = 설계 영역 밖이라 미시행)\n\n'
        + t_main8_by_scenario(F['main8']) + '\n', encoding='utf-8')
    tpl = (HERE/'README.template.md').read_text(encoding='utf-8')
    for k, v in tables.items():
        tpl = tpl.replace('{{' + k + '}}', v)
    if '{{' in tpl:
        raise SystemExit('unfilled marker in README.template.md: ' + tpl[tpl.index('{{'):tpl.index('{{') + 40])
    (HERE/'README.md').write_text(tpl, encoding='utf-8')
    print({k: len(v) for k, v in F.items()})


if __name__ == '__main__':
    main()
