"""
센서 조건 스윕 + RTK EKF 결론 검증 — 연구계획서 과제 3
=======================================================

두 가지 모드:

  [grid] 센서 조건 격자 × 제어기 비교           ← 과제 3 본체
         잡음 0.02~3.0m × 주기 1~20Hz × 지연 0~0.2s × 결측 0~20%
         "제어기 순위가 뒤집히는 구간"을 찾는 게 목적.

  [mc]   고정 센서 조건에서 몬테카를로            ← 기존 결론 검증
         results/MISSION_ANALYSIS.md의 "Hybrid+RTK EKF = 1.670m,
         참값 대비 -4.7% 개선"은 단일런 결과다. 재현 환경에 따라
         +408%까지 갈리는 것이 확인되었으므로 MC로 재판정한다.

실행:
    python3 sensor_sweep.py mc   --ctrl hybrid --trials 10
    python3 sensor_sweep.py grid --quick
    python3 sensor_sweep.py grid                 # 전체 격자

출력:
    results/sweep_<모드>_<태그>.csv   기계가 읽는 원본
    표준출력                          사람이 읽는 표

주의:
  - 제어기 인스턴스는 시행마다 reset(). VirtualNMPC의 warm start 오염이
    시행 간 독립성을 깨뜨린 전례가 있음 (커밋 1e6d085 참고).
  - MC에서 센서 시드도 시행마다 바꾼다. 초기조건·돌풍만 랜덤화하고
    센서 노이즈 실현을 고정하면 '센서가 만드는 분산'을 못 본다.
  - 격자 모드는 조건 간 공정 비교를 위해 초기조건·돌풍을 고정한다.
"""

import argparse
import csv
import os
import time as timer

import numpy as np

from dynamics import AxialDronePlant
from vehicle_params import vehicle_params as P
from controller import ScheduledLQR
from hybrid_comparison import VirtualNMPC, ProperHybrid
from fallback_controller import HybridWithFallback
from sensors import create_default_sensors
from mission_sim import (MissionProfile, MissionController, make_gust_fn,
                         run_mission, run_mission_ekf,
                         compute_overall, compute_phase_metrics)

RESULTS_DIR = 'results'

# 실기 판정 기준 (results/acados_port_status.md에서 확정된 기준)
#   |ω| > 35 rad/s  또는  |ω| > 25가 200ms 이상 지속  → 실기 기준 실패
OMEGA_HARD = 35.0
OMEGA_SOFT = 25.0
OMEGA_SOFT_MS = 200.0


# ══════════════════════════════════════════════════════
# 지표
# ══════════════════════════════════════════════════════

def omega_metrics(xs, dt=0.001):
    """각속도 기반 실기 판정. Returns (|ω|max, fail: bool)."""
    w = xs[:, 10:13]
    if not np.all(np.isfinite(w)):
        return np.inf, True
    mag = np.linalg.norm(w, axis=1)
    w_max = float(np.max(mag))
    if w_max > OMEGA_HARD:
        return w_max, True
    # 소프트 기준: >25가 연속 200ms 이상
    over = mag > OMEGA_SOFT
    if over.any():
        # 연속 구간 최장 길이
        idx = np.flatnonzero(np.diff(np.concatenate(([0], over.view(np.int8), [0]))))
        runs = (idx[1::2] - idx[0::2]) * dt * 1000.0
        if runs.size and runs.max() >= OMEGA_SOFT_MS:
            return w_max, True
    return w_max, False


def summarize(res, profile, dt=0.001):
    """한 번의 시뮬 결과 → 지표 dict."""
    ov = compute_overall(res)
    out = {
        'diverged': bool(ov.get('diverged', False)),
        'rmse_z': ov['rmse_z'],
        'rmse_vx': ov['rmse_vx'],
    }
    w_max, w_fail = omega_metrics(res['xs'], dt)
    out['omega_max'] = w_max
    out['omega_fail'] = w_fail

    # 구간별 (순항·감속만 뽑음 — 나머지는 전 환경에서 안정적으로 재현됨)
    for pm in compute_phase_metrics(res, profile):
        if pm['name'] in ('순항', '감속'):
            key = 'cruise' if pm['name'] == '순항' else 'decel'
            out[f'{key}_rmse_z'] = pm['rmse_z']
            out[f'{key}_max_dz'] = pm['max_z_err']

    # 추정 오차 (EKF 런에만 존재)
    if 'xs_est' in res:
        est = res['xs_est']
        tru = res['xs']
        if np.all(np.isfinite(est)) and np.all(np.isfinite(tru)):
            out['est_pos_rmse'] = float(np.sqrt(np.mean(
                np.sum((est[:, 0:3] - tru[:, 0:3]) ** 2, axis=1))))
            out['est_vel_rmse'] = float(np.sqrt(np.mean(
                np.sum((est[:, 3:6] - tru[:, 3:6]) ** 2, axis=1))))
        else:
            out['est_pos_rmse'] = np.inf
            out['est_vel_rmse'] = np.inf
    return out


# ══════════════════════════════════════════════════════
# 제어기 생성
# ══════════════════════════════════════════════════════

def make_controller(kind, profile, dt):
    """
    제어기 + MissionController 래퍼 생성.

    lqr       : ScheduledLQR — 폴백 전용으로 쓰이던 것, 여기선 단독 비교군
    hybrid    : ProperHybrid 단독 — mission_sim.py가 쓰는 그 구성.
                안전 계층이 없다. 팀 문서의 "주 제어기" 성능 숫자가 이것.
    hybrid_fb : ProperHybrid + ScheduledLQR 인수 (ω15 백스톱, 복귀 금지)
                — results/acados_port_status.md에서 최종 확정된 아키텍처.
                mission_sim.py의 베이스라인은 이걸 반영하지 않고 있다.
    """
    if kind == 'lqr':
        inner = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=2.0)
    elif kind == 'hybrid':
        vnmpc = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                            N=20, dt_nmpc=0.05, dt_ctrl=0.02)
        inner = ProperHybrid(vnmpc, P, dt=dt)
    elif kind == 'hybrid_fb':
        vnmpc = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                            N=20, dt_nmpc=0.05, dt_ctrl=0.02)
        hyb = ProperHybrid(vnmpc, P, dt=dt)
        lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=2.0)
        # 파라미터는 팀 실험 스크립트(acados_fallback_mc.py)와 동일하게 맞춤:
        #   z_err_limit=10.0  — 기본값 2.0은 순항 유지 기준이라 이륙 상승
        #                       추종 지연만으로 t≈1.8s에 오발동한다(실측).
        #   cooldown_sec=1e6  — 사실상 복귀 금지. acados 솔버 재개가
        #                       QP 승수 오염으로 불안정 → 복귀 후 재텀블.
        #   omega_limit=15.0  — LQR이 아직 인수 가능한 영역에서 넘김.
        inner = HybridWithFallback(hyb, lqr, z_ref=2.0, dt=dt,
                                   z_err_limit=10.0,
                                   omega_limit=15.0, cooldown_sec=1e6)
    else:
        raise ValueError(
            f"알 수 없는 제어기: {kind} (lqr | hybrid | hybrid_fb)")
    return MissionController(inner, profile)


def fallback_info(ctrl):
    """폴백 전환 횟수 (없는 제어기면 None)."""
    return getattr(ctrl.inner, 'fallback_count', None)


# ══════════════════════════════════════════════════════
# 모드 1: MC — 기존 RTK 결론 재판정
# ══════════════════════════════════════════════════════

def run_mc(args):
    dt = 0.001
    plant = AxialDronePlant(P, dt=dt)
    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    cs, ce = profile.cruise_start, profile.cruise_end

    rng = np.random.default_rng(args.seed)
    ctrl = make_controller(args.ctrl, profile, dt)

    label = f"{args.ctrl}_{'ekf' if args.ekf else 'true'}"
    print(f"\n{'='*72}")
    print(f"  MC 재판정 — 제어기 {args.ctrl} / "
          f"{'RTK EKF (σ=%.3gm)' % args.noise_pos if args.ekf else '참값'}"
          f" / {args.trials}회")
    if args.ekf:
        print(f"  센서: {args.gps_rate:.0f}Hz, 지연 {args.delay*1000:.0f}ms, "
              f"결측 {args.dropout*100:.0f}%")
    print(f"{'='*72}\n")

    rows = []
    for i in range(args.trials):
        x0 = AxialDronePlant.hover_state(P)
        x0[2] = 2.0 + rng.normal(0, 0.5)
        x0[3] = rng.normal(0, 0.5)
        x0[5] = rng.normal(0, 0.3)
        W = float(rng.uniform(5, 15))
        t_g = float(rng.uniform(cs + 2, ce - 3))
        gust = make_gust_fn('vertical', W, t_g, 1.0)

        ctrl.reset()
        t0 = timer.time()
        if args.ekf:
            # 센서 시드를 시행마다 바꿔 노이즈 실현도 랜덤화
            sensors = create_default_sensors(
                dt_plant=dt, seed=args.seed + 1000 + i,
                gps_rate=args.gps_rate,
                noise_pos=args.noise_pos, noise_vel=args.noise_vel,
                delay=args.delay, dropout=args.dropout)
            res = run_mission_ekf(plant, ctrl, x0, profile, sensors,
                                  wind_fn=gust, seed=args.seed + 1000 + i)
        else:
            res = run_mission(plant, ctrl, x0, profile, wind_fn=gust)
        el = timer.time() - t0

        m = summarize(res, profile, dt)
        fb = fallback_info(ctrl)
        m.update(trial=i, W_max=W, t_gust=t_g, elapsed=el,
                 fallback_count=(-1 if fb is None else fb))
        rows.append(m)

        flag = 'DIV' if m['diverged'] else ('ωFAIL' if m['omega_fail'] else 'ok')
        fbs = '' if fb is None else f"  전환 {fb}"
        print(f"  [{i+1:2d}/{args.trials}] z={m['rmse_z']:8.3f}  "
              f"감속={m.get('decel_rmse_z', float('nan')):8.3f}  "
              f"max_dz={m.get('decel_max_dz', float('nan')):7.2f}  "
              f"|ω|={m['omega_max']:6.2f}  {flag:6s}{fbs} [{el:.0f}s]",
              flush=True)

    _print_mc_summary(rows, label)
    _write_csv(rows, f'sweep_mc_{label}.csv')
    return rows


def _print_mc_summary(rows, label):
    ok = [r for r in rows if not r['diverged']]
    z = np.array([r['rmse_z'] for r in ok]) if ok else np.array([])
    dz = np.array([r.get('decel_max_dz', np.nan) for r in ok]) if ok else np.array([])
    n_wfail = sum(r['omega_fail'] for r in rows)

    print(f"\n{'-'*72}")
    print(f"  요약 [{label}]  완주 {len(ok)}/{len(rows)}, "
          f"실기판정 FAIL {n_wfail}/{len(rows)}")
    n_phase_div = int(np.sum(~np.isfinite(dz))) if len(dz) else 0
    if len(z):
        print(f"  RMSE z    평균 {z.mean():.3f}  std {z.std():.3f}  "
              f"최소 {z.min():.3f}  최대 {z.max():.3f}")
        fin = dz[np.isfinite(dz)]
        if len(fin):
            print(f"  감속 max_dz  중앙 {np.median(fin):.2f}m  "
                  f"P90 {np.percentile(fin, 90):.2f}m  최악 {np.max(fin):.2f}m"
                  + (f"  (+ 구간발산 {n_phase_div}회 제외)" if n_phase_div else ""))
        print(f"  5m+ 고도이탈 발생률 {100*np.mean(~(dz <= 5.0)):.0f}%"
              f"  (구간발산 포함)")
        # 폴백 전환 통계
        fbs = [r.get('fallback_count', -1) for r in rows]
        if any(f >= 0 for f in fbs):
            n_sw = sum(1 for f in fbs if f > 0)
            print(f"  폴백 전환  {n_sw}/{len(rows)}회 시행에서 발생 "
                  f"(평균 {np.mean([f for f in fbs if f >= 0]):.1f}회/시행)")
    print(f"{'-'*72}\n")


# ══════════════════════════════════════════════════════
# 모드 2: grid — 과제 3 센서 조건 격자
# ══════════════════════════════════════════════════════

# (라벨, noise_pos, gps_rate, delay, dropout)
GRID_QUICK = [
    ('RTK 기준',        0.02, 10.0, 0.00, 0.00),
    ('지연 100ms',      0.02, 10.0, 0.10, 0.00),
    ('결측 20%',        0.02, 10.0, 0.00, 0.20),
    ('일반GPS 1.5m',    1.50, 10.0, 0.00, 0.00),
]

GRID_FULL = [
    # 기준
    ('RTK 기준',        0.02, 10.0, 0.00, 0.00),
    # 위치 잡음 스윕 (계획서: 0.02~3.0 m)
    ('잡음 0.2m',       0.20, 10.0, 0.00, 0.00),
    ('잡음 1.5m',       1.50, 10.0, 0.00, 0.00),
    ('잡음 3.0m',       3.00, 10.0, 0.00, 0.00),
    # 갱신주기 스윕 (계획서: 1~20 Hz)
    ('20Hz',            0.02, 20.0, 0.00, 0.00),
    ('5Hz',             0.02,  5.0, 0.00, 0.00),
    ('1Hz',             0.02,  1.0, 0.00, 0.00),
    # 지연 스윕 (계획서: 0~0.2 s)
    ('지연 50ms',       0.02, 10.0, 0.05, 0.00),
    ('지연 100ms',      0.02, 10.0, 0.10, 0.00),
    ('지연 200ms',      0.02, 10.0, 0.20, 0.00),
    # 결측 스윕 (계획서: 0~20%)
    ('결측 5%',         0.02, 10.0, 0.00, 0.05),
    ('결측 10%',        0.02, 10.0, 0.00, 0.10),
    ('결측 20%',        0.02, 10.0, 0.00, 0.20),
    # 복합 (실제 열화 환경)
    ('1.5m+5Hz+100ms',  1.50,  5.0, 0.10, 0.00),
    ('1.5m+5Hz+20%',    1.50,  5.0, 0.00, 0.20),
    ('최악 복합',        3.00,  1.0, 0.20, 0.20),
]


def run_grid(args):
    dt = 0.001
    plant = AxialDronePlant(P, dt=dt)
    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)

    grid = GRID_QUICK if args.quick else GRID_FULL
    kinds = args.ctrls.split(',')

    # 조건 간 공정 비교 — 초기조건·돌풍 고정
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 2.0
    gust = make_gust_fn('vertical', 10.0, 35.0, 1.0)

    print(f"\n{'='*100}")
    print(f"  과제 3 — 센서 조건 격자  ({len(grid)}조건 × {len(kinds)}제어기 "
          f"= {len(grid)*len(kinds)}런)")
    print(f"  초기조건·돌풍 고정(수직 10m/s @ t=35s), 센서 시드 고정 — 조건 효과만 분리")
    print(f"{'='*100}\n")
    print(f"  {'제어기':>7s} {'조건':>16s} {'전체 z':>9s} {'순항 z':>9s} "
          f"{'감속 z':>9s} {'감속 dz':>9s} {'추정 pos':>9s} {'|ω|max':>8s} "
          f"{'결측률':>7s} {'판정':>6s}")
    print("  " + "-"*98)

    rows = []
    for kind in kinds:
        ctrl = make_controller(kind, profile, dt)
        # 참값 기준선 (센서 없음) — 열화율 계산용
        ctrl.reset()
        base = summarize(run_mission(plant, ctrl, x0.copy(), profile,
                                     wind_fn=gust), profile, dt)
        print(f"  {kind:>7s} {'참값(센서없음)':>16s} {base['rmse_z']:9.3f} "
              f"{base.get('cruise_rmse_z', np.nan):9.3f} "
              f"{base.get('decel_rmse_z', np.nan):9.3f} "
              f"{base.get('decel_max_dz', np.nan):9.2f} "
              f"{'-':>9s} {base['omega_max']:8.2f} {'-':>7s} "
              f"{'FAIL' if base['omega_fail'] else 'ok':>6s}", flush=True)
        rows.append({**base, 'ctrl': kind, 'cond': '참값', 'noise_pos': 0.0,
                     'gps_rate': 0.0, 'delay': 0.0, 'dropout': 0.0,
                     'dropout_meas': 0.0, 'degrade_pct': 0.0})

        for (label, npos, rate, delay, drop) in grid:
            sensors = create_default_sensors(
                dt_plant=dt, seed=args.seed, gps_rate=rate,
                noise_pos=npos, noise_vel=max(0.05, npos / 3.0),
                delay=delay, dropout=drop)
            ctrl.reset()
            t0 = timer.time()
            res = run_mission_ekf(plant, ctrl, x0.copy(), profile, sensors,
                                  wind_fn=gust, seed=args.seed)
            el = timer.time() - t0
            m = summarize(res, profile, dt)
            deg = (100.0 * (m['rmse_z'] - base['rmse_z']) / base['rmse_z']
                   if np.isfinite(m['rmse_z']) and base['rmse_z'] > 0 else np.inf)
            m.update(ctrl=kind, cond=label, noise_pos=npos, gps_rate=rate,
                     delay=delay, dropout=drop,
                     dropout_meas=sensors.gps.dropout_rate(),
                     degrade_pct=deg, elapsed=el)
            rows.append(m)

            flag = 'DIV' if m['diverged'] else ('ωFAIL' if m['omega_fail'] else 'ok')
            print(f"  {kind:>7s} {label:>16s} {m['rmse_z']:9.3f} "
                  f"{m.get('cruise_rmse_z', np.nan):9.3f} "
                  f"{m.get('decel_rmse_z', np.nan):9.3f} "
                  f"{m.get('decel_max_dz', np.nan):9.2f} "
                  f"{m.get('est_pos_rmse', np.nan):9.3f} "
                  f"{m['omega_max']:8.2f} "
                  f"{sensors.gps.dropout_rate():7.3f} {flag:>6s}", flush=True)

    _print_rank_flip(rows, kinds)
    _write_csv(rows, f"sweep_grid_{'quick' if args.quick else 'full'}.csv")
    return rows


def _print_rank_flip(rows, kinds):
    """조건별 제어기 순위 — 계획서가 찾겠다고 한 '순위 역전 구간'."""
    if len(kinds) < 2:
        return
    conds = []
    for r in rows:
        if r['cond'] not in conds:
            conds.append(r['cond'])

    print(f"\n{'-'*72}")
    print("  조건별 제어기 순위 (RMSE z 기준, 낮을수록 좋음)")
    print(f"{'-'*72}")
    print(f"  {'조건':>16s} " + " ".join(f"{k:>10s}" for k in kinds) + "   승자")
    prev_winner = None
    for c in conds:
        vals = {}
        for k in kinds:
            hit = [r for r in rows if r['cond'] == c and r['ctrl'] == k]
            vals[k] = hit[0]['rmse_z'] if hit else np.nan
        winner = min(vals, key=lambda k: (np.inf if not np.isfinite(vals[k])
                                          else vals[k]))
        flip = ' ← 역전!' if prev_winner and winner != prev_winner else ''
        print(f"  {c:>16s} " + " ".join(f"{vals[k]:10.3f}" for k in kinds)
              + f"   {winner}{flip}")
        prev_winner = winner
    print(f"{'-'*72}\n")


# ══════════════════════════════════════════════════════

def _write_csv(rows, name):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    path = os.path.join(RESULTS_DIR, name)
    keys = sorted({k for r in rows for k in r})
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"  [저장] {path}")


def main():
    ap = argparse.ArgumentParser(description='센서 조건 스윕 / RTK 결론 재판정')
    sub = ap.add_subparsers(dest='mode', required=True)

    m = sub.add_parser('mc', help='고정 센서 조건에서 몬테카를로')
    m.add_argument('--ctrl', default='hybrid',
                   choices=['lqr', 'hybrid', 'hybrid_fb'])
    m.add_argument('--trials', type=int, default=10)
    m.add_argument('--seed', type=int, default=0)
    m.add_argument('--ekf', action='store_true', default=True)
    m.add_argument('--true-state', dest='ekf', action='store_false',
                   help='센서 없이 참값으로 (기존 MC 재현용)')
    m.add_argument('--noise-pos', dest='noise_pos', type=float, default=0.02)
    m.add_argument('--noise-vel', dest='noise_vel', type=float, default=0.05)
    m.add_argument('--gps-rate', dest='gps_rate', type=float, default=10.0)
    m.add_argument('--delay', type=float, default=0.0)
    m.add_argument('--dropout', type=float, default=0.0)

    g = sub.add_parser('grid', help='센서 조건 격자')
    g.add_argument('--ctrls', default='lqr,hybrid',
                   help='쉼표 구분 (lqr,hybrid,hybrid_fb)')
    g.add_argument('--quick', action='store_true', help='4조건 축약 격자')
    g.add_argument('--seed', type=int, default=42)

    args = ap.parse_args()
    t0 = timer.time()
    if args.mode == 'mc':
        run_mc(args)
    else:
        run_grid(args)
    print(f"  총 소요 {(timer.time()-t0)/60:.1f}분\n")


if __name__ == '__main__':
    main()
