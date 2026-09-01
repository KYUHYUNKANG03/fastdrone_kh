"""
각속도 시계열 그림 — 안전 계층 유무 비교 (발표 핵심 그림)
=========================================================

무엇을 보여주는가
-----------------
같은 65초 미션을 두 구성으로 날린다.

  (A) 분리형 단독      ProperHybrid            — mission_sim.py의 현 베이스라인
  (B) 분리형 + 안전계층 HybridWithFallback      — 8월에 확정된 아키텍처

세 패널을 같은 시간축에 쌓는다:

  ① 고도 오차 Δz   — 두 구성이 비슷해 보인다 (RMSE가 보는 것)
  ② 각속도 |ω|     — 한쪽만 자이로 포화 한계를 넘는다 (RMSE가 못 보는 것)
  ③ 모터 평균속도  — 감속 중 바닥에 닿는 순간이 ②의 원인

즉 "RMSE만 보면 같아 보이지만 실기 기준으로는 하나가 실패한다"를
한 장으로 보여주는 것이 목적이다.

실행
----
    python3 omega_plot.py                # 참값 (센서 없음, 약 4분)
    python3 omega_plot.py --ekf          # RTK EKF 조건 (약 6분)
    python3 omega_plot.py --no-motor     # 2단 구성 (슬라이드용, 더 크게)

출력
----
    results/omega_timeseries.png       (--ekf 면 _ekf 접미사)
    콘솔에 발표에서 인용할 숫자 요약

주의
----
  - 축 라벨은 영문. 한글 폰트가 없는 환경에서 네모가 뜨는 것을 피하기 위함.
    슬라이드 캡션은 PPT에서 한글로 달면 된다.
  - 색은 CVD(색각이상) 검증을 통과한 조합 (ΔE 22.7, deutan 기준).
"""

import argparse
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from dynamics import AxialDronePlant
from vehicle_params import vehicle_params as P
from controller import ScheduledLQR
from hybrid_comparison import VirtualNMPC, ProperHybrid
from fallback_controller import HybridWithFallback
from sensors import create_default_sensors
from mission_sim import (MissionProfile, MissionController, make_gust_fn,
                         run_mission, run_mission_ekf)

# ── 실기 판정 기준 (results/acados_port_status.md) ──
OMEGA_HARD = 35.0    # 자이로 측정 한계 (±2000 deg/s ≈ 34.9 rad/s)
OMEGA_SOFT = 25.0
OMEGA_TRIG = 15.0    # 폴백 전환 임계

# ── 색 (validate_palette.js 전 항목 통과) ──
C_BARE = '#C2410C'   # 분리형 단독
C_SAFE = '#1A7FB5'   # 분리형 + 안전계층
C_INK = '#1F2937'
C_MUTED = '#6B7280'
C_GRID = '#D8DEE3'
C_SHADE = '#F1E9DC'


class SwitchLogger(HybridWithFallback):
    """폴백 전환 시각을 기록."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.switch_times = []

    def __call__(self, t, x):
        was = self._using_hybrid
        u = super().__call__(t, x)
        if was and not self._using_hybrid:
            self.switch_times.append(float(t))
        return u


def build(kind, profile, dt):
    vn = VirtualNMPC(P, v_ref=[0, 0, 0], z_ref=2.0,
                     N=20, dt_nmpc=0.05, dt_ctrl=0.02)
    hyb = ProperHybrid(vn, P, dt=dt)
    if kind == 'bare':
        return MissionController(hyb, profile), None
    lqr = ScheduledLQR(P, v_ref=[0, 0, 0], z_ref=2.0)
    inner = SwitchLogger(hyb, lqr, z_ref=2.0, dt=dt,
                         z_err_limit=10.0, omega_limit=OMEGA_TRIG,
                         cooldown_sec=1e6)
    return MissionController(inner, profile), inner


def omega_stats(ts, xs, dt=0.001):
    w = np.linalg.norm(xs[:, 10:13], axis=1)
    finite = np.isfinite(w)
    w_max = float(np.nanmax(w[finite])) if finite.any() else np.inf
    fail = (not finite.all()) or w_max > OMEGA_HARD
    if not fail:
        over = w > OMEGA_SOFT
        if over.any():
            idx = np.flatnonzero(np.diff(np.concatenate(
                ([0], over.view(np.int8), [0]))))
            runs = (idx[1::2] - idx[0::2]) * dt * 1000.0
            fail = bool(runs.size and runs.max() >= 200.0)
    t_cross = None
    hit = np.flatnonzero(w > OMEGA_HARD)
    if hit.size:
        t_cross = float(ts[hit[0]])
    return w, w_max, fail, t_cross


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ekf', action='store_true',
                    help='RTK EKF 조건 (기본은 참값)')
    ap.add_argument('--no-motor', dest='motor', action='store_false',
                    help='모터 패널 생략 (2단, 슬라이드용)')
    ap.add_argument('--zoom', action='store_true',
                    help='감속 구간 확대 (t=40~65, 발표 슬라이드용)')
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    dt = 0.001
    plant = AxialDronePlant(P, dt=dt)
    profile = MissionProfile(cruise_speed=70.0, cruise_alt=50.0)
    x0 = AxialDronePlant.hover_state(P)
    x0[2] = 2.0
    gust = make_gust_fn('vertical', 10.0, 35.0, 1.0)

    runs = {}
    for kind, label in [('bare', 'Interface split only'),
                        ('safe', 'Interface split + safety layer')]:
        ctrl, logger = build(kind, profile, dt)
        ctrl.reset()
        print(f"  {label} 시뮬 중...", flush=True)
        if args.ekf:
            sensors = create_default_sensors(
                dt_plant=dt, seed=args.seed,
                noise_pos=0.02, noise_vel=0.05)
            res = run_mission_ekf(plant, ctrl, x0.copy(), profile,
                                  sensors, wind_fn=gust, seed=args.seed)
        else:
            res = run_mission(plant, ctrl, x0.copy(), profile, wind_fn=gust)
        w, w_max, fail, t_cross = omega_stats(res['ts'], res['xs'], dt)
        runs[kind] = dict(label=label, res=res, w=w, w_max=w_max,
                          fail=fail, t_cross=t_cross,
                          switches=(logger.switch_times if logger else []))
        print(f"      |ω|max {w_max:6.2f} rad/s   실기판정 "
              f"{'FAIL' if fail else 'PASS'}"
              + (f"   전환 t={logger.switch_times[0]:.1f}s"
                 if logger and logger.switch_times else ""))

    # ── 그림 ──
    n = 3 if args.motor else 2
    fig, axes = plt.subplots(n, 1, figsize=(11, 3.0 * n + 0.6), sharex=True,
                             gridspec_kw={'hspace': 0.18})
    bounds = profile.get_phase_boundaries()
    decel = [b for b in bounds if b[0] == '감속'][0]
    xlo, xhi = (40.0, profile.T_total) if args.zoom else (0.0, profile.T_total)
    axes[0].set_xlim(xlo, xhi)          # sharex — 주석 배치 전에 확정

    for ax in axes:
        ax.axvspan(decel[1], decel[2], color=C_SHADE, zorder=0)
        for _, t0, _ in bounds:
            ax.axvline(t0, color=C_GRID, lw=0.8, zorder=1)
        ax.grid(True, color=C_GRID, lw=0.6, alpha=0.7, zorder=1)
        ax.set_axisbelow(True)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        for s in ('left', 'bottom'):
            ax.spines[s].set_color(C_GRID)
        ax.tick_params(colors=C_MUTED, labelsize=9)

    # bare(주황)를 위에 — t<전환시각에는 두 곡선이 완전히 동일하므로
    # 아래 선이 가려진다. 그 구간이 '같은 비행'임을 캡션으로 설명할 것.
    style = {'bare': dict(color=C_BARE, lw=1.8, zorder=4),
             'safe': dict(color=C_SAFE, lw=2.2, zorder=3)}

    # ① 고도 오차
    for k, r in runs.items():
        dz = r['res']['xs'][:, 2] - r['res']['z_refs']
        axes[0].plot(r['res']['ts'], dz, label=r['label'], **style[k])
    axes[0].axhline(0, color=C_MUTED, lw=0.8, ls=(0, (4, 3)))
    axes[0].set_ylabel('Altitude error  Δz  [m]', fontsize=10, color=C_INK)
    axes[0].set_title('Both look comparable on the altitude metric',
                      fontsize=11, color=C_INK, loc='left', pad=8)

    # ② 각속도 — 주인공
    for k, r in runs.items():
        axes[1].plot(r['res']['ts'], r['w'], **style[k])
    axes[1].axhline(OMEGA_HARD, color=C_MUTED, lw=1.1, ls=(0, (5, 3)))
    axes[1].axhline(OMEGA_TRIG, color=C_MUTED, lw=1.0, ls=(0, (2, 3)))
    ymax = max(r['w_max'] for r in runs.values())
    axes[1].set_ylim(-2, max(ymax * 1.18, 45))
    ytr = axes[1].get_yaxis_transform()   # x=축비율, y=데이터
    axes[1].text(0.012, OMEGA_HARD + ymax * 0.025, 'Gyro saturation  35 rad/s',
                 fontsize=9, color=C_MUTED, transform=ytr)
    axes[1].text(0.012, OMEGA_TRIG + ymax * 0.025, 'Fallback trigger  15 rad/s',
                 fontsize=9, color=C_MUTED, transform=ytr)
    axes[1].set_ylabel('Body rate  |ω|  [rad/s]', fontsize=10, color=C_INK)
    axes[1].set_title('But only one stays inside the gyro measurement range',
                      fontsize=11, color=C_INK, loc='left', pad=8)

    bare = runs['bare']
    if bare['t_cross'] is not None:
        span = xhi - xlo
        axes[1].annotate(
            f"tumbling  |ω| peak {bare['w_max']:.0f} rad/s",
            xy=(bare['t_cross'], OMEGA_HARD),
            xytext=(max(xlo + span * 0.06, bare['t_cross'] - span * 0.28),
                    ymax * 0.82),
            fontsize=10, color=C_BARE, fontweight='medium',
            arrowprops=dict(arrowstyle='->', color=C_BARE, lw=1.3))
    span = xhi - xlo
    for t_sw in runs['safe']['switches'][:1]:
        axes[1].annotate(
            f"handover to LQR  t={t_sw:.1f}s",
            xy=(t_sw, OMEGA_TRIG),
            xytext=(xhi - span * 0.30, ymax * 0.60),
            ha='left',
            fontsize=10, color=C_SAFE, fontweight='medium',
            arrowprops=dict(arrowstyle='->', color=C_SAFE, lw=1.3))

    # ③ 모터
    if args.motor:
        for k, r in runs.items():
            us = r['res']['us']
            axes[2].plot(r['res']['ts'][:len(us)], us.mean(axis=1), **style[k])
        axes[2].axhline(0, color=C_MUTED, lw=0.8, ls=(0, (4, 3)))
        axes[2].set_ylabel('Mean rotor speed  [rad/s]', fontsize=10, color=C_INK)
        axes[2].set_title('Cause: rotors approach zero during deceleration, '
                          'so INDI loses control authority',
                          fontsize=11, color=C_INK, loc='left', pad=8)

    axes[-1].set_xlabel('Time  [s]', fontsize=10, color=C_INK)

    # 구간 라벨
    for name, t0, t1 in bounds:
        en = {'이륙': 'takeoff', '안정화': '', '가속': 'accel',
              '순항': 'cruise', '감속': 'DECELERATION', '호버링': 'hover'}[name]
        if en and t1 > xlo and t0 < xhi:
            axes[0].text(np.clip((t0 + t1) / 2, xlo + 2, xhi - 2),
                         axes[0].get_ylim()[1] * 0.90, en,
                         ha='center', fontsize=8.5, color=C_MUTED)

    cond = 'RTK GPS + ESKF' if args.ekf else 'true state (no sensors)'
    fig.suptitle(f'Angular rate reveals what altitude RMSE hides  —  {cond}',
                 fontsize=12.5, color=C_INK, x=0.125, ha='left', y=1.00)
    handles = [plt.Line2D([], [], **style[k], label=runs[k]['label'])
               for k in ('bare', 'safe')]
    fig.legend(handles=handles, frameon=False, fontsize=10, ncol=2,
               loc='upper left', bbox_to_anchor=(0.122, 0.975),
               labelcolor=C_INK, handlelength=2.4, columnspacing=2.4)

    os.makedirs('results', exist_ok=True)
    path = (f"results/omega_timeseries{'_ekf' if args.ekf else ''}"
            f"{'_zoom' if args.zoom else ''}.png")
    fig.savefig(path, dpi=170, bbox_inches='tight', facecolor='white')
    print(f"\n  [저장] {path}")

    # ── 발표용 숫자 요약 ──
    print(f"\n{'='*66}")
    print(f"  발표에서 인용할 숫자  ({cond})")
    print(f"{'='*66}")
    hdr = f"  {'구성':<24s} {'RMSE z':>9s} {'|ω|max':>9s} {'실기판정':>9s}"
    print(hdr)
    print("  " + "-" * 62)
    for k, r in runs.items():
        xs, zr = r['res']['xs'], r['res']['z_refs']
        rmse = float(np.sqrt(np.mean((xs[:, 2] - zr) ** 2)))
        name = '분리형 단독' if k == 'bare' else '분리형 + 안전계층'
        print(f"  {name:<24s} {rmse:9.3f} {r['w_max']:9.2f} "
              f"{'FAIL' if r['fail'] else 'PASS':>9s}")
    print("  " + "-" * 62)
    print("  → 고도 RMSE 차이는 작지만, 각속도 기준으로는 판정이 갈린다.")
    print(f"{'='*66}\n")


if __name__ == '__main__':
    main()
