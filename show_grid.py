"""sweep_grid_*.csv → 사람이 읽는 표 + 조건별 제어기 순위."""
import csv, sys, math

path = sys.argv[1] if len(sys.argv) > 1 else 'results/sweep_grid_full.csv'
rows = list(csv.DictReader(open(path)))
if not rows:
    sys.exit(f"{path} 가 비어 있습니다")

def f(r, k):
    v = r.get(k, '')
    try:
        x = float(v)
        return x if math.isfinite(x) else float('inf')
    except (TypeError, ValueError):
        return float('nan')

def s(x, w=9, p=3):
    if math.isnan(x): return ' ' * (w - 1) + '-'
    if math.isinf(x): return ' ' * (w - 3) + 'DIV'
    return f"{x:{w}.{p}f}"

ctrls, conds = [], []
for r in rows:
    if r['ctrl'] not in ctrls: ctrls.append(r['ctrl'])
    if r['cond'] not in conds: conds.append(r['cond'])

print(f"\n{'='*104}")
print(f"  센서 조건 격자 — {len(conds)}조건 × {len(ctrls)}제어기 = {len(rows)}런   [{path}]")
print(f"{'='*104}")
print(f"  {'제어기':>9s} {'조건':>16s} {'전체 z':>9s} {'순항 z':>9s} {'감속 z':>9s} "
      f"{'감속 dz':>9s} {'추정 pos':>9s} {'|w|max':>8s} {'판정':>6s}")
print("  " + "-" * 100)
for c in ctrls:
    for r in [x for x in rows if x['ctrl'] == c]:
        fail = str(r.get('omega_fail', '')).lower().startswith('t')
        div = str(r.get('diverged', '')).lower().startswith('t')
        flag = 'DIV' if div else ('wFAIL' if fail else 'ok')
        print(f"  {c:>9s} {r['cond']:>16s} {s(f(r,'rmse_z'))} {s(f(r,'cruise_rmse_z'))} "
              f"{s(f(r,'decel_rmse_z'))} {s(f(r,'decel_max_dz'),9,2)} "
              f"{s(f(r,'est_pos_rmse'))} {s(f(r,'omega_max'),8,2)} {flag:>6s}")
    print()

# ── 순위 ──
print(f"{'-'*104}")
print("  조건별 제어기 순위 (전체 RMSE z 기준, 낮을수록 좋음)")
print(f"{'-'*104}")
print(f"  {'조건':>16s} " + " ".join(f"{c:>11s}" for c in ctrls) + "     승자")
prev = None
for cd in conds:
    vals = {}
    for c in ctrls:
        hit = [x for x in rows if x['cond'] == cd and x['ctrl'] == c]
        vals[c] = f(hit[0], 'rmse_z') if hit else float('nan')
    ok = {k: v for k, v in vals.items() if not math.isnan(v)}
    win = min(ok, key=ok.get) if ok else '-'
    flip = '   <-- 역전!' if prev and win != prev else ''
    print(f"  {cd:>16s} " + " ".join(s(vals[c], 11) for c in ctrls) + f"   {win}{flip}")
    prev = win

# ── 실기판정 요약 ──
print(f"\n{'-'*104}")
print("  실기판정 통과율 (|w| 기준)")
for c in ctrls:
    sub = [x for x in rows if x['ctrl'] == c]
    bad = sum(1 for x in sub if str(x.get('omega_fail','')).lower().startswith('t'))
    print(f"    {c:>9s}  {len(sub)-bad}/{len(sub)} 통과")
print(f"{'-'*104}\n")
