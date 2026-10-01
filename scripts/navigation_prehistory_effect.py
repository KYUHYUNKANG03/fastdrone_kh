"""D5 effect check (SENSOR_DECISIONS 'D5 결정' 검증 3): one smoke trial per call, then a table.

run:     python scripts/navigation_prehistory_effect.py run <variant> <controller> <seed> <case> <prehistory_s> <out_dir>
analyze: python scripts/navigation_prehistory_effect.py analyze <results_dir> <out_md> [--e1 <e_iso_dir>]

`run` follows the E1 isolation procedure (fast_drone-arena results/sensor_d1_d6_isolation_2026-10-01/e_run.py):
v9 config, `sensor_group_profile` for variants other than full, validate_config, then the validation suite
smoke runner for one case/controller/seed. The only additions are the case argument and, when
prehistory_s > 0, estimator.navigation_prehistory_s. prehistory_s = 0 writes the profile exactly as E1 did.
This check is for reporting only; it does not choose the prehistory length.

Results layout: <results_dir>/<on|off>/<case>/<variant>/<controller>_s<seed>/arena_*/{trials.jsonl,*.npz}.
"""
import glob
import json
import os
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIG = 'configs/arena_rotor_projected_development_v9.json'
WINDOWS = ((0.0, 0.5), (0.5, 3.0), (3.0, 1e9))


def run(variant, controller, seed, case, prehistory, out):
    sys.path.insert(0, str(ROOT))
    os.chdir(ROOT)
    os.makedirs(out, exist_ok=True)
    from control.arena import load_config, validate_config
    from control.arena_sensors import load_sensor_profile
    from control.sensor_binding import resolve_feedback
    from control.sensor_matching_screen import sensor_group_profile

    config = load_config(CONFIG)
    if variant != 'full':
        profile = dict(sensor_group_profile(resolve_feedback(config).profile, variant))
        profile['name'] = f'{variant}_nominal'
        config['sensor_feedback']['profile'] = load_sensor_profile(profile)
        config = validate_config(config)
    duration = float(prehistory)
    if duration:
        profile = dict(config['sensor_feedback']['profile'])
        profile['estimator'] = dict(profile['estimator'], navigation_prehistory_s=duration)
        config['sensor_feedback']['profile'] = load_sensor_profile(profile)
        config = validate_config(config)
    config_path = os.path.join(out, 'config.json')
    with open(config_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=1, allow_nan=False)
    from control.validation_suite import main as suite_main
    return suite_main(['--config', config_path, '--smoke', '--only-cases', case,
                       '--only-controllers', controller, '--sensor-seed', str(int(seed)), '--output', out])


def trial_row(trial_dir, p, domain_status, Rotation):
    jl = glob.glob(os.path.join(trial_dir, 'arena_*', 'trials.jsonl'))
    npz = glob.glob(os.path.join(trial_dir, 'arena_*', '*.npz'))
    if not jl or not npz:
        return dict(status='missing')
    record = json.loads(open(jl[0], encoding='utf-8').readline())
    d = np.load(npz[0], allow_pickle=True)
    ts, xs, wind = d['ts'], d['xs'], d['wind']
    n = len(wind)
    inside = np.ones(n, dtype=bool)
    for k in range(n):
        # Same check as validation_suite.run_trial: truth after each step, wind of that step.
        x = xs[k+1]
        vb = Rotation.from_quat(x[6:10]).as_matrix().T @ (x[3:6]-wind[k])
        inside[k] = domain_status(p, x[13:17], vb)['inside_assumed_domain']
    t_end = ts[1:n+1]
    rpm = xs[1:n+1, 13:17].min(axis=1)*60/(2*np.pi)
    early = ts <= .2+1e-9
    row = dict(status='ok', passed=record.get('passed'), tracking=record.get('tracking_pass'),
               domain=record.get('model_domain_valid'), reasons=record.get('failure_reasons'),
               stop_reason=record.get('stop_reason'), simulated_s=record.get('simulated_seconds'),
               outside_frac_recorded=record.get('prop_domain_outside_fraction'),
               outside_frac_recomputed=float((~inside).mean()) if n else None,
               first_exit_s=float(t_end[~inside][0]) if (~inside).any() else None,
               est_z_error_max_0_0p2=float(np.max(np.abs(d['xs_est'][early, 2]-xs[early, 2]))),
               est_pos_error_at_0=float(np.linalg.norm(d['xs_est'][0, 0:3]-xs[0, 0:3])),
               window_rmse_velocity=record.get('window_rmse_velocity'), window_rmse_z=record.get('window_rmse_z'),
               trajectory_sha256=record.get('trajectory_sha256'),
               sensor_profile_sha256=record.get('sensor_profile_sha256'), wall_seconds=record.get('wall_seconds'),
               navigation_prehistory=record.get('estimator_diagnostics', {}).get('navigation_prehistory'))
    for lo, hi in WINDOWS:
        m = (t_end > lo) & (t_end <= hi)
        key = f'{lo:g}-{hi:g}' if hi < 1e8 else f'{lo:g}-'
        row[f'exits_{key}'] = int((~inside & m).sum())
        row[f'minrpm_{key}'] = float(rpm[m].min()) if m.any() else None
    return row


def analyze(base, out, e1=None):
    sys.path.insert(0, str(ROOT))
    from scipy.spatial.transform import Rotation
    from models.team_light.control.baseline_v2 import baseline_params
    from models.team_light.control.propeller_curve import domain_status
    p = baseline_params()
    rows = []
    sources = [('ours', base)] + ([('e1', e1)] if e1 else [])
    for source, root in sources:
        pattern = (os.path.join(root, '*', '*', '*', '*_s*') if source == 'ours'
                   else os.path.join(root, '*', '*_s*'))
        for trial_dir in sorted(glob.glob(pattern)):
            if not os.path.isdir(trial_dir):
                continue
            parts = Path(trial_dir).relative_to(root).parts
            if source == 'ours':
                condition, case, variant = parts[0], parts[1], parts[2]
            else:
                condition, case, variant = 'off_e1', 'gust_lateral_p10_VL', parts[0]
                if variant not in ('full', 'navigation_only'):
                    continue
            ctrl, seed = parts[-1].rsplit('_s', 1)
            row = dict(source=source, condition=condition, case=case, variant=variant, ctrl=ctrl, seed=int(seed))
            row.update(trial_row(trial_dir, p, domain_status, Rotation))
            rows.append(row)
    Path(out).with_suffix('.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1)+'\n', encoding='utf-8')
    fmt = lambda v, f='{:.0f}': '—' if v is None else f.format(v)
    lines = ['| condition | case | variant | ctrl | seed | pass | domain | first exit s | exits 0-0.5 / 0.5-3 / 3- '
             '| min RPM 0-0.5 | max est z err 0-0.2 s (m) | window RMSE v / z |',
             '|---|---|---|---|---:|---|---|---:|---|---:|---:|---|']
    for r in sorted(rows, key=lambda r: (r['case'], r['variant'], r['ctrl'], r['seed'], r['condition'])):
        if r['status'] != 'ok':
            lines.append(f"| {r['condition']} | {r['case']} | {r['variant']} | {r['ctrl']} | {r['seed']} | MISSING |||||||")
            continue
        lines.append(f"| {r['condition']} | {r['case']} | {r['variant']} | {r['ctrl']} | {r['seed']} | {r['passed']} | "
                     f"{r['domain']} | {fmt(r['first_exit_s'], '{:.3f}')} | "
                     f"{r['exits_0-0.5']} / {r['exits_0.5-3']} / {r['exits_3-']} | {fmt(r['minrpm_0-0.5'])} | "
                     f"{fmt(r['est_z_error_max_0_0p2'], '{:.3f}')} | "
                     f"{fmt(r['window_rmse_velocity'], '{:.3f}')} / {fmt(r['window_rmse_z'], '{:.3f}')} |")
    Path(out).write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(f'{len(rows)} trials -> {out}')
    return rows


if __name__ == '__main__':
    if sys.argv[1] == 'run':
        sys.exit(run(*sys.argv[2:8]))
    elif sys.argv[1] == 'analyze':
        args = sys.argv[2:]
        e1 = args[args.index('--e1')+1] if '--e1' in args else None
        analyze(args[0], args[1], e1)
    else:
        raise SystemExit(__doc__)
