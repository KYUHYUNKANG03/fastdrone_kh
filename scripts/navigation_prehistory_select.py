"""D5: choose the navigation prehistory length from covariance only.

Common setting (SENSOR_DECISIONS 'D5 결정', kj 2026-10-01): v9
navigation_only profile, sensor seed 3. Start states are built exactly as
validation_suite.run_trial builds them: hover 0 m/s (ref_accel_0_VH_rho1),
20 m/s (gust_lateral_p10_VL), 85 m/s (gust_lateral_p10_VH). sigma_i(T) =
sqrt(P_ii) for position, velocity and attitude right after initial_packets(0),
i.e. what the controller's first estimate carries. Each state takes its own
pick; the result is the largest over the three states. No controller, no plant
integration and no tracking metric is used by either rule.

Original rule (pre-registered): candidates {1, 2, 3, 5, 8} s; T_j passes when
every |sigma_i(T_{j+1})/sigma_i(T_j) - 1| <= 0.05. It selected nothing: the
bias-driven attitude and velocity terms were still changing 25-32 % per step.
Revised rule (kj 2026-10-01, set after seeing the covariance table and before
any performance result): sigma(120 s) is the steady state; pick the smallest
candidate in {1, 2, 3, 5, 8, 13, 20, 30, 45, 60} s whose every component is
within 5 % of it.

python scripts/navigation_prehistory_select.py --output results/navigation_prehistory_2026-10-01
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from control.arena import build_scenarios, load_config, config_sha256  # noqa: E402
from control.arena_factory import ArenaFactory  # noqa: E402
from control.arena_feedback import ArenaSensorFeedback  # noqa: E402
from control.arena_plant_wrench import build_plant  # noqa: E402
from control.arena_sensors import load_sensor_profile  # noqa: E402
from control.sensor_binding import FeedbackBinding, resolve_feedback, runtime_source_hashes  # noqa: E402
from control.sensor_matching_screen import sensor_group_profile  # noqa: E402
from control.validation_suite import plant_truth  # noqa: E402
from models.team_light.control.trim import find_trim  # noqa: E402

CONFIG = 'configs/arena_rotor_projected_development_v9.json'
CANDIDATES = (1., 2., 3., 5., 8.)
REVISED_CANDIDATES = (1., 2., 3., 5., 8., 13., 20., 30., 45., 60.)
STEADY_STATE_S = 120.
STATES = (('hover_0', 'ref_accel_0_VH_rho1'), ('V_L_20', 'gust_lateral_p10_VL'),
          ('V_H_85', 'gust_lateral_p10_VH'))
SEED, TOLERANCE = 3, .05
COMPONENTS = ('pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'att_x', 'att_y', 'att_z')


def start_state(factory, scenario):
    """validation_suite.run_trial lines up to the sensor initialization, nothing else."""
    case = scenario.cases[0]
    truth = plant_truth(factory.p, case)
    if 'cg_offset_axis' in case.get('extra_params', {}):
        raise ValueError('CG-offset start states are not part of this selection')
    initial_v, initial_z, _ = scenario.profile.get_ref(0.)
    trim = find_trim(truth, float(initial_v[0]))
    x = trim['state'].copy()
    x[2] = initial_z
    plant = build_plant(truth, dt=factory.dt)
    xdot0 = np.asarray(plant.evaluate_xdot(x, trim['control'], np.zeros(3)), dtype=float)
    return x, xdot0, trim['control']


def navigation_only(base_profile, duration):
    profile = dict(sensor_group_profile(base_profile, 'navigation_only'))
    profile['name'] = 'navigation_only_nominal'
    if duration:
        profile['estimator'] = dict(profile['estimator'], navigation_prehistory_s=duration)
    return load_sensor_profile(profile)


def handover_sigma(profile, x, xdot0, command, dt):
    started = perf_counter()
    feedback = ArenaSensorFeedback(x, profile, dt, seed=SEED)
    feedback.initial_packets(0., x, xdot0, initial_command=command)
    return np.sqrt(np.diag(feedback.filter.P)[:9]), perf_counter()-started


def select(sigmas):
    """sigmas: {T: array(9)} -> (smallest passing T or None, per-candidate max relative change)."""
    changes = {}
    for left, right in zip(CANDIDATES, CANDIDATES[1:]):
        changes[left] = np.abs(sigmas[right]/sigmas[left]-1.)
    passing = [T for T in CANDIDATES[:-1] if np.all(changes[T] <= TOLERANCE)]
    return (passing[0] if passing else None), changes


def select_revised(sigmas):
    """Smallest revised candidate with every component within tolerance of sigma(120 s)."""
    steady = sigmas[STEADY_STATE_S]
    distance = {T: np.abs(sigmas[T]/steady-1.) for T in REVISED_CANDIDATES}
    passing = [T for T in REVISED_CANDIDATES if np.all(distance[T] <= TOLERANCE)]
    return (passing[0] if passing else None), distance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    config = load_config(ROOT/CONFIG)
    base = resolve_feedback(config).profile
    factory = ArenaFactory(config)
    scenarios = {s.id: s for s in build_scenarios(config, factory.cp, factory.p,
                                                   only=[sid for _, sid in STATES])}
    lengths = sorted(set(CANDIDATES) | set(REVISED_CANDIDATES) | {STEADY_STATE_S})
    report = dict(schema='navigation_prehistory_selection/2', config=CONFIG, config_sha256=config_sha256(config),
                  setting=dict(seed=SEED, relative_tolerance=TOLERANCE, components=COMPONENTS,
                               measured='sqrt(diag P) after initial_packets(0)', combined='largest over states'),
                  original_rule=dict(candidates_s=CANDIDATES, status='pre-registered',
                                     per_state='smallest T_j with all |sigma(T_j+1)/sigma(T_j)-1| <= tol'),
                  revised_rule=dict(candidates_s=REVISED_CANDIDATES, steady_state_s=STEADY_STATE_S,
                                    status='kj 2026-10-01: set after the covariance table, before any performance result',
                                    per_state='smallest T with all |sigma(T)/sigma(steady)-1| <= tol'),
                  runtime_source_sha256=runtime_source_hashes(), states={})
    original, revised = [], []
    for name, scenario_id in STATES:
        x, xdot0, command = start_state(factory, scenarios[scenario_id])
        sigmas, seconds = {}, {}
        sigma0, _ = handover_sigma(navigation_only(base, 0.), x, xdot0, command, factory.dt)
        for T in lengths:
            sigmas[T], seconds[T] = handover_sigma(navigation_only(base, T), x, xdot0, command, factory.dt)
            print(f'{name} T={T:g}: {seconds[T]:.1f} s', flush=True)
        pick, changes = select(sigmas)
        pick_revised, distance = select_revised(sigmas)
        original.append(pick); revised.append(pick_revised)
        report['states'][name] = dict(
            scenario_id=scenario_id, speed_m_s=float(x[3]), altitude_m=float(x[2]),
            trim_specific_force_residual_m_s2=float(np.linalg.norm(xdot0[3:6])),
            profile_sha256=FeedbackBinding('sensors', navigation_only(base, CANDIDATES[0]), SEED).metadata[
                'sensor_profile_sha256'],
            sigma_without_prehistory=sigma0.tolist(),
            sigma={str(T): sigmas[T].tolist() for T in lengths},
            original_relative_change_to_next={str(T): changes[T].tolist() for T in CANDIDATES[:-1]},
            revised_distance_to_steady={str(T): distance[T].tolist() for T in REVISED_CANDIDATES},
            wall_seconds={str(T): seconds[T] for T in lengths},
            original_selected_s=pick, revised_selected_s=pick_revised)
    combined = lambda picks: None if any(p is None for p in picks) else max(picks)
    report['original_rule']['selected_s'] = combined(original)
    report['revised_rule']['selected_s'] = combined(revised)
    report['selected_s'] = report['revised_rule']['selected_s']
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'t_pre_selection.json').write_text(json.dumps(report, indent=1)+'\n', encoding='utf-8')
    names = [n for n, _ in STATES]
    lines = [f"# T_pre selection (covariance only, seed {SEED}, navigation_only)", '',
             f"- Original pre-registered rule (consecutive candidates within 5 %): "
             f"**{report['original_rule']['selected_s']}** — per state {dict(zip(names, original))}.",
             f"- Revised rule (within 5 % of sigma({STEADY_STATE_S:g} s); kj, after this covariance table, "
             f"before any performance result): **{report['revised_rule']['selected_s']} s** — "
             f"per state {dict(zip(names, revised))}.", '']
    for name, state in report['states'].items():
        lines += [f"## {name} ({state['scenario_id']}, {state['speed_m_s']:g} m/s)", '',
                  '| T (s) | ' + ' | '.join(COMPONENTS) + ' | max change to next (original) '
                  '| max distance to steady (revised) | wall s |',
                  '|---|' + '---:|'*(len(COMPONENTS)+3)]
        lines.append('| 0 (none) | ' + ' | '.join(f'{v:.4g}' for v in state['sigma_without_prehistory'])
                     + ' | — | — | — |')
        for T in lengths:
            change = state['original_relative_change_to_next'].get(str(T))
            distance = state['revised_distance_to_steady'].get(str(T))
            lines.append(f'| {T:g} | ' + ' | '.join(f'{v:.4g}' for v in state['sigma'][str(T)]) + ' | '
                         + ('—' if change is None else f'{100*max(change):.2f}%') + ' | '
                         + ('—' if distance is None else f'{100*max(distance):.2f}%')
                         + f" | {state['wall_seconds'][str(T)]:.2f} |")
        lines += ['', f"original: {state['original_selected_s']} s, revised: {state['revised_selected_s']} s", '']
    lines.append('Units: position m, velocity m/s, attitude rad (error-state sqrt(diag P)). '
                 'Wall seconds: one prehistory plus the t = 0 sample, single process.')
    (args.output/'t_pre_selection.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(f"original rule: {report['original_rule']['selected_s']}; revised rule: {report['selected_s']}")
    return report


if __name__ == '__main__':
    main()
