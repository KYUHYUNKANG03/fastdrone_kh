"""Prepare, inspect, then explicitly run bounded DEVELOPMENT validation stages.

Planning never constructs a controller or launches a simulation. Run requires
an unchanged saved plan and a passing local development preflight. Paper runs
and tuning are deliberately outside this tool.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.arena import check_confirmed_facts, config_sha256, load_config, validate_config
from control.sensor_binding import nonnegative_seed, resolve_feedback, runtime_source_hashes
from control.sensor_campaign import run_campaign
from control.sensor_matching_screen import sensor_group_profile, write_atomic
from scripts.sensor_reproduce import canonical_source_hashes

CONFIG = 'configs/arena_rotor_projected_development_v9.json'
CONTROLLERS = ('V13', 'F13', 'M17', 'GSLQR', 'CPID')
STAGES = ('integration', 'gnss_startup')


def profile_for(base, context, noise=True, delay=True):
    nominal = resolve_feedback(base).profile
    if nominal is None:
        raise ValueError('preparation requires a sensor-feedback configuration')
    quiet = sensor_group_profile(nominal, 'quiet_sampled')
    if context == 'quiet':
        return quiet
    if context == 'nominal':
        return nominal
    if context not in ('isolated', 'full'):
        raise ValueError('unknown GNSS context')
    profile = deepcopy(quiet if context == 'isolated' else nominal)
    # Keep nominal process, measurement and initialization covariances in every
    # cell. The generated GNSS noise/latency alone forms the factorial axes.
    profile['estimator'] = deepcopy(quiet['estimator'])
    profile['estimator']['trace_updates'] = True
    profile['gnss'] = deepcopy(nominal['gnss'])
    if not noise:
        profile['gnss'].update(pos_sigma=0., vel_sigma=0.)
    if not delay:
        profile['gnss']['latency_s'] = 0.
    profile['name'] = f'gnss_{context}_noise{int(noise)}_delay{int(delay)}'
    return profile


def make_plan(config=CONFIG, seeds=(3, 4, 5), timeout_s=1200.):
    base = load_config(ROOT/config)
    if not seeds or len(seeds) != len(set(seeds)):
        raise ValueError('development seeds must be nonempty and unique')
    for seed in seeds:
        nonnegative_seed(seed)
        if any(lo <= seed <= hi for lo, hi in base['seeds'].values()):
            raise ValueError('development cannot consume reserved tuning/main seeds')
    if not 0 < timeout_s < float('inf'):
        raise ValueError('timeout must be positive and finite')
    violations = check_confirmed_facts(base)
    if violations:
        raise ValueError('arena facts: '+', '.join(violations))
    gnss = resolve_feedback(base).profile['gnss']
    if any(gnss[k] != [0., 0., 0.] for k in ('pos_bias', 'vel_bias')) or any(
            gnss.get(k) for k in ('dropout_prob', 'outage_windows', 'outlier_windows')):
        raise ValueError('GNSS factorial requires nominal bias/dropout/outlier-free GNSS')
    tasks = []

    def task(stage, controller, case, condition, seed, profile=None, truth_id=None):
        cfg = deepcopy(base)
        cfg['sensor_feedback'] = dict(schema='sensor_feedback/1',
            mode='truth' if profile is None else 'sensors', seed=seed)
        if profile is not None:
            cfg['sensor_feedback']['profile'] = deepcopy(profile)
        validate_config(cfg)
        trial_id = f'{stage}/{controller}/{case}/{condition}/seed_{seed}'
        tasks.append(dict(trial_id=trial_id, stage=stage, controller=controller,
            case=case, condition=condition, seed=seed,
            feedback=cfg['sensor_feedback']['mode'], truth_trial_id=truth_id,
            config=cfg, config_sha256=config_sha256(cfg)))
        return trial_id

    available = {s['id'] for s in base['scenarios']}
    for controller in CONTROLLERS:
        for speed in ('VL', 'VH'):
            if controller == 'CPID' and speed == 'VH':
                continue  # The team's declared CPID design region.
            case = f'gust_lateral_p10_{speed}'
            if case not in available:
                raise ValueError(f'missing case {case}')
            truth = task('integration', controller, case, 'truth', 0)
            task('integration', controller, case, 'quiet_sampled', 0,
                 profile_for(base, 'quiet'), truth)
            for seed in seeds:
                task('integration', controller, case, 'nominal', seed,
                     profile_for(base, 'nominal'), truth)
    case = 'gust_lateral_p10_VL'
    for controller in ('V13', 'F13'):
        # Shared deterministic truth control, explicitly referenced across stages.
        truth = next(t['trial_id'] for t in tasks if t['controller'] == controller
                     and t['case'] == case and t['feedback'] == 'truth')
        for context in ('isolated', 'full'):
            for noise in (False, True):
                for delay in (False, True):
                    profile = profile_for(base, context, noise, delay)
                    for seed in seeds:
                        task('gnss_startup', controller, case, profile['name'], seed, profile, truth)
    return dict(schema='sensor_preparation/1', stage='DEVELOPMENT',
        base_config=str(config), base_config_sha256=config_sha256(base),
        seeds=list(seeds), timeout_s=timeout_s,
        runtime_source_sha256=canonical_source_hashes(runtime_source_hashes()),
        driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        tooling_source_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            for name in ('scripts/sensor_preparation.py', 'scripts/sensor_reproduce.py',
                         'scripts/sensor_preflight.py', 'scripts/setup_env.py')},
        tasks=tasks, counts={s: sum(t['stage'] == s for t in tasks) for s in STAGES},
        note='Planned trials, not results. Fixed team gains; no final tuning or held-out seeds. '
             'Quiet sampled feedback is diagnostic. Same seeds pair random draws, not trajectories. '
             'Deterministic controls and zero-noise repeated cells are not independent evidence.')


def save_plan(output, plan):
    output = Path(output)
    path = output/'plan.json'
    if path.exists():
        if json.loads(path.read_text(encoding='utf-8')) != plan:
            raise ValueError('plan changed; choose a new directory')
    elif output.exists() and any(output.iterdir()):
        raise ValueError('nonempty output has no plan')
    output.mkdir(parents=True, exist_ok=True)
    write_atomic(path, plan)


def checked_plan(output):
    plan = json.loads((Path(output)/'plan.json').read_text(encoding='utf-8'))
    if plan.get('schema') != 'sensor_preparation/1' or plan.get('stage') != 'DEVELOPMENT':
        raise ValueError('expected a DEVELOPMENT preparation plan')
    current = make_plan(plan['base_config'], plan['seeds'], plan['timeout_s'])
    if current != plan:
        raise ValueError('saved plan/source/configuration changed; prepare a new directory')
    return plan


def execute(output, stage, runner=run_campaign):
    output = Path(output).resolve()
    if stage not in STAGES:
        raise ValueError('unknown preparation stage')
    plan = checked_plan(output)
    # One writer for the whole plan, even when running different stages.
    lock = output/'run.lock'
    with lock.open('x', encoding='utf-8') as file:
        file.write(json.dumps(dict(pid=os.getpid(), stage=stage))+'\n')
    try:
        return _execute(output, stage, plan, runner)
    finally:
        lock.unlink()


def _execute(output, stage, plan, runner):
    plan_hash = config_sha256(plan)
    path = output/'results.json'
    records = {}
    if path.exists():
        previous = json.loads(path.read_text(encoding='utf-8'))
        if previous.get('plan_sha256') != plan_hash:
            raise ValueError('result plan hash differs')
        records = {r['trial_id']: r for r in previous['records']}
        tasks = {t['trial_id']: t for t in plan['tasks']}
        if len(records) != len(previous['records']) or set(records)-set(tasks):
            raise ValueError('duplicate or unknown recorded trials')
        for tid, row in records.items():
            if row.get('input_config_sha256') != tasks[tid]['config_sha256']:
                raise ValueError('recorded input config differs')

    selected = {t['trial_id'] for t in plan['tasks'] if t['stage'] == stage}
    selected |= {t['truth_trial_id'] for t in plan['tasks']
                 if t['trial_id'] in selected and t['truth_trial_id']}

    def save():
        doc = dict(schema='sensor_preparation_results/1', stage='DEVELOPMENT',
            plan_sha256=plan_hash, expected_trials=len(plan['tasks']),
            complete=len(records) == len(plan['tasks']), records=list(records.values()))
        write_atomic(path, doc)
        return doc

    save()
    for task in plan['tasks']:
        tid = task['trial_id']
        if tid not in selected or tid in records:
            continue
        # Keep Windows paths short while retaining readable IDs in the plan.
        storage_id = 't_'+hashlib.sha256(tid.encode()).hexdigest()[:20]
        folder = output/'inputs'/storage_id
        folder.mkdir(parents=True, exist_ok=True)
        cfg, profile = folder/'arena.json', folder/'sensors.json'
        write_atomic(cfg, task['config'])
        if task['feedback'] == 'sensors':
            write_atomic(profile, task['config']['sensor_feedback']['profile'])
        print(f'START {tid}', flush=True)
        result = runner(cfg, profile, output/'runs'/storage_id, cases=[task['case']],
            controllers=[task['controller']], seed=task['seed'],
            feedback=task['feedback'], timeout_s=plan['timeout_s'])
        if len(result['records']) != 1:
            raise ValueError(f'{tid}: expected one recorded trial')
        row = dict(result['records'][0], trial_id=tid, stage=task['stage'],
            condition=task['condition'], truth_trial_id=task['truth_trial_id'],
            input_config_sha256=task['config_sha256'])
        records[tid] = row
        save()
        print(f'DONE {tid}: passed={row.get("passed")} duration={row.get("simulated_seconds")}', flush=True)
        if 'no_trial_output' in row.get('failure_reasons', []):
            raise RuntimeError(f'{tid}: no trial output; inspect saved failure before continuing')
    return save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prepare = sub.add_parser('plan')
    prepare.add_argument('--config', default=CONFIG)
    prepare.add_argument('--seeds', nargs='+', type=int, default=[3, 4, 5])
    prepare.add_argument('--timeout-s', type=float, default=1200.)
    prepare.add_argument('--output', type=Path, required=True)
    run = sub.add_parser('run')
    run.add_argument('--output', type=Path, required=True)
    run.add_argument('--stage', choices=STAGES, required=True)
    args = parser.parse_args(argv)
    if args.command == 'plan':
        plan = make_plan(args.config, args.seeds, args.timeout_s)
        save_plan(args.output, plan)
        print(json.dumps(dict(planned=True, simulations_started=0, counts=plan['counts'],
            total=len(plan['tasks']), plan_sha256=config_sha256(plan)), indent=2))
    else:
        from scripts.sensor_preflight import audit, configure_process
        configure_process()
        plan = checked_plan(args.output)
        result = audit(plan['base_config'])
        write_atomic(args.output/'preflight.json', result)
        if not result['development_ready']:
            print(json.dumps(result, indent=2))
            return 1
        execute(args.output, args.stage)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
