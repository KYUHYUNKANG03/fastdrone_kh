"""Paired truth/fusion DEVELOPMENT screen; no gain or runtime modifications.

Every distinct physical scenario gets one deterministic truth control per
controller. Sensor conditions/seeds share that control, never count copies of
it as independent trials. The immutable manifest supports interrupted resumes.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.arena import check_confirmed_facts, config_sha256, load_config, validate_config
from control.arena_sensors import load_sensor_profile
from control.sensor_binding import nonnegative_seed, runtime_source_hashes
from control.sensor_campaign import run_campaign
from control.sensor_matching_screen import write_atomic

CONFIG = ROOT/'configs/arena_rotor_projected_development_v9.json'
CONDITIONS = {
    'lateral_high': 'gust_lateral_p10_VH',
    'lateral_low': 'gust_lateral_p10_VL',
    'vertical_high': 'gust_vertical_m5_VH',
    'vertical_low': 'gust_vertical_m5_VL',
    'acceleration': 'ref_accel_0_VH_rho1',
    'braking': 'ref_brake_VH_VL_rho1',
    'mission': 'mission_VH',
    'altitude_low': None,
    'plant_tau10ms': 'gust_lateral_p10_VH',
    'plant_tau40ms': 'gust_lateral_p10_VH',
    'rpm_outage200ms': 'gust_lateral_p10_VH',
    'rpm_latency20ms': 'gust_lateral_p10_VH',
    'rpm_rate100hz': 'gust_lateral_p10_VH',
    'gnss_outage1s': 'gust_lateral_p10_VH',
    'initial_error': 'gust_lateral_p10_VH',
}
DEFAULT_CONDITIONS = ['lateral_low', 'vertical_high', 'acceleration', 'altitude_low',
                      'plant_tau10ms', 'plant_tau40ms', 'rpm_outage200ms', 'initial_error']


def condition_config(base, name):
    if name not in CONDITIONS:
        raise ValueError('unknown envelope condition')
    result = deepcopy(base)
    profile = result.get('sensor_feedback', {}).get('profile')
    if (result.get('sensor_feedback', {}).get('mode') != 'sensors' or not profile
            or profile['rotor_observer']['source'] != 'telemetry_predictor'
            or profile['rotor_observer']['motor_tau_s'] != .02):
        raise ValueError('screen requires the fixed 20 ms telemetry-predictor candidate')
    if name == 'altitude_low':
        scenario = dict(id='dev_altitude_p1_VL', type='step', speed='V_L',
                        axis='altitude', size=1., durations_s=[8.])
    else:
        scenario = deepcopy(next(s for s in base['scenarios'] if s['id'] == CONDITIONS[name]))
    if name.startswith('plant_tau'):
        if scenario.get('perturbation') or scenario.get('extra_params'):
            raise ValueError('motor mismatch requires an unperturbed reference scenario')
        scenario['perturbation'] = {'motor_tau': .5 if name == 'plant_tau10ms' else 2.}
        scenario['id'] += '_'+name
    if name == 'rpm_outage200ms':
        profile['rpm']['outage_windows'] = [[3.5, 3.7]]
    elif name == 'rpm_latency20ms':
        profile['rpm']['latency_s'] = .02
    elif name == 'rpm_rate100hz':
        profile['rpm']['rate_hz'] = 100.
    elif name == 'gnss_outage1s':
        profile['gnss']['outage_windows'] = [[3.5, 4.5]]
    elif name == 'initial_error':
        profile['estimator'].update(initial_position_error_m=[.2, -.1, .3],
            initial_velocity_error_m_s=[.1, -.1, .05], initial_attitude_error_deg=[.5, -.5, 1.])
    profile['name'] = 'envelope_'+name
    result['sensor_feedback']['profile'] = load_sensor_profile(profile)
    if scenario['id'] not in {s['id'] for s in result['scenarios']}:
        result['scenarios'].append(scenario)
    validate_config(result)
    violations = check_confirmed_facts(result)
    if violations:
        raise ValueError('arena facts: '+', '.join(violations))
    return result


def condition_case(name):
    case = CONDITIONS[name] or 'dev_altitude_p1_VL'
    return case+'_'+name if name.startswith('plant_tau') else case


def make_tasks(base, controllers, seeds, conditions):
    for items in (controllers, seeds, conditions):
        if not items or len(items) != len(set(items)):
            raise ValueError('axes must be nonempty and unique')
    if set(controllers) - {'V13', 'F13'}:
        raise ValueError('this diagnostic screen supports V13/F13')
    for seed in seeds:
        nonnegative_seed(seed)
        if any(lo <= seed <= hi for lo, hi in base['seeds'].values()):
            raise ValueError('development cannot consume reserved tuning/main seeds')
    tasks = {}
    for name in conditions:
        fused = condition_config(base, name)
        truth = deepcopy(fused)
        truth['sensor_feedback'] = dict(schema='sensor_feedback/1', mode='truth', seed=0)
        case = condition_case(name)
        for controller in controllers:
            truth_id = f'truth/{controller}/{case}/{config_sha256(truth)[:16]}'
            tasks.setdefault(truth_id, dict(trial_id=truth_id, condition='truth',
                controller=controller, feedback='truth', seed=0, case=case, config=truth))
            for seed in seeds:
                config = deepcopy(fused)
                config['sensor_feedback']['seed'] = seed
                trial_id = f'{name}/{controller}/seed_{seed}'
                tasks[trial_id] = dict(trial_id=trial_id, condition=name, controller=controller,
                    feedback='sensors', seed=seed, case=case, config=config,
                    truth_trial_id=truth_id)
    return list(tasks.values())


def run(config, output, controllers, seeds, conditions, timeout_s=1200., runner=run_campaign):
    base = load_config(config)
    if not 0 < timeout_s < float('inf'):
        raise ValueError('timeout must be positive and finite')
    tasks = make_tasks(base, controllers, seeds, conditions)
    contract = dict(schema='sensor_envelope_screen/1', stage='DEVELOPMENT',
        base_config=base, base_config_sha256=config_sha256(base),
        controllers=controllers, seeds=seeds, conditions=conditions, timeout_s=timeout_s,
        driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        runtime_source_sha256=runtime_source_hashes(), tasks=tasks)
    output = Path(output).resolve()
    path = output/'experiment.json'
    records = {}
    if path.exists():
        previous = json.loads(path.read_text(encoding='utf-8'))
        if previous['contract'] != contract:
            raise ValueError('screen contract changed; use a new output directory')
        records = {r['trial_id']: r for r in previous['records']}
        if len(records) != len(previous['records']) or set(records) - {t['trial_id'] for t in tasks}:
            raise ValueError('invalid recorded trial identities')
    elif output.exists() and any(output.iterdir()):
        raise ValueError('nonempty output has no experiment contract')
    output.mkdir(parents=True, exist_ok=True)

    def save():
        doc = dict(contract=contract, records=list(records.values()),
                   expected_trials=len(tasks), complete=len(records) == len(tasks))
        write_atomic(path, doc)
        return doc

    save()
    for task in tasks:
        trial_id = task['trial_id']
        if trial_id in records:
            continue
        folder = output/'inputs'/trial_id
        folder.mkdir(parents=True, exist_ok=True)
        config_path, profile_path = folder/'arena.json', folder/'sensors.json'
        write_atomic(config_path, task['config'])
        if task['feedback'] == 'sensors':
            write_atomic(profile_path, task['config']['sensor_feedback']['profile'])
        print(f'START {trial_id} case={task["case"]}', flush=True)
        campaign = runner(config_path, profile_path, output/'runs'/trial_id,
            cases=[task['case']], controllers=[task['controller']], seed=task['seed'],
            timeout_s=timeout_s, feedback=task['feedback'])
        if len(campaign['records']) != 1:
            raise ValueError(f'{trial_id}: expected one recorded trial')
        row = dict(campaign['records'][0], trial_id=trial_id, condition=task['condition'],
            truth_trial_id=task.get('truth_trial_id'), input_config_sha256=config_sha256(task['config']))
        records[trial_id] = row
        save()
        print(f'DONE {trial_id} tracking={row.get("tracking_pass")} '
              f'domain={row.get("model_domain_valid")} duration={row.get("simulated_seconds")} '
              f'reasons={row.get("failure_reasons")}', flush=True)
        if 'no_trial_output' in row.get('failure_reasons', []):
            raise RuntimeError(f'{trial_id}: no trial output; inspect campaign.log before continuing')
    return save()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=CONFIG)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--controllers', nargs='+', choices=['V13', 'F13'], default=['V13', 'F13'])
    p.add_argument('--seeds', nargs='+', type=int, default=[3])
    p.add_argument('--conditions', nargs='+', choices=list(CONDITIONS), default=DEFAULT_CONDITIONS)
    p.add_argument('--timeout-s', type=float, default=1200.)
    p.add_argument('--plan-only', action='store_true', help='print counts without creating output or running trials')
    a = p.parse_args()
    if a.plan_only:
        tasks = make_tasks(load_config(a.config), a.controllers, a.seeds, a.conditions)
        print(json.dumps(dict(total=len(tasks), truth=sum(t['feedback']=='truth' for t in tasks),
            sensors=sum(t['feedback']=='sensors' for t in tasks), conditions=a.conditions), indent=2))
        return
    run(a.config, a.output, a.controllers, a.seeds, a.conditions, a.timeout_s)


if __name__ == '__main__':
    main()
