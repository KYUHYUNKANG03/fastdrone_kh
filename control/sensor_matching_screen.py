"""Paired development ablations on the corrected arena model, with strict resume.

One subprocess per controller/case/seed prevents partial multi-case output from
silently losing trials. This runner never labels results as final paper data.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

from control.arena import ROOT, load_config, validate_config, config_sha256
from control.arena_sensors import load_sensor_profile
from control.sensor_binding import nonnegative_seed, resolve_feedback, runtime_source_hashes
from control.sensor_campaign import run_campaign

GROUP_VARIANTS = ('quiet_sampled', 'imu_only', 'navigation_only', 'rotor_only')
VARIANTS = ('baseline', 'startup_guard', 'gyro_quiet', 'rotor_tau2ms',
            'cutoff25', 'aligned50', 'guarded_fast', 'guarded_fast25') + GROUP_VARIANTS
FAULTS = ('nominal', 'baro_drift', 'gnss_outlier', 'baro_outlier', 'gnss_outage',
          'gyro_vibration', 'initial_error')


def sensor_group_profile(profile, variant):
    """Remove sensor errors, then restore one nominal group for diagnosis.

    Rates, enabled sensors, clipping, initialization and navigation algorithm
    stay fixed. Pin the filter's nominal Q and preflight covariance explicitly:
    changing the generated IMU/barometer noise must not also retune the filter.
    This is a sampled, quiet measurement control, not truth-state feedback.
    """
    if variant not in GROUP_VARIANTS:
        raise ValueError('unknown sensor group variant')
    nominal = load_sensor_profile(profile)
    result = deepcopy(nominal)
    e, imu = result['estimator'], nominal['imu']
    for target, source in (('accel_noise_density', 'accel_noise_density'),
                           ('gyro_noise_density', 'gyro_noise_density'),
                           ('accel_bias_rw_density', 'accel_bias_rw'),
                           ('gyro_bias_rw_density', 'gyro_bias_rw')):
        e.setdefault(target, imu[source])
    e.setdefault('preflight_baro_sigma_m', nominal['barometer']['sigma'])
    scalar_errors = ('sigma', 'pos_sigma', 'vel_sigma', 'accel_noise_density',
                     'gyro_noise_density', 'accel_bias_rw', 'gyro_bias_rw',
                     'latency_s', 'dropout_prob', 'bias_rate_m_s')
    for sensor in ('imu', 'gnss', 'barometer', 'magnetometer', 'rpm'):
        spec = result[sensor]
        for key in scalar_errors:
            if key in spec:
                spec[key] = 0.
        for key in ('bias', 'accel_bias', 'gyro_bias', 'pos_bias', 'vel_bias'):
            if key in spec:
                spec[key] = [0.]*len(spec[key]) if isinstance(spec[key], list) else 0.
        for key in ('outage_windows', 'outlier_windows', 'gyro_vibration'):
            if key in spec:
                spec[key] = []
    # Near-instant observation filter; no extra command/plant truth access.
    result['rotor_observer'] = dict(source='telemetry', tau_s=1e-6)
    restored = {'quiet_sampled': (), 'imu_only': ('imu',),
                'navigation_only': ('gnss', 'barometer', 'magnetometer'),
                'rotor_only': ('rpm', 'rotor_observer')}[variant]
    for sensor in restored:
        result[sensor] = deepcopy(nominal[sensor])
    return load_sensor_profile(result)


def variant_config(base, controller, variant, fault='nominal'):
    if controller not in ('V13', 'F13') or variant not in VARIANTS or fault not in FAULTS:
        raise ValueError('unknown screen controller, variant, or fault')
    result = deepcopy(base)
    profile = resolve_feedback(result).profile
    if profile is None:
        raise ValueError('matching screen requires an inline sensor feedback profile')
    if variant in GROUP_VARIANTS:
        profile = sensor_group_profile(profile, variant)
    if variant in ('startup_guard', 'guarded_fast', 'guarded_fast25'):
        result['controllers'][controller]['rotor_startup_guard'] = True
    if variant == 'gyro_quiet':
        profile['imu']['gyro_noise_density'] = 0.
    if variant in ('rotor_tau2ms', 'guarded_fast', 'guarded_fast25'):
        profile['rotor_observer']['tau_s'] = .002
    if variant in ('cutoff25', 'guarded_fast25'):
        result['controllers'][controller]['indi_cutoff_hz'] = 25.
    if variant == 'aligned50':
        result['controllers'][controller].update(time_align='S1', indi_cutoff_hz=50.)
    if fault == 'baro_drift':
        profile['barometer']['bias_rate_m_s'] = .05
    elif fault == 'gnss_outlier':
        profile['gnss']['outlier_windows'] = [dict(start_s=5., end_s=5.2, pos_offset_m=[0., 0., 20.])]
    elif fault == 'baro_outlier':
        profile['barometer']['outlier_windows'] = [dict(start_s=5., end_s=5.1, height_offset_m=5.)]
    elif fault == 'gnss_outage':
        profile['gnss']['outage_windows'] = [[5., 6.]]
    elif fault == 'gyro_vibration':
        profile['imu']['gyro_vibration'] = [dict(frequency_hz=80., amplitude_rad_s=[.03]*3,
                                                  phase_rad=[0., 1., 2.])]
    elif fault == 'initial_error':
        profile['estimator'].update(initial_position_error_m=[.2, -.1, .3],
            initial_velocity_error_m_s=[.1, -.1, .05], initial_attitude_error_deg=[.5, -.5, 1.])
    if variant != 'baseline' or fault != 'nominal':
        profile['name'] = f'{variant}_{fault}'
    result['sensor_feedback']['profile'] = load_sensor_profile(profile)
    return validate_config(result)


def write_atomic(path, data):
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    tmp.replace(path)


def run(config, output, controllers, seeds, variants, faults, cases, timeout_s=600., runner=run_campaign):
    base = load_config(config)
    for seed in seeds:
        nonnegative_seed(seed)
        if any(lo <= seed <= hi for lo, hi in base['seeds'].values()):
            raise ValueError('development screen cannot consume reserved tuning/main seeds')
    if any(len(items) != len(set(items)) or not items for items in (controllers, seeds, variants, faults, cases)):
        raise ValueError('screen axes must be nonempty and have no duplicates')
    available = {s['id'] for s in base['scenarios']}
    if set(cases) - available:
        raise ValueError('screen case is not in the base config')
    if not 0 < timeout_s < float('inf'):
        raise ValueError('timeout must be finite and positive')
    output = Path(output).resolve()
    contract = dict(schema='sensor_matching_screen/1', stage='DEVELOPMENT',
        base_config=base, base_config_sha256=config_sha256(base), controllers=controllers,
        seeds=seeds, variants=variants, faults=faults, cases=cases, timeout_s=timeout_s,
        runtime_source_sha256=runtime_source_hashes())
    path = output/'experiment.json'
    records = {}
    if path.exists():
        previous = json.loads(path.read_text(encoding='utf-8'))
        if previous['contract'] != contract:
            raise ValueError('screen contract changed; use a new output directory')
        records = {r['trial_id']: r for r in previous['records']}
    elif output.exists() and any(output.iterdir()):
        raise ValueError('nonempty output has no screen manifest')
    output.mkdir(parents=True, exist_ok=True)
    def save(complete=False):
        doc = dict(contract=contract, records=list(records.values()), complete=complete,
                   expected_trials=len(controllers)*len(seeds)*len(variants)*len(faults)*len(cases))
        write_atomic(path, doc)
        return doc
    save()
    for variant in variants:
        for fault in faults:
            for controller in controllers:
                derived = variant_config(base, controller, variant, fault)
                folder = output/'inputs'/variant/fault/controller
                folder.mkdir(parents=True, exist_ok=True)
                cfg, profile = folder/'arena.json', folder/'sensors.json'
                write_atomic(cfg, derived)
                write_atomic(profile, derived['sensor_feedback']['profile'])
                for seed in seeds:
                    for case in cases:
                        trial_id = f'{variant}/{fault}/{controller}/{case}/seed_{seed}'
                        if trial_id in records:
                            continue
                        print(f'START {trial_id}', flush=True)
                        result = runner(cfg, profile, output/'runs'/trial_id,
                            cases=[case], controllers=[controller], seed=seed, timeout_s=timeout_s)
                        if len(result['records']) != 1:
                            raise ValueError(f'{trial_id}: expected exactly one recorded trial')
                        row = dict(result['records'][0], trial_id=trial_id, variant=variant, fault=fault)
                        records[trial_id] = row
                        save()
                        print(f"DONE tracking={row.get('tracking_pass')} domain={row.get('model_domain_valid')} "
                              f"duration={row.get('simulated_seconds')} reasons={row.get('failure_reasons')}", flush=True)
    return save(True)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'configs/arena_sensor_candidate_v6.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--controllers', nargs='+', choices=['V13', 'F13'], default=['V13', 'F13'])
    p.add_argument('--seeds', nargs='+', type=int, default=[3])
    p.add_argument('--variants', nargs='+', choices=VARIANTS,
                   default=['baseline', 'startup_guard', 'gyro_quiet', 'rotor_tau2ms'])
    p.add_argument('--faults', nargs='+', choices=FAULTS, default=['nominal'])
    p.add_argument('--cases', nargs='+', default=['gust_lateral_p10_VH'])
    p.add_argument('--timeout-s', type=float, default=600.)
    a = p.parse_args(argv)
    run(a.config, a.output, a.controllers, a.seeds, a.variants, a.faults, a.cases, a.timeout_s)


if __name__ == '__main__':
    main()
