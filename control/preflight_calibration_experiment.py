"""Compare barometer startup calibration with paired in-flight sensor noise."""
import argparse
from copy import deepcopy
import json
from pathlib import Path

from control.navigation_grid_campaign import navigation_profile
from control.sensor_campaign import ROOT, run_campaign


VARIANTS = {
    'baseline': {},
    'preflight1': {'preflight_baro_samples': 1, 'preflight_baro_rng': 'independent'},
    'preflight25': {'preflight_baro_samples': 25, 'preflight_baro_rng': 'independent'},
    'preflight100': {'preflight_baro_samples': 100, 'preflight_baro_rng': 'independent'},
    'preflight100_frozen': {'preflight_baro_samples': 100, 'preflight_baro_rng': 'independent',
                            'estimate_baro_bias': False},
}


def calibration_profile(base, variant):
    out = deepcopy(base)
    out['estimator'].update(VARIANTS[variant])
    out['estimator']['trace_updates'] = True
    if variant != 'baseline':
        out['name'] += '_'+variant
    return out


def run_experiment(output, controllers, seeds, variants, case, timeout_s):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    base = navigation_profile(ROOT/'configs/sensors/nominal.json', ROOT/'configs/sensors/stress.json', 0., 0.)
    result = dict(case=case, controllers=controllers, seeds=seeds, variants=variants,
                  changes=VARIANTS, covariance_policy='unchanged; calibration uncertainty is not in P',
                  flight_noise_policy='same five flight streams; separate sixth preflight stream',
                  records=[], complete=False)
    for variant in variants:
        path = output/f'{variant}.json'
        path.write_text(json.dumps(calibration_profile(base, variant), indent=2)+'\n')
        for controller in controllers:
            for seed in seeds:
                print(f'START {controller} / {variant} / seed {seed}', flush=True)
                summary = run_campaign(ROOT/'configs/arena.json', path,
                    output/'runs'/variant/controller/f'seed_{seed}', cases=[case],
                    controllers=[controller], seed=seed, timeout_s=timeout_s)
                for record in summary['records']:
                    result['records'].append(dict(record, variant=variant))
                    print(f"DONE {controller} / {variant} / seed {seed}: "
                          f"track={record.get('tracking_pass')} domain={record.get('model_domain_valid')} "
                          f"outside={record.get('prop_domain_outside_fraction')} "
                          f"reasons={record.get('failure_reasons')}", flush=True)
                (output/'experiment.json').write_text(json.dumps(result, indent=2)+'\n')
    result['complete'] = True
    (output/'experiment.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--controllers', nargs='+', choices=['V13', 'F13'], default=['V13', 'F13'])
    parser.add_argument('--sensor-seeds', type=int, nargs='+', default=[1, 2])
    parser.add_argument('--variants', nargs='+', choices=list(VARIANTS), default=list(VARIANTS))
    parser.add_argument('--case', default='gust_lateral_p10_VL')
    parser.add_argument('--timeout-s', type=float, default=240.)
    args = parser.parse_args(argv)
    run_experiment(args.output, args.controllers, args.sensor_seeds, args.variants, args.case, args.timeout_s)


if __name__ == '__main__':
    main()
