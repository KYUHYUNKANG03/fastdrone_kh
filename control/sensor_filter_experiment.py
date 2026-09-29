"""Bounded INDI cutoff/alignment and startup-availability experiments.

Sensor noise and estimator tuning remain fixed. Controller variants are written
to separate arena configs; the bundled arena config is never edited.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

from control.arena import validate_config
from control.navigation_grid_campaign import navigation_profile
from control.sensor_campaign import ROOT, run_campaign


VARIANTS = ('baseline', 'aligned50', 'aligned25', 'aligned12p5', 'startup_guard', 'guarded25')


def variant_config(base, controller, variant):
    if controller not in ('V13', 'F13') or variant not in VARIANTS:
        raise ValueError('unsupported controller or filter variant')
    result = deepcopy(base)
    spec = result['controllers'][controller]
    if variant.startswith('aligned') or variant == 'guarded25':
        cutoff = {'aligned50': 50., 'aligned25': 25., 'aligned12p5': 12.5, 'guarded25': 25.}[variant]
        spec.update(indi_cutoff_hz=cutoff, time_align='S1')
    if variant in ('startup_guard', 'guarded25'):
        spec['rotor_startup_guard'] = True
    return validate_config(result)


def run_experiment(output, controllers, seeds, variants, level, case, timeout_s):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    base = json.loads((ROOT/'configs/arena.json').read_text())
    profile = navigation_profile(ROOT/'configs/sensors/nominal.json',
                                 ROOT/'configs/sensors/stress.json', 0., level)
    profile_path = output/'sensor_profile.json'
    profile_path.write_text(json.dumps(profile, indent=2)+'\n')
    summary = dict(case=case, controllers=controllers, seeds=seeds, variants=variants,
                   gnss_level=level, records=[], complete=False)
    for variant in variants:
        for controller in controllers:
            # V13 already uses S1/50 Hz, so its baseline is this control.
            if variant == 'aligned50' and controller == 'V13':
                continue
            config = variant_config(base, controller, variant)
            path = output/f'{controller}_{variant}.json'
            path.write_text(json.dumps(config, indent=2)+'\n')
            for seed in seeds:
                print(f'START {variant} / {controller} / seed {seed}', flush=True)
                result = run_campaign(path, profile_path, output/'runs'/variant/controller/f'seed_{seed}',
                                      cases=[case], controllers=[controller], seed=seed, timeout_s=timeout_s)
                for r in result['records']:
                    summary['records'].append(dict(r, variant=variant))
                    print(f"DONE {variant} / {controller} / seed {seed}: "
                          f"track={r.get('tracking_pass')} domain={r.get('model_domain_valid')} "
                          f"outside={r.get('prop_domain_outside_fraction')} "
                          f"reasons={r.get('failure_reasons')}", flush=True)
                (output/'experiment.json').write_text(json.dumps(summary, indent=2)+'\n')
    summary['complete'] = True
    (output/'experiment.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--controllers', nargs='+', choices=['V13', 'F13'], default=['V13', 'F13'])
    parser.add_argument('--sensor-seeds', type=int, nargs='+', default=[1, 2])
    parser.add_argument('--variants', nargs='+', choices=VARIANTS,
                        default=['baseline', 'aligned50', 'aligned25', 'aligned12p5'])
    parser.add_argument('--gnss-level', type=float, default=.25)
    parser.add_argument('--case', default='gust_lateral_p10_VL')
    parser.add_argument('--timeout-s', type=float, default=240.)
    args = parser.parse_args(argv)
    run_experiment(args.output, args.controllers, args.sensor_seeds, args.variants,
                   args.gnss_level, args.case, args.timeout_s)


if __name__ == '__main__':
    main()
