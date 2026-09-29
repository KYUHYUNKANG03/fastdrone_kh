"""Separate rotor telemetry latency from measurement smoothing on fixed gains.

All other sensor/estimator settings stay at the candidate configuration. These
are diagnostic configurations, not selected hardware or final tuning profiles.
Each condition uses the strict matching-screen recorder and seed guards.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.arena import load_config, config_sha256
from control.sensor_matching_screen import run, write_atomic
from control.sensor_binding import runtime_source_hashes

CONDITIONS = ('rotor_unfiltered', 'rotor_no_latency', 'rotor_direct')


def condition_config(base, condition):
    if condition not in CONDITIONS:
        raise ValueError('unknown rotor timing condition')
    result = deepcopy(base)
    binding = result.get('sensor_feedback', {})
    if binding.get('mode') != 'sensors' or 'profile' not in binding:
        raise ValueError('rotor screen requires an inline sensor binding')
    profile = binding['profile']
    if profile['rotor_observer']['source'] != 'telemetry' or not profile['rpm']['enabled']:
        raise ValueError('rotor screen requires enabled rotor telemetry')
    if condition in ('rotor_unfiltered', 'rotor_direct'):
        profile['rotor_observer']['tau_s'] = 1e-6
    if condition in ('rotor_no_latency', 'rotor_direct'):
        profile['rpm']['latency_s'] = 0.
    profile['name'] = condition
    return result


def run_screen(config, output, controllers, seeds, conditions, cases, timeout_s=900.):
    base, output = load_config(config), Path(output).resolve()
    if not conditions or len(set(conditions)) != len(conditions):
        raise ValueError('conditions must be nonempty and unique')
    derived = {name: condition_config(base, name) for name in conditions}
    contract = dict(schema='sensor_rotor_screen/1', stage='DEVELOPMENT',
        base_config_sha256=config_sha256(base), controllers=controllers, seeds=seeds,
        conditions=conditions, cases=cases, timeout_s=timeout_s,
        runtime_source_sha256=runtime_source_hashes(),
        driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        configs=derived)
    manifest = output/'rotor_screen.json'
    if manifest.exists():
        if json.loads(manifest.read_text(encoding='utf-8')) != contract:
            raise ValueError('rotor screen contract changed; use a new directory')
    elif output.exists() and any(output.iterdir()):
        raise ValueError('nonempty rotor screen output has no contract')
    output.mkdir(parents=True, exist_ok=True)
    write_atomic(manifest, contract)
    for name, settings in derived.items():
        config_path = output/f'{name}.json'
        write_atomic(config_path, settings)
        run(config_path, output/name, controllers, seeds, ['baseline'], ['nominal'], cases, timeout_s)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'configs/arena_sensor_candidate_v6.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--controllers', nargs='+', choices=['V13', 'F13'], default=['V13', 'F13'])
    p.add_argument('--seeds', nargs='+', type=int, default=[3])
    p.add_argument('--conditions', nargs='+', choices=CONDITIONS, default=list(CONDITIONS))
    p.add_argument('--cases', nargs='+', default=['gust_lateral_p10_VH'])
    p.add_argument('--timeout-s', type=float, default=900.)
    a = p.parse_args()
    run_screen(a.config, a.output, a.controllers, a.seeds, a.conditions, a.cases, a.timeout_s)


if __name__ == '__main__':
    main()
