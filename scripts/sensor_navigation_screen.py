"""Isolate navigation measurement groups at fixed estimator covariance/gains."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.arena import load_config, config_sha256, validate_config
from control.sensor_matching_screen import run, sensor_group_profile, write_atomic
from control.sensor_binding import runtime_source_hashes

SENSORS = ('gnss', 'barometer', 'magnetometer')


def condition_config(base, sensor):
    if sensor not in SENSORS:
        raise ValueError('unknown navigation sensor')
    result = deepcopy(base)
    nominal = base['sensor_feedback']['profile']
    quiet = sensor_group_profile(nominal, 'quiet_sampled')
    quiet[sensor] = deepcopy(nominal[sensor])
    quiet['name'] = 'nav_startup_'+sensor+'_only'
    result['sensor_feedback']['profile'] = quiet
    return validate_config(result)


def run_screen(config, output, controllers, seeds, sensors, cases, timeout_s=900.):
    if not sensors or len(set(sensors)) != len(sensors):
        raise ValueError('sensors must be nonempty and unique')
    base, output = load_config(config), Path(output).resolve()
    configs = {sensor: condition_config(base, sensor) for sensor in sensors}
    contract = dict(schema='sensor_navigation_screen/1', stage='DEVELOPMENT',
        base_config_sha256=config_sha256(base), controllers=controllers, seeds=seeds,
        sensors=sensors, cases=cases, timeout_s=timeout_s, configs=configs,
        runtime_source_sha256=runtime_source_hashes(),
        driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    manifest = output/'navigation_screen.json'
    if manifest.exists():
        if json.loads(manifest.read_text(encoding='utf-8')) != contract:
            raise ValueError('navigation screen contract changed; use a new directory')
    elif output.exists() and any(output.iterdir()):
        raise ValueError('nonempty navigation output has no contract')
    output.mkdir(parents=True, exist_ok=True)
    write_atomic(manifest, contract)
    for sensor, settings in configs.items():
        path = output/f'{sensor}.json'
        write_atomic(path, settings)
        run(path, output/sensor, controllers, seeds, ['baseline'], ['nominal'], cases, timeout_s)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'configs/arena_rotor_projected_development_v9.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--controllers', nargs='+', choices=['V13', 'F13'], default=['V13'])
    p.add_argument('--seeds', nargs='+', type=int, default=[3])
    p.add_argument('--sensors', nargs='+', choices=SENSORS, default=list(SENSORS))
    p.add_argument('--cases', nargs='+', default=['gust_lateral_p10_VL'])
    p.add_argument('--timeout-s', type=float, default=900.)
    a = p.parse_args()
    run_screen(a.config, a.output, a.controllers, a.seeds, a.sensors, a.cases, a.timeout_s)


if __name__ == '__main__':
    main()
