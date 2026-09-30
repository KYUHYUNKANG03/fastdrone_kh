"""Replay a recorded sensor DEVELOPMENT case across machines.

PASS means agreement with the reference, including its failures. It does not
mean the controller passed tracking or that final tuning may begin.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.arena import config_sha256, load_config
from control.sensor_binding import runtime_source_hashes
from control.sensor_campaign import run_campaign

VERDICTS = ('controller', 'scenario_id', 'feedback', 'sensor_seed',
            'sensor_profile_sha256', 'tracking_pass', 'model_domain_valid',
            'passed', 'failure_reasons', 'stop_reason', 'paper_failed', 'paper_reasons',
            'campaign_timed_out', 'optimizer_failures', 'optimizer_calls')
NUMERICS = ('simulated_seconds', 'rmse_z', 'rmse_velocity', 'max_omega',
            'prop_domain_outside_fraction', 'window_rmse_z', 'window_rmse_velocity',
            'command_total_variation')


def canonical_source_hashes(hashes):
    """Normalize path separators at the portable artifact boundary, not bytes.

    Runtime manifests use native Path string keys. Windows backslashes must
    compare with the Mac reference's slashes without weakening any file hash.
    """
    normalized = {}
    for name, digest in hashes.items():
        name = name.replace('\\', '/')
        if name in normalized:
            raise ValueError('duplicate normalized source path')
        normalized[name] = digest
    return normalized


def compare(reference, actual, rtol=1e-3):
    import math
    problems = []
    for key in VERDICTS:
        if reference.get(key) != actual.get(key):
            problems.append(f'{key}: {reference.get(key)!r} -> {actual.get(key)!r}')
    for key in NUMERICS:
        a, b = reference.get(key), actual.get(key)
        if a is None or b is None:
            good = a is None and b is None
        else:
            good = math.isfinite(a) and math.isfinite(b) and abs(a-b) <= rtol*max(abs(a), abs(b))+1e-9
        if not good:
            problems.append(f'{key}: {a!r} -> {b!r}')
    return problems


def reproduce(reference, output):
    reference = json.loads(Path(reference).read_text(encoding='utf-8'))
    if reference.get('schema') != 'sensor_reproduction/1' or reference.get('stage') != 'DEVELOPMENT':
        raise ValueError('reference must be a recorded DEVELOPMENT reproduction case')
    if canonical_source_hashes(reference['runtime_source_sha256']) != canonical_source_hashes(runtime_source_hashes()):
        raise ValueError('runtime source differs from the reference')
    config = ROOT/reference['config']
    if config_sha256(load_config(config)) != reference['config_sha256']:
        raise ValueError('configuration differs from the reference')
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError('reproduction output must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    profile = output/'sensors.json'
    profile.write_text(json.dumps(load_config(config)['sensor_feedback']['profile']), encoding='utf-8')
    ref = reference['record']
    trial = run_campaign(config, profile, output/'run', cases=[ref['scenario_id']],
                         controllers=[ref['controller']], seed=ref['sensor_seed'], timeout_s=900.)
    if len(trial['records']) != 1:
        raise ValueError('expected one reproduction trial')
    actual = trial['records'][0]
    problems = compare(ref, actual)
    result = dict(stage='DEVELOPMENT', reproduction_pass=not problems, problems=problems,
                  rtol=1e-3, atol=1e-9, trajectory_bit_identical=(actual.get('trajectory_sha256') ==
                  ref.get('trajectory_sha256')), reference=ref, actual=actual,
                  note='Reproduction agreement is separate from flight performance and tuning readiness.')
    (output/'comparison.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference', type=Path, default=ROOT/'scripts/data/sensor_reproduction_v9.json')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(argv)
    result = reproduce(a.reference, a.output)
    print(json.dumps({k: v for k, v in result.items() if k not in ('reference', 'actual')}, indent=2))
    return 0 if result['reproduction_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
