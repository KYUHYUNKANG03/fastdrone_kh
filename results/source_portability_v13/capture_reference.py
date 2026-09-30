"""Record fresh Mac references after the declared source-manifest-only change.

This is an explicit source transition, not a replay under the archived runtime.
The old fixtures are preserved. Require both unchanged numeric verdicts and an
identical trajectory before writing each new source-bound fixture.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from control.arena import config_sha256, load_config
from control.sensor_binding import runtime_source_hashes
from control.sensor_campaign import run_campaign
from scripts.sensor_reproduce import compare


def capture(kind):
    stem = 'sensor_reproduction' if kind == 'projected' else 'sensor_legacy_reproduction'
    old_path = ROOT/'scripts/data'/f'{stem}_v9.json'
    old = json.loads(old_path.read_text(encoding='utf-8'))
    current = runtime_source_hashes()
    changed = sorted(k for k in current.keys() | old['runtime_source_sha256'].keys()
                     if current.get(k) != old['runtime_source_sha256'].get(k))
    assert changed == ['control/sensor_binding.py', 'control/source_manifest.py',
                       'control/validation_suite.py'], changed
    config = ROOT/old['config']
    base = load_config(config)
    assert config_sha256(base) == old['config_sha256']
    output = ROOT/'results/source_portability_v13'/kind
    output.mkdir(exist_ok=False)
    profile = output/'sensors.json'
    profile.write_text(json.dumps(base['sensor_feedback']['profile']), encoding='utf-8')
    ref = old['record']
    campaign = run_campaign(config, profile, output/'run', cases=[ref['scenario_id']],
                            controllers=[ref['controller']], seed=ref['sensor_seed'], timeout_s=900.)
    assert runtime_source_hashes() == current, 'runtime changed during capture'
    assert len(campaign['records']) == 1
    actual = campaign['records'][0]
    problems = compare(ref, actual)
    bit_identical = actual.get('trajectory_sha256') == ref.get('trajectory_sha256')
    receipt = dict(stage='DEVELOPMENT', source_transition=True,
                   old_reference=old_path.relative_to(ROOT).as_posix(),
                   old_reference_sha256=hashlib.sha256(old_path.read_bytes()).hexdigest(),
                   capture_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   changed_runtime_files=changed, runtime_source_sha256=current,
                   comparison_problems=problems, trajectory_bit_identical=bit_identical,
                   transition_verified=not problems and bit_identical,
                   note='Native Mac capture; no native Windows execution claimed.')
    (output/'comparison.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    if not receipt['transition_verified']:
        raise ValueError('source transition changed numerical behavior; preserve outcome for diagnosis')
    recorded = dict(actual)
    recorded['run_dir'] = Path(actual['run_dir']).relative_to(ROOT).as_posix()
    fresh = dict(schema=old['schema'], stage=old['stage'], config=old['config'],
                 config_sha256=old['config_sha256'],
                 source_screen=(output/'run/campaign.json').relative_to(ROOT).as_posix(),
                 environment=old['environment'], runtime_source_sha256=current, record=recorded)
    destination = ROOT/'scripts/data'/f'{stem}_v13.json'
    with destination.open('x', encoding='utf-8') as file:
        json.dump(fresh, file, indent=2, ensure_ascii=False, allow_nan=False)
        file.write('\n')
    print(json.dumps(dict(kind=kind, reference=destination.name,
                         transition_verified=True, trajectory_bit_identical=True)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('projected', 'legacy'))
    capture(parser.parse_args().kind)
