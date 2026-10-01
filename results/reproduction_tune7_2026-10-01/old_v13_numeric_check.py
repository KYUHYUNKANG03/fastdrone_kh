"""Before re-recording: replay the two old *_v13 references with the new code and compare numerically.

Same steps as scripts/sensor_reproduce.reproduce (config hash check -> run_campaign -> compare), except
the runtime-source hash gate: the old references pin pre-D4 sources, so the differing files are listed
instead and must be exactly the D4/7 and D5 runtime changes. Also records whether the trajectory equals
the one a44c718 produced on this Mac (D4 session, 2026-10-01), and that compare() catches a 1 % change.

python results/reproduction_tune7_2026-10-01/old_v13_numeric_check.py
"""
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent/'old_v13_numeric_check'
sys.path.insert(0, str(ROOT))
from control.arena import config_sha256, load_config  # noqa: E402
from control.sensor_binding import runtime_source_hashes  # noqa: E402
from control.sensor_campaign import run_campaign  # noqa: E402
from scripts.sensor_reproduce import canonical_source_hashes, compare  # noqa: E402

CHANGED = {'control/arena.py', 'control/arena_tune.py', 'control/sensor_binding.py',          # D4 and 7
           'control/main_distributed.py', 'control/main_experiment.py',
           'control/arena_sensors.py', 'control/arena_feedback.py', 'control/arena_estimator.py',  # D5
           'control/arena_joint_estimator.py'}
# Trajectories a44c718 produced on this Mac for the two cases (D4 session numeric replay, 2026-10-01).
SAME_MAC_A44C718 = {
    'scripts/data/sensor_reproduction_v13.json':
        '34afba26b9dff911e39bf9eb32d027d4502098f394059e317f466b7de02a82b7',
    'scripts/data/sensor_legacy_reproduction_v13.json':
        '313d81f8eb99aaad67b74c785d6ef8bed10b232b9bd7624894297386ed2a39d4'}


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def main():
    OUT.mkdir(exist_ok=False)
    state = dict(head=git('rev-parse', 'HEAD'), tracked_changes=git('status', '--porcelain', '--untracked-files=no'))
    now = canonical_source_hashes(runtime_source_hashes())
    results = dict(code_state=state, references=[])
    for ref_path, expected in SAME_MAC_A44C718.items():
        reference = json.loads((ROOT/ref_path).read_text(encoding='utf-8'))
        stored = canonical_source_hashes(reference['runtime_source_sha256'])
        differing = sorted(k for k in set(now) | set(stored) if now.get(k) != stored.get(k))
        entry = dict(reference=ref_path, hash_differences=differing,
                     hash_differences_are_the_known_changes=set(differing) == CHANGED)
        results['references'].append(entry)
        if set(differing) != CHANGED:
            entry['stopped'] = 'hash differences are not exactly the known runtime changes'
            break
        config = ROOT/reference['config']
        if config_sha256(load_config(config)) != reference['config_sha256']:
            entry['stopped'] = 'configuration differs from the reference'
            break
        out = OUT/Path(ref_path).stem
        out.mkdir()
        profile = out/'sensors.json'
        profile.write_text(json.dumps(load_config(config)['sensor_feedback']['profile']), encoding='utf-8')
        ref = reference['record']
        trial = run_campaign(config, profile, out/'run', cases=[ref['scenario_id']],
                             controllers=[ref['controller']], seed=ref['sensor_seed'], timeout_s=900.)
        actual = trial['records'][0] if len(trial['records']) == 1 else None
        if actual is None:
            entry['stopped'] = f"expected one trial, got {len(trial['records'])}"
            break
        problems = compare(ref, actual)
        tampered = dict(actual, rmse_z=actual['rmse_z']*1.01) if actual.get('rmse_z') else None
        manifest = json.loads(next(Path(actual['run_dir']).glob('manifest.json')).read_text(encoding='utf-8'))
        entry.update(case=ref['scenario_id'], controller=ref['controller'], seed=ref['sensor_seed'],
                     numeric_problems=problems, numeric_match=not problems,
                     trajectory_equals_reference=actual.get('trajectory_sha256') == ref.get('trajectory_sha256'),
                     trajectory_equals_a44c718_same_mac=actual.get('trajectory_sha256') == expected,
                     actual_trajectory_sha256=actual.get('trajectory_sha256'),
                     compare_catches_1pct_change=bool(tampered and compare(ref, tampered)),
                     run_git_revision=manifest.get('git_revision'), run_git_dirty=manifest.get('git_dirty'))
    results['code_unchanged_during_run'] = git('rev-parse', 'HEAD') == state['head'] and \
        canonical_source_hashes(runtime_source_hashes()) == now
    (OUT.parent/'old_v13_numeric_check.json').write_text(json.dumps(results, indent=1, ensure_ascii=False)+'\n',
                                                          encoding='utf-8')
    print(json.dumps(results, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
