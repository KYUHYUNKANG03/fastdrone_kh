"""Receipt linking the new reproduction references and the V13 evaluation-0 record to their commit.

The reference files pin all runtime source hashes but have no commit field (same as the *_v13 files).
The commit and clean-tree flag live in each run's manifest.json, which also holds absolute local paths
and is not committed. This receipt keeps only the provenance fields, with repository-relative paths, in
the way results/source_portability_v13 commits small receipts and not raw run folders.

python results/reproduction_tune7_2026-10-01/provenance_summary.py
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
RUNS = {'record_projected': 'projected', 'record_legacy': 'legacy',
        'replay_projected': 'replay_projected', 'replay_legacy': 'replay_legacy',
        'old_v13_check_projected': 'old_v13_numeric_check/sensor_reproduction_v13',
        'old_v13_check_legacy': 'old_v13_numeric_check/sensor_legacy_reproduction_v13'}
FILES = ['scripts/data/sensor_reproduction_tune7.json', 'scripts/data/sensor_legacy_reproduction_tune7.json',
         'scripts/data/sensor_reproduction_v13.json', 'scripts/data/sensor_legacy_reproduction_v13.json',
         'scripts/sensor_reproduction_record.py', 'configs/arena_tune7.json',
         'results/arena/tuning/env_check_tune7/V13.record.json', 'results/arena/tuning/env_check_tune7/V13.jsonl']


def sha(path):
    return hashlib.sha256((ROOT/path).read_bytes()).hexdigest()


def main():
    runs = {}
    for role, folder in RUNS.items():
        manifest_path = next((HERE/folder).glob('run/runs/*/*/arena_*/manifest.json'))
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        trial = json.loads(next(manifest_path.parent.glob('trials.jsonl')).read_text(encoding='utf-8').splitlines()[0])
        entry = dict(run_dir=manifest_path.parent.relative_to(ROOT).as_posix(),
                     git_revision=manifest['git_revision'], git_dirty=manifest['git_dirty'],
                     environment=manifest['environment'], config_sha256=manifest['config_sha256'],
                     sensor_seed=manifest['sensor_seed'], sensor_profile_sha256=manifest.get('sensor_profile_sha256'),
                     scenario_id=trial['scenario_id'], controller=trial['controller'],
                     passed=trial['passed'], model_domain_valid=trial['model_domain_valid'],
                     trajectory_sha256=trial['trajectory_sha256'])
        comparison = HERE/folder/'comparison.json'
        if comparison.exists():
            c = json.loads(comparison.read_text(encoding='utf-8'))
            entry.update(reproduction_pass=c['reproduction_pass'], problems=c['problems'],
                         trajectory_bit_identical=c['trajectory_bit_identical'],
                         hash_gate='passed (sensor_reproduce refuses differing runtime sources)')
        runs[role] = entry
    record = json.loads((ROOT/'results/arena/tuning/env_check_tune7/V13.record.json').read_text(encoding='utf-8'))
    receipt = dict(
        purpose='Reproduction references *_tune7.json and V13 evaluation-0 record, made from a clean commit',
        file_sha256={path: sha(path) for path in FILES},
        runs=runs,
        env_check_tune7=dict(git_revision=record['git_revision'], git_dirty=record['git_dirty'],
                             config_sha256=record['config_sha256'], status=record['status'],
                             scenario_workers=record['scenario_workers'],
                             prior_objective=record['prior_objective']),
        checks=dict(old_v13_numeric_check='old_v13_numeric_check.json',
                    clean_vs_uncommitted_record='compare_env_check_records.json'))
    (HERE/'provenance.json').write_text(json.dumps(receipt, indent=1, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps({k: (v.get('git_revision'), v.get('git_dirty')) for k, v in runs.items()}, indent=1))
    print('env_check_tune7', record['git_revision'][:7], record['git_dirty'])


if __name__ == '__main__':
    main()
