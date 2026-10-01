"""Compare the clean-tree V13 evaluation-0 record with the earlier uncommitted-tree record.

The earlier record (results/arena/tuning/env_check_tune7_dirty_8bc6b4e/) was made before the D5 commit
with git_dirty True; it is kept for comparison only. Verdicts must match exactly; numbers use the
arena_tune_repro rule |a-b| <= rtol*max(|a|,|b|) + 1e-9 (rtol 1e-3); per-scenario trajectory hashes are
compared bit for bit.

python results/reproduction_tune7_2026-10-01/compare_env_check_records.py
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CLEAN = ROOT/'results/arena/tuning/env_check_tune7'
DIRTY = ROOT/'results/arena/tuning/env_check_tune7_dirty_8bc6b4e'
VERDICTS = ('failed', 'stop_reason', 'paper_reasons', 'sensor_seed', 'sensor_profile_sha256')
NUMBERS = ('score', 'window_rmse_velocity', 'window_rmse_z', 'max_omega')


def close(a, b, rtol=1e-3):
    return a == b or (a is not None and b is not None and abs(a-b) <= rtol*max(abs(a), abs(b))+1e-9)


def main():
    rec = {name: json.loads((d/'V13.record.json').read_text(encoding='utf-8')) for name, d in (('clean', CLEAN), ('dirty', DIRTY))}
    log = {name: json.loads((d/'V13.jsonl').read_text(encoding='utf-8').splitlines()[0])
           for name, d in (('clean', CLEAN), ('dirty', DIRTY))}
    differing_fields = sorted(k for k in set(rec['clean']) | set(rec['dirty'])
                              if rec['clean'].get(k) != rec['dirty'].get(k))
    scenarios = []
    for a, b in zip(log['clean']['scenarios'], log['dirty']['scenarios']):
        assert a['id'] == b['id']
        scenarios.append(dict(id=a['id'],
                              verdicts_equal=all(a.get(k) == b.get(k) for k in VERDICTS),
                              numbers_within_rtol=all(close(a.get(k), b.get(k)) for k in NUMBERS),
                              trajectory_bit_identical=a['trajectory_sha256'] == b['trajectory_sha256']))
    result = dict(
        clean=dict(git_revision=rec['clean']['git_revision'], git_dirty=rec['clean']['git_dirty'],
                   objective=log['clean']['objective'], config_sha256=rec['clean']['config_sha256']),
        dirty=dict(git_revision=rec['dirty']['git_revision'], git_dirty=rec['dirty']['git_dirty'],
                   objective=log['dirty']['objective'], config_sha256=rec['dirty']['config_sha256']),
        objective_bit_identical=log['clean']['objective'] == log['dirty']['objective'],
        sensor_runtime_source_equal=rec['clean'].get('sensor_runtime_source_sha256') ==
            rec['dirty'].get('sensor_runtime_source_sha256'),
        record_fields_differing=differing_fields,
        scenarios=len(scenarios),
        all_verdicts_equal=all(s['verdicts_equal'] for s in scenarios),
        all_numbers_within_rtol=all(s['numbers_within_rtol'] for s in scenarios),
        trajectories_bit_identical=sum(s['trajectory_bit_identical'] for s in scenarios),
        per_scenario=scenarios)
    (Path(__file__).resolve().parent/'compare_env_check_records.json').write_text(
        json.dumps(result, indent=1, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'per_scenario'}, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
