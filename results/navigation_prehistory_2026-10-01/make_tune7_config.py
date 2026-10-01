"""Generate configs/arena_tune7.json from v9 by the rule in docs/TUNE7_CONFIG.md, then check it."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from control.arena import check_confirmed_facts, config_sha256, validate_config  # noqa: E402
from control.sensor_binding import resolve_feedback  # noqa: E402
from models.team_light.control.baseline_v2 import baseline_params  # noqa: E402

BASE = ROOT/'configs/arena_rotor_projected_development_v9.json'
TARGET = ROOT/'configs/arena_tune7.json'


def leaves(value, prefix=''):
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            out.update(leaves(item, f'{prefix}.{key}' if prefix else key))
        return out
    return {prefix: value}


base = json.loads(BASE.read_text(encoding='utf-8'))
config = deepcopy(base)
block = config['sensor_feedback']
assert block['seed'] == 3                                     # kept: development seed
ids = [s['id'] for s in config['tuning']['scenarios']]
block['tuning_seeds'] = {sid: 2001+i for i, sid in enumerate(ids)}   # order of tuning.scenarios
block['profile']['estimator']['navigation_prehistory_s'] = 30.0
validate_config(config)
problems = check_confirmed_facts(config, baseline_params())
assert not problems, problems
text = json.dumps(config, indent=2, ensure_ascii=False)+'\n'
TARGET.write_text(text, encoding='utf-8')

old, new = leaves(base), leaves(config)
changed = {k: (old.get(k, '<absent>'), new.get(k, '<absent>')) for k in sorted(set(old) | set(new))
           if old.get(k, '<absent>') != new.get(k, '<absent>')}
binding = resolve_feedback(config)
report = dict(base=str(BASE.relative_to(ROOT)), target=str(TARGET.relative_to(ROOT)),
              base_config_sha256=config_sha256(base), config_sha256=config_sha256(config),
              raw_file_sha256=hashlib.sha256(TARGET.read_bytes()).hexdigest(),
              sensor_profile_sha256=binding.metadata['sensor_profile_sha256'], development_seed=binding.seed,
              tuning_seeds=block['tuning_seeds'], changed_leaves=changed,
              validate_config='passed', check_confirmed_facts='passed')
(ROOT/'results/navigation_prehistory_2026-10-01/tune7_config_check.json').write_text(
    json.dumps(report, indent=1, ensure_ascii=False)+'\n', encoding='utf-8')
print(json.dumps({k: v for k, v in report.items() if k != 'tuning_seeds'}, indent=1, ensure_ascii=False))
