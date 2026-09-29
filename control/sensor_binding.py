"""One sensor contract for development, tuning, and distributed experiments.

The profile is embedded in the arena config, so the existing tuning/config hash
also covers sensor and estimator settings. Old configs still select truth.
"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

from control.arena_sensors import load_sensor_profile


def nonnegative_seed(value):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError('sensor seed must be a nonnegative integer')
    return value


@dataclass(frozen=True)
class FeedbackBinding:
    mode: str
    profile: dict | None
    seed: int

    @property
    def metadata(self):
        if self.mode == 'truth':
            return dict(feedback='truth')
        encoded = json.dumps(self.profile, sort_keys=True, separators=(',', ':'),
                             ensure_ascii=False, allow_nan=False).encode('utf-8')
        return dict(feedback='sensors', sensor_seed=self.seed,
                    sensor_profile=self.profile['name'],
                    sensor_profile_sha256=hashlib.sha256(encoded).hexdigest())


def resolve_feedback(config=None, *, mode=None, profile=None, seed=None):
    block = (config or {}).get('sensor_feedback', {})
    if not isinstance(block, dict):
        raise ValueError('sensor_feedback must be an object')
    if block:
        if block.get('schema') != 'sensor_feedback/1':
            raise ValueError("sensor_feedback requires schema 'sensor_feedback/1'")
        unknown = set(block) - {'schema', 'mode', 'profile', 'seed'}
        if unknown:
            raise ValueError(f'unknown sensor_feedback fields: {sorted(unknown)}')
        if block.get('mode') not in ('truth', 'sensors'):
            raise ValueError('sensor_feedback.mode must be truth or sensors')
        if 'seed' in block:
            nonnegative_seed(block['seed'])
        if block['mode'] == 'sensors' and not isinstance(block.get('profile'), dict):
            raise ValueError('sensor_feedback.profile must be an inline sensor/1 object')
    mode = block.get('mode', 'truth') if mode is None else mode
    seed = nonnegative_seed(block.get('seed', 0) if seed is None else seed)
    if mode not in ('truth', 'sensors'):
        raise ValueError(f'unknown feedback mode {mode!r}')
    if mode == 'truth':
        if profile is not None:
            raise ValueError('sensor profile supplied with truth feedback; select sensors explicitly')
        return FeedbackBinding(mode, None, seed)
    profile = block.get('profile') if profile is None else profile
    if profile is None:
        raise ValueError('sensor feedback requires a sensor profile')
    return FeedbackBinding(mode, load_sensor_profile(deepcopy(profile)), seed)


def main_sensor_seeds(spec, base):
    """Require a declared held-out seed set; never reuse the tuning binding seed."""
    binding = resolve_feedback(base)
    seeds = spec.get('sensor_seeds')
    if binding.mode == 'truth':
        if seeds is not None:
            raise ValueError('sensor_seeds requires a sensor-feedback base config')
        return [None]
    if not isinstance(seeds, list) or not seeds:
        raise ValueError('sensor main experiments require a nonempty sensor_seeds list')
    for seed in seeds:
        nonnegative_seed(seed)
    if len(seeds) != len(set(seeds)):
        raise ValueError('sensor_seeds contains duplicates')
    main_lo, main_hi = base['seeds']['main']
    tune_lo, tune_hi = base['seeds']['tuning']
    if any(not main_lo <= seed <= main_hi or tune_lo <= seed <= tune_hi
           or seed == binding.seed for seed in seeds):
        raise ValueError('sensor_seeds must be in the main range and disjoint from tuning')
    return list(seeds)


def runtime_source_hashes():
    """Pin runtime Python separately from final specs filled after tuning ends."""
    root = Path(__file__).resolve().parents[1]
    files = [p for folder in ('control', 'models/team_light/control')
             for p in (root/folder).glob('*.py') if not p.name.startswith('test_')]
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}


def sensor_record_problems(record, config):
    binding = resolve_feedback(config)
    if binding.mode == 'truth':
        return []
    problems = [f'{key} differs from sensor binding' for key, value in binding.metadata.items()
                if record.get(key) != value]
    if record.get('sensor_runtime_source_sha256') != runtime_source_hashes():
        problems.append('sensor runtime source hashes differ or are missing')
    if record.get('seeds', {}).get('used') != [binding.seed]:
        problems.append('used sensor tuning seeds differ')
    return problems
