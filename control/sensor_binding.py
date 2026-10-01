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
from control.source_manifest import file_hashes


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
        unknown = set(block) - {'schema', 'mode', 'profile', 'seed', 'tuning_seeds'}
        if unknown:
            raise ValueError(f'unknown sensor_feedback fields: {sorted(unknown)}')
        if block.get('mode') not in ('truth', 'sensors'):
            raise ValueError('sensor_feedback.mode must be truth or sensors')
        if 'seed' in block:
            nonnegative_seed(block['seed'])
        if 'tuning_seeds' in block:
            _check_tuning_seed_shape(block)
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


def _check_tuning_seed_shape(block):
    """D4 (c): tuning_seeds = {tuning scenario id: seed}. Shape only here; the
    cross-check against tuning.scenarios lives in arena.validate_config."""
    seeds = block['tuning_seeds']
    if block.get('mode') != 'sensors':
        raise ValueError('sensor_feedback.tuning_seeds requires mode sensors')
    if not isinstance(seeds, dict) or not seeds:
        raise ValueError('sensor_feedback.tuning_seeds must be a nonempty object')
    for seed in seeds.values():
        nonnegative_seed(seed)
    if len(set(seeds.values())) != len(seeds):
        raise ValueError('sensor_feedback.tuning_seeds values must be distinct')
    if block.get('seed') in set(seeds.values()):
        # 잡음 수열은 시드만의 함수다 — 기본 seed(스모크·설계점검)가 튜닝 시드와 같으면 표본 내 평가가 된다.
        raise ValueError('sensor_feedback.seed must differ from every tuning seed')


def tuning_seed_map(config):
    """Per-scenario tuning seeds, or None when the config uses the single seed."""
    block = (config or {}).get('sensor_feedback', {})
    seeds = block.get('tuning_seeds') if isinstance(block, dict) else None
    return dict(seeds) if seeds else None


def tuning_seed_for(config, scenario_id):
    """Seed passed to run_trial for one tuning scenario. None = the binding seed (old path)."""
    seeds = tuning_seed_map(config)
    return None if seeds is None else seeds[scenario_id]


def expected_used_seeds(config, scenario_ids):
    """What a tuning record must list in seeds.used — written and checked through this one function."""
    seeds = tuning_seed_map(config)
    if seeds is None:
        return [resolve_feedback(config).seed]
    return [seeds[sid] for sid in scenario_ids]


def main_sensor_seeds(spec, base):
    """Require a declared held-out seed set; never reuse the tuning binding seed.

    Returns the comparison list. An optional ladder_sensor_seeds (a subset) is
    validated here too, so every caller of this guard also checks it."""
    binding = resolve_feedback(base)
    seeds = spec.get('sensor_seeds')
    if binding.mode == 'truth':
        if seeds is not None or spec.get('ladder_sensor_seeds') is not None:
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
    _ladder_seeds(spec, seeds)
    return list(seeds)


# 본 실험 묶음 분류 — 비교 묶음은 서로 짝이라(표7 섭동 ↔ table7_base, 사다리 V13 nominal ↔ reference의 V13)
# 같은 시드 목록을 쓰고, 사다리만 그 부분집합을 쓸 수 있다. 모르는 묶음은 오류(조용히 한쪽으로 넣지 않는다).
COMPARISON_BATCHES = ('reference', 'table7_base', 'table7', 'gust', 'mission')
LADDER_PREFIX = 'ladder:'


def _ladder_seeds(spec, seeds):
    ladder = spec.get('ladder_sensor_seeds')
    if ladder is None:
        return list(seeds)
    if not isinstance(ladder, list) or not ladder:
        raise ValueError('ladder_sensor_seeds must be a nonempty list')
    for seed in ladder:
        nonnegative_seed(seed)
    if len(ladder) != len(set(ladder)):
        raise ValueError('ladder_sensor_seeds contains duplicates')
    if not set(ladder) <= set(seeds):
        raise ValueError('ladder_sensor_seeds must be a subset of sensor_seeds (paired with the comparison batches)')
    return list(ladder)


def batch_sensor_seeds(spec, seeds, batch_name):
    """Seeds for one main-experiment batch. `seeds` is the main_sensor_seeds(spec, base) result."""
    if batch_name in COMPARISON_BATCHES:
        return list(seeds)
    if batch_name.startswith(LADDER_PREFIX):
        return list(seeds) if seeds == [None] else _ladder_seeds(spec, seeds)
    raise ValueError(f'unknown main-experiment batch {batch_name!r}: neither comparison nor ladder')


def runtime_source_hashes():
    """Pin runtime Python separately from final specs filled after tuning ends."""
    root = Path(__file__).resolve().parents[1]
    files = [p for folder in ('control', 'models/team_light/control')
             for p in (root/folder).glob('*.py') if not p.name.startswith('test_')]
    return file_hashes(files, root)


def sensor_record_problems(record, config):
    binding = resolve_feedback(config)
    if binding.mode == 'truth':
        return []
    problems = [f'{key} differs from sensor binding' for key, value in binding.metadata.items()
                if record.get(key) != value]
    if record.get('sensor_runtime_source_sha256') != runtime_source_hashes():
        problems.append('sensor runtime source hashes differ or are missing')
    seeds = record.get('seeds', {})
    if seeds.get('used') != expected_used_seeds(config, record.get('scenario_ids', [])):
        problems.append('used sensor tuning seeds differ')
    if seeds.get('by_scenario') != tuning_seed_map(config):
        problems.append('per-scenario sensor tuning seeds differ')
    return problems
