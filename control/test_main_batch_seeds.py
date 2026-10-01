"""본 실험 묶음별 센서 시드 — ladder_sensor_seeds(로드맵 7번, 튜닝 전 코드).

비교 묶음(reference·table7_base·table7·gust·mission)은 서로 짝이라 같은 목록(sensor_seeds)을 쓰고,
사다리 묶음(ladder:*)만 그 부분집합을 쓸 수 있다. 칸이 없으면 지금과 같다.
실제 본 실험 설정의 실제 묶음 구성을 쓰고, 무거운 시나리오 생성(가용 가속도 계산)만 가짜 하나로 바꾼다.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from control import main_distributed as md
from control import main_experiment as me
from control.arena import load_config
from control.sensor_binding import (main_sensor_seeds, batch_sensor_seeds, COMPARISON_BATCHES,
                                    LADDER_PREFIX)

SENSOR_SPEC = Path('configs/main_experiment_sensor_candidate_v6.json')
TRUTH_SPEC = Path('configs/main_experiment.json')
ALL = [1000, 1001, 1002, 1003]
LADDER = [1000, 1002]


def sensor_spec(ladder=None, seeds=ALL):
    spec = me.load_spec(SENSOR_SPEC)
    spec['sensor_seeds'] = list(seeds)
    if ladder is not None:
        spec['ladder_sensor_seeds'] = list(ladder)
    return spec


def light_campaign(spec, monkeypatch):
    """실제 묶음(build_batches, 0.2 s)에 가짜 시나리오 하나씩 — 시드 전개만 본다."""
    from control.validation_suite import baseline_params
    camp = object.__new__(md.Campaign)
    camp.spec = spec
    camp.base = load_config(spec['base_config']['path'])
    camp.sensor_seeds = main_sensor_seeds(spec, camp.base)
    camp.native = baseline_params()
    camp.batches = {b.name: b for b in me.build_batches(spec, camp.base, camp.native)}
    camp.scenarios = lambda name: {f'{name}_s': SimpleNamespace(id=f'{name}_s', profile=SimpleNamespace(T_total=1.0))}
    camp._trim = {}
    monkeypatch.setattr(md, 'excluded_from', lambda *args: None)
    return camp


def seeds_by_batch(trials):
    out = {}
    for t in trials:
        out.setdefault(t['batch'], set()).add(t.get('sensor_seed'))
    return out


# ── 분류: 모든 묶음은 비교 또는 사다리 ────────────────────────────

@pytest.mark.parametrize('spec_path', [SENSOR_SPEC, TRUTH_SPEC])
def test_every_real_batch_is_comparison_or_ladder(spec_path):
    from control.validation_suite import baseline_params
    spec = me.load_spec(spec_path)
    base = load_config(spec['base_config']['path'])
    seeds = main_sensor_seeds(spec, base)
    names = [b.name for b in me.build_batches(spec, base, baseline_params())]
    for name in names:
        assert (name in COMPARISON_BATCHES) != name.startswith(LADDER_PREFIX)
        batch_sensor_seeds(spec, seeds, name)                   # 오류 없이 분류된다
    # 묶음 종류 목록(main_experiment.BATCHES)과 분류표가 어긋나지 않는다
    assert set(me.BATCHES) - {'ladder'} == set(COMPARISON_BATCHES)
    assert any(n.startswith(LADDER_PREFIX) for n in names)


@pytest.mark.parametrize('name', ['ladder', 'table8', 'Reference', ''])
def test_unknown_batch_is_an_error(name):
    with pytest.raises(ValueError, match='neither comparison nor ladder'):
        batch_sensor_seeds(sensor_spec(LADDER), ALL, name)


# ── 전개 ─────────────────────────────────────────────────────────

def test_without_the_field_every_batch_uses_the_full_list(monkeypatch):
    trials = light_campaign(sensor_spec(), monkeypatch).trials()
    assert all(seeds == set(ALL) for seeds in seeds_by_batch(trials).values())


def test_ladder_batches_use_exactly_the_subset(monkeypatch):
    by_batch = seeds_by_batch(light_campaign(sensor_spec(LADDER), monkeypatch).trials())
    ladder = [b for b in by_batch if b.startswith(LADDER_PREFIX)]
    assert len(ladder) == 4                                      # 사다리 변형 넷이 모두 같은 목록
    for batch, seeds in by_batch.items():
        assert seeds == (set(LADDER) if batch in ladder else set(ALL))


def test_shrinking_the_ladder_list_reduces_only_ladder_trials(monkeypatch):
    # 반대 방향: 칸이 실제로 쓰이지 않으면 위 시험이 우연히 통과할 수 있다
    def counts(spec):
        out = {}
        for t in light_campaign(spec, monkeypatch).trials():
            out[t['batch']] = out.get(t['batch'], 0) + 1
        return out
    full, small = counts(sensor_spec()), counts(sensor_spec([1001]))
    for batch in full:
        if batch.startswith(LADDER_PREFIX):
            assert small[batch] == full[batch] // len(ALL)
        else:
            assert small[batch] == full[batch]


# ── 검사 ─────────────────────────────────────────────────────────

@pytest.mark.parametrize('ladder,message', [
    ([1004], 'subset'), ([1000, 1000], 'duplicates'), ([], 'nonempty'),
    ([1000.5], 'seed'), (['1000'], 'seed'), ([True], 'seed'), (1000, 'nonempty list'),
])
def test_invalid_ladder_list_is_rejected(ladder, message):
    spec = sensor_spec()
    spec['ladder_sensor_seeds'] = ladder
    with pytest.raises(ValueError, match=message):
        main_sensor_seeds(spec, load_config(spec['base_config']['path']))
    assert any(message in p for p in me.guard_problems(spec))     # 실행 가드도 같은 검사를 거친다


def test_ladder_list_on_a_truth_base_is_rejected():
    spec = me.load_spec(TRUTH_SPEC)
    spec['ladder_sensor_seeds'] = [1000]
    with pytest.raises(ValueError, match='sensor-feedback base'):
        main_sensor_seeds(spec, load_config(spec['base_config']['path']))


def test_run_trial_rejects_a_comparison_only_seed_on_a_ladder_trial(monkeypatch):
    camp = light_campaign(sensor_spec(LADDER), monkeypatch)

    def reached(*args):
        raise RuntimeError('passed the held-out seed check')
    monkeypatch.setattr(me, 'trim_status', reached)
    ladder = next(n for n in camp.batches if n.startswith(LADDER_PREFIX))
    with pytest.raises(ValueError, match='held-out'):
        camp.run_trial(dict(batch=ladder, scenario_id=f'{ladder}_s', controller='V13', sensor_seed=1001))
    with pytest.raises(RuntimeError, match='passed the held-out'):   # 같은 시드가 비교 묶음에서는 허용
        camp.run_trial(dict(batch='reference', scenario_id='reference_s', controller='V13', sensor_seed=1001))


def test_plan_counts_repeats_per_batch(monkeypatch):
    import control.arena_factory as af
    monkeypatch.setattr(af, 'ArenaFactory', lambda *args, **kwargs: SimpleNamespace(cp=None))
    monkeypatch.setattr(me, 'build_scenarios', lambda config, cp, native: [SimpleNamespace(
        id='s', type='gust', profile=SimpleNamespace(T_total=2.0), meta={})])
    monkeypatch.setattr(me, 'excluded_from', lambda *args: None)
    monkeypatch.setattr(me, 'guard_problems', lambda *args: [])
    spec = sensor_spec(LADDER)
    rows = {r['batch']: r for r in me.plan(spec, check_trims=False)['batches']}
    for name, row in rows.items():
        controllers = 1 if name.startswith(LADDER_PREFIX) else len(spec['controllers'])
        repeats = len(LADDER) if name.startswith(LADDER_PREFIX) else len(ALL)
        assert row['trials'] == controllers*repeats
        assert row['sim_seconds'] == 2.0*controllers*repeats


# ── 조각 만들기 → 실행 → 합치기 ─────────────────────────────────

def test_shards_run_and_merge_with_a_ladder_subset(tmp_path, monkeypatch):
    spec = sensor_spec(LADDER)
    spec.pop('moment_model', None)
    labels = ['V13', 'CPID']
    spec['controllers'] = labels
    run_dir = tmp_path/'synthetic_tuning_fixture'                # 해시 가드용 합성 기록 — 튜닝 근거 아님
    run_dir.mkdir()
    for label in labels:
        (run_dir/f'{label}.record.json').write_text(json.dumps(dict(controller=label)), encoding='utf-8')
    spec['tuned']['run_dir'] = str(run_dir)
    spec['tuned']['records'] = {label: {'sha256': hashlib.sha256(
        (run_dir/f'{label}.record.json').read_bytes()).hexdigest()} for label in labels}
    spec_path = tmp_path/'spec.json'
    spec_path.write_text(json.dumps(spec, ensure_ascii=False), encoding='utf-8')

    monkeypatch.setattr(md.Campaign, 'scenarios', lambda self, name: {f'{name}_s': SimpleNamespace(
        id=f'{name}_s', profile=SimpleNamespace(T_total=1.0))})
    monkeypatch.setattr(md, 'excluded_from', lambda *args: None)
    from control import arena_design_check
    monkeypatch.setattr(arena_design_check, 'tuned_overrides', lambda *args, **kwargs: ({}, {}))

    def fake(self, trial):
        assert trial['sensor_seed'] in self.seeds_for(trial['batch'])
        return dict(trial_id=trial['trial_id'], batch=trial['batch'], scenario_id=trial['scenario_id'],
                    controller=trial['controller'], variant=trial['variant'], **self.hashes_for(trial['controller']),
                    feedback='sensors', sensor_seed=trial['sensor_seed'],
                    sensor_profile_sha256=trial['sensor_profile_sha256'], skipped=False, skip_reason=None,
                    passed=True, paper_failed=False, paper_reasons=[], stop_reason=None, failure_reasons=[],
                    window_rmse_velocity=.1, window_rmse_z=.2, window_max_omega=1., max_omega=1.,
                    trajectory_sha256='x'), None
    monkeypatch.setattr(md.Campaign, 'run_trial', fake)

    doc = md.make_shards(spec_path, 2, tmp_path/'s.json')
    assert seeds_by_batch(doc['trials'].values())['ladder:V13-0'] == set(LADDER)
    assert seeds_by_batch(doc['trials'].values())['reference'] == set(ALL)
    for index in range(2):
        md.run_shard(tmp_path/'s.json', index, tmp_path/'out')
    report = md.merge(tmp_path/'s.json', [tmp_path/'out'], tmp_path/'merged', require_platform=None)
    assert report['problems'] == [] and report['found'] == report['expected'] == len(doc['trials'])
