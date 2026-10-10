"""tune8(2026-10-10) — 튜닝 집합 개정과 objective.acceptance_failure.

확인하는 것:
  - configs/arena_tune8.json이 검증을 통과하고, 튜닝 시드가 시나리오와 1:1이며 본시험 id와 겹치지 않는다
  - 고속 시나리오가 V_H 양쪽(83·85 m/s)에서 본시험과 같은 길이(12 s 이상)로 들어 있다
  - 그대로 둔 시나리오(저속 돌풍·계단)는 tune7과 내용·시드가 같다
  - acceptance_failure가 꺼져 있으면(기본, tune7 이하) 채점 항목이 예전과 같고,
    켜져 있으면 본시험 합격 기준 미달이 실패(벌점)로 잡힌다
시나리오는 0.12 s 돌풍 하나(CPID)라 가볍다.
"""
from copy import deepcopy

import pytest

from control.arena import load_config, validate_config, build_scenarios
from control.arena_sensors import load_sensor_profile


@pytest.fixture(scope='module')
def tune8():
    return load_config('configs/arena_tune8.json')


@pytest.fixture(scope='module')
def tune7():
    return load_config('configs/arena_tune7.json')


def test_tune8_config_is_valid_and_seeds_match(tune8):
    validate_config(tune8)
    ids = [s['id'] for s in tune8['tuning']['scenarios']]
    seeds = tune8['sensor_feedback']['tuning_seeds']
    assert set(seeds) == set(ids) and len(set(seeds.values())) == len(ids)
    lo, hi = tune8['seeds']['tuning']
    assert all(lo <= seed <= hi for seed in seeds.values())
    assert not set(ids) & {s['id'] for s in tune8['scenarios']}
    assert tune8['tuning']['budget'] == 60
    assert tune8['tuning']['objective']['acceptance_failure'] is True


def test_tune8_keeps_everything_but_the_tuning_block(tune8, tune7):
    for key in tune7:
        if key not in ('tuning', 'sensor_feedback'):
            assert tune8[key] == tune7[key], key
    for key in tune7['sensor_feedback']:
        if key != 'tuning_seeds':
            assert tune8['sensor_feedback'][key] == tune7['sensor_feedback'][key], key
    for key in ('search', 'parameters', 'label'):
        assert tune8['tuning'][key] == tune7['tuning'][key], key
    assert tune8['tuning']['objective']['metric'] == tune7['tuning']['objective']['metric']
    assert tune8['tuning']['objective']['failure_penalty'] == tune7['tuning']['objective']['failure_penalty']


def test_unchanged_scenarios_keep_content_and_seed(tune8, tune7):
    old = {s['id']: s for s in tune7['tuning']['scenarios']}
    old_seeds = tune7['sensor_feedback']['tuning_seeds']
    kept = [s for s in tune8['tuning']['scenarios'] if s['id'] in old]
    assert len(kept) == 12                      # 저속 돌풍 4 + 계단 8
    for s in kept:
        assert s == old[s['id']]
        assert tune8['sensor_feedback']['tuning_seeds'][s['id']] == old_seeds[s['id']]
    new_seeds = {seed for sid, seed in tune8['sensor_feedback']['tuning_seeds'].items() if sid not in old}
    assert not new_seeds & set(old_seeds.values())


def test_high_speed_scenarios_bracket_within_checked_range_and_last_long_enough(tune8):
    from control.arena import SPEED_RANGE
    from control.arena_factory import ArenaFactory
    factory = ArenaFactory(tune8)
    scenarios = build_scenarios(tune8, factory.cp, factory.p, scenarios=tune8['tuning']['scenarios'])
    v_h = float(tune8['speeds_m_s']['V_H'])
    fast = [s for s in scenarios if s.type in ('gust', 'cruise') and s.profile.cruise_speed >= 80.0]
    speeds = {float(s.profile.cruise_speed) for s in fast}
    assert speeds == {83.0, v_h} and max(speeds) <= SPEED_RANGE[1]
    assert all(s.profile.T_total >= 12.0 - 1e-9 for s in fast)
    assert sum(s.profile.cruise_speed == v_h for s in fast) >= 2
    # 섭동 2개(지연·질량)는 83 m/s 순항에 붙는다
    perturbed = [s for s in scenarios if s.cases[0].get('factors') or s.cases[0].get('extra_params')]
    assert sorted(s.id for s in perturbed) == ['tune4_cruise_V83__mass_1.15', 'tune4_cruise_V83__state_delay_5ms']
    # 기동의 꼬리는 본시험과 같은 5 s
    for s in tune8['tuning']['scenarios']:
        if s['type'] == 'reference':
            assert s['tail_s'] == 5.0


def _tiny_config(acceptance_failure):
    config = load_config('configs/arena_v2.json')
    config['sensor_feedback'] = dict(schema='sensor_feedback/1', mode='sensors', seed=3,
                                     profile=load_sensor_profile('configs/sensors/nominal.json'),
                                     tuning_seeds=dict(accept_check=2001))
    template = next(s for s in config['scenarios'] if s['type'] == 'gust')
    config['tuning']['scenarios'] = [dict(template, id='accept_check', speed='V_L', times_s=[.04]*3)]
    if acceptance_failure is not None:
        config['tuning']['objective'] = dict(config['tuning']['objective'], acceptance_failure=acceptance_failure)
    validate_config(config)
    return config


def _score(config, limits=None):
    from control.arena_factory import ArenaFactory
    from control.arena_tune import Evaluator
    factory = ArenaFactory(config)
    evaluator = Evaluator(config, factory.p, factory.model, 'CPID')
    if limits is not None:
        evaluator.limits = limits
    try:
        objective, scores = evaluator({})
        return objective, scores[0], evaluator.penalty
    finally:
        evaluator.close()


def test_acceptance_failure_off_keeps_the_old_entry():
    _, default, _ = _score(_tiny_config(None))
    _, off, _ = _score(_tiny_config(False))
    assert 'tracking_pass' not in default and 'acceptance_reasons' not in default
    assert off == default


def test_acceptance_failure_on_penalises_a_run_that_misses_acceptance():
    from dataclasses import replace
    from control.validation_metrics import Acceptance
    config = _tiny_config(True)
    _, plain, _ = _score(_tiny_config(None))
    objective, entry, penalty = _score(config)
    # 궤적은 같다 — 판정만 더해진다
    assert entry['trajectory_sha256'] == plain['trajectory_sha256']
    assert entry['failed'] == (plain['failed'] or not entry['tracking_pass'])
    # 합격 기준을 일부러 못 넘게 조이면(정착 각속도 한계 ≈ 0 — 시뮬 중단 조건은 건드리지 않는다)
    # 멈추지 않은 평가도 벌점을 받는다
    strict = replace(Acceptance(**config["acceptance"]), recovery_omega=1e-9)
    objective, entry, penalty = _score(config, strict)
    assert entry['tracking_pass'] is False and entry['failed'] is True
    assert entry['score'] == penalty == objective
    assert entry['stop_reason'] is None and entry['acceptance_reasons']
    # 같은 조임을 꺼진 설정에 걸면 판정이 바뀌지 않는다
    _, off, _ = _score(_tiny_config(False), strict)
    assert off['failed'] == plain['failed'] and off['score'] == plain['score']
