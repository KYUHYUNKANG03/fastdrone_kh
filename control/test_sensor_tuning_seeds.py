"""D4 (c) 시나리오별 고정 튜닝 시드 — sensor_feedback.tuning_seeds.

튜닝 시나리오마다 시드 하나를 설정에 미리 적어 두고(설정 해시에 고정), 모든 제어기·후보에 그대로 쓴다.
확인하는 것: 기존 단일 시드 경로와 비트 동일 / 실행 순서·작업자 수·재시작과 무관 / 기록·재개·I-4·게인 로딩·
재현 도구가 매핑을 검사한다 / 잘못된 매핑은 설정 단계에서 거부된다.
시나리오는 0.12 s 돌풍 두 개(CPID)라 가볍다.
"""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from control.arena import load_config, validate_config, config_sha256
from control.arena_sensors import load_sensor_profile
from control.sensor_binding import sensor_record_problems

IDS = ('seed_check_a', 'seed_check_b')
SIGN = dict(seed_check_a=1, seed_check_b=-1, seed_check_c=1, seed_check_d=-1)


def sensor_config(seed=2001, tuning_seeds=None, ids=IDS):
    config = load_config('configs/arena_v2.json')
    config['sensor_feedback'] = dict(schema='sensor_feedback/1', mode='sensors', seed=seed,
                                     profile=load_sensor_profile('configs/sensors/nominal.json'))
    if tuning_seeds is not None:
        config['sensor_feedback']['tuning_seeds'] = dict(tuning_seeds)
    template = next(s for s in config['scenarios'] if s['type'] == 'gust')
    # 두 시나리오의 돌풍 방향을 반대로 — 내용이 달라야 순서·시드 뒤바뀜을 잡는다.
    # 부호는 목록 위치가 아니라 id에 묶는다(순서를 뒤집는 시험에서 내용이 같이 바뀌지 않게).
    config['tuning']['scenarios'] = [dict(template, id=sid, speed='V_L', times_s=[.04]*3,
                                          peak_m_s=template['peak_m_s']*SIGN[sid])
                                     for sid in ids]
    validate_config(config)
    return config


MAPPED = dict(seed_check_a=2001, seed_check_b=2002)


@pytest.fixture(scope='module')
def plant():
    from control.arena_factory import ArenaFactory
    factory = ArenaFactory(sensor_config())
    return factory.p, factory.model


def evaluate(config, plant, workers=1, label='CPID'):
    from control.arena_tune import Evaluator
    evaluator = Evaluator(config, plant[0], plant[1], label, workers)
    try:
        return evaluator({})
    finally:
        evaluator.close()


def by_id(scores):
    return {s['id']: s for s in scores}


# ── 기존 경로와 비트 동일 ────────────────────────────────────────

def test_single_scenario_mapping_equals_single_seed(plant):
    one = IDS[:1]
    single = evaluate(sensor_config(seed=2001, ids=one), plant)
    mapped = evaluate(sensor_config(seed=3, tuning_seeds={one[0]: 2001}, ids=one), plant)
    assert mapped == single


def test_each_mapped_scenario_equals_old_path_with_its_seed(plant):
    _, mapped = evaluate(sensor_config(seed=3, tuning_seeds=MAPPED), plant)
    for sid, seed in MAPPED.items():
        _, old = evaluate(sensor_config(seed=seed), plant)
        assert by_id(mapped)[sid] == by_id(old)[sid]
        assert by_id(mapped)[sid]['sensor_seed'] == seed


def test_swapping_seeds_changes_trajectories(plant):
    # 반대 방향 시험: 시드가 실제로 쓰이지 않으면 위 비트 동일 시험이 우연히 통과할 수 있다
    _, a = evaluate(sensor_config(seed=3, tuning_seeds=MAPPED), plant)
    swapped = dict(seed_check_a=2002, seed_check_b=2001)
    _, b = evaluate(sensor_config(seed=3, tuning_seeds=swapped), plant)
    for sid in IDS:
        assert by_id(a)[sid]['trajectory_sha256'] != by_id(b)[sid]['trajectory_sha256']


# ── 실행 순서·작업자 수와 무관 ───────────────────────────────────

def test_mapping_is_independent_of_workers_and_scenario_order(plant):
    config = sensor_config(seed=3, tuning_seeds=MAPPED)
    sequential = evaluate(config, plant, workers=1)
    assert evaluate(config, plant, workers=2) == sequential
    reversed_config = sensor_config(seed=3, tuning_seeds=MAPPED, ids=IDS[::-1])
    _, reordered = evaluate(reversed_config, plant)
    assert [s['id'] for s in reordered] == list(IDS[::-1])
    assert by_id(reordered) == by_id(sequential[1])


def test_mapping_is_independent_of_three_workers(plant):
    # 시나리오가 2개면 작업자 수가 min(작업자, 시나리오)=2로 잘린다 — 4개로 작업자 3을 실제로 쓴다
    ids = ('seed_check_a', 'seed_check_b', 'seed_check_c', 'seed_check_d')
    seeds = dict(zip(ids, (2001, 2002, 2003, 2004)))
    config = sensor_config(seed=3, tuning_seeds=seeds, ids=ids)
    sequential = evaluate(config, plant, workers=1)
    assert evaluate(config, plant, workers=3) == sequential
    assert [s['sensor_seed'] for s in sequential[1]] == [2001, 2002, 2003, 2004]


# ── 기록·재개·I-4·게인 로딩 ──────────────────────────────────────

def tune(config, plant, run_dir, budget=2):
    from control.arena_tune import tune_controller
    return tune_controller(config, 'CPID', budget, run_dir, native=plant[0], model=plant[1])


def test_record_lists_mapping_in_scenario_order(plant, tmp_path):
    from control.arena_tune import check_tuning_records
    config = sensor_config(seed=3, tuning_seeds=MAPPED)
    record = tune(config, plant, tmp_path/'run')
    assert record['seeds']['used'] == [2001, 2002]
    assert record['seeds']['by_scenario'] == MAPPED
    assert record['sensor_seed'] == 3                 # 기본 seed는 기록되지만 튜닝에는 안 쓰였다
    assert check_tuning_records([record], config) == []
    for line in (tmp_path/'run'/'CPID.jsonl').read_text(encoding='utf-8').splitlines():
        for score in json.loads(line)['scenarios'] or []:
            assert score['sensor_seed'] == MAPPED[score['id']]


def test_resume_recomputes_with_the_same_seeds(plant, tmp_path):
    config = sensor_config(seed=3, tuning_seeds=MAPPED)
    run = tmp_path/'run'
    tune(config, plant, run)
    log = run/'CPID.jsonl'
    lines = log.read_text(encoding='utf-8').splitlines()
    log.write_text(lines[0] + '\n', encoding='utf-8')          # 두 번째 평가 전에 끊긴 것처럼
    tune(config, plant, run)
    assert log.read_text(encoding='utf-8').splitlines() == lines


@pytest.mark.parametrize('field', ['used', 'by_scenario'])
def test_tampered_record_is_rejected_everywhere(plant, tmp_path, field):
    from control.arena_tune import check_tuning_records, check_resume_compatible
    from control.arena_design_check import tuned_overrides
    config = sensor_config(seed=3, tuning_seeds=MAPPED)
    run = tmp_path/'run'
    record = tune(config, plant, run)
    if field == 'used':
        record['seeds']['used'] = [2002, 2001]
    else:
        record['seeds']['by_scenario'] = dict(seed_check_a=2002, seed_check_b=2001)
    (run/'CPID.record.json').write_text(json.dumps(record), encoding='utf-8')
    assert sensor_record_problems(record, config)
    assert check_tuning_records([record], config)
    with pytest.raises(RuntimeError, match='seeds differ'):
        check_resume_compatible(config, 'CPID', run)
    with pytest.raises(ValueError, match='seeds differ'):
        tuned_overrides(config, run, ('CPID',))


def test_resume_refuses_a_changed_mapping(plant, tmp_path):
    from control.arena_tune import check_resume_compatible
    run = tmp_path/'run'
    tune(sensor_config(seed=3, tuning_seeds=MAPPED), plant, run)
    changed = sensor_config(seed=3, tuning_seeds=dict(seed_check_a=2001, seed_check_b=2003))
    with pytest.raises(RuntimeError) as info:
        check_resume_compatible(changed, 'CPID', run)
    # 두 겹 모두 걸린다: 설정 해시, 시나리오별 시드
    assert 'config sha256 differs' in str(info.value)
    assert 'seeds differ' in str(info.value)


def test_old_single_seed_record_has_no_seed_problems(plant, tmp_path):
    config = sensor_config(seed=2001)
    record = tune(config, plant, tmp_path/'run', budget=1)
    assert 'by_scenario' not in record['seeds'] and record['seeds']['used'] == [2001]
    assert sensor_record_problems(record, config) == []


# ── 재현 도구 ────────────────────────────────────────────────────

def test_repro_reproduces_mapped_record_and_catches_a_wrong_seed(plant, tmp_path):
    from control.arena_tune_repro import reproduce
    config = sensor_config(seed=3, tuning_seeds=MAPPED)
    run = tmp_path/'run'
    tune(config, plant, run, budget=1)
    result = reproduce(config, 'CPID', run, 0, 1)
    assert result['passed'] and result['bit_identical']
    log = run/'CPID.jsonl'
    entry = json.loads(log.read_text(encoding='utf-8').splitlines()[0])
    entry['scenarios'][1]['sensor_seed'] = 2001
    log.write_text(json.dumps(entry) + '\n', encoding='utf-8')
    tampered = reproduce(config, 'CPID', run, 0, 1)
    assert not tampered['passed']
    assert any('sensor_seed' in p for p in tampered['tolerance_problems'])


# ── 설정 검사 ────────────────────────────────────────────────────

@pytest.mark.parametrize('seeds,message', [
    (dict(seed_check_a=2001), 'exactly the tuning scenarios'),
    (dict(MAPPED, seed_check_c=2003), 'exactly the tuning scenarios'),
    (dict(seed_check_a=2001, seed_check_b=1500), 'seeds.tuning'),
    (dict(seed_check_a=2001, seed_check_b=2001), 'distinct'),
    (dict(seed_check_a=2001, seed_check_b=2.5), 'seed'),
    (dict(seed_check_a=2001, seed_check_b=True), 'seed'),
    ({}, 'nonempty'),
])
def test_invalid_mapping_is_rejected(seeds, message):
    config = sensor_config()
    config['sensor_feedback']['seed'] = 3
    config['sensor_feedback']['tuning_seeds'] = seeds
    with pytest.raises(ValueError, match=message):
        validate_config(config)


def test_default_seed_must_not_be_a_tuning_seed():
    config = sensor_config()
    config['sensor_feedback']['tuning_seeds'] = MAPPED            # 기본 seed 2001 = 튜닝 시드 → 표본 내
    with pytest.raises(ValueError, match='differ from every tuning seed'):
        validate_config(config)


def test_mapping_requires_sensor_mode():
    config = sensor_config(seed=3, tuning_seeds=MAPPED)
    config['sensor_feedback']['mode'] = 'truth'
    config['sensor_feedback'].pop('profile')
    with pytest.raises(ValueError, match='requires mode sensors'):
        validate_config(config)


def test_dev_default_seed_can_tune_only_with_a_mapping(plant):
    from control.arena_tune import Evaluator
    with pytest.raises(ValueError, match='seeds.tuning'):
        Evaluator(sensor_config(seed=3), plant[0], plant[1], 'CPID')
    Evaluator(sensor_config(seed=3, tuning_seeds=MAPPED), plant[0], plant[1], 'CPID').close()


def test_missing_mapping_entry_fails_loudly_even_without_validation(plant):
    # load_config를 거치지 않은 메모리 설정이라도 빠진 id가 기본 seed로 조용히 돌지 않는다
    from control.arena_tune import Evaluator
    config = sensor_config(seed=3, tuning_seeds=MAPPED)
    del config['sensor_feedback']['tuning_seeds']['seed_check_b']
    evaluator = Evaluator(config, plant[0], plant[1], 'CPID')
    try:
        with pytest.raises(KeyError, match='seed_check_b'):
            evaluator({})
    finally:
        evaluator.close()


def test_main_experiment_seeds_stay_disjoint_from_mapped_tuning_seeds():
    from control.sensor_binding import main_sensor_seeds
    config = sensor_config(seed=3, tuning_seeds=MAPPED)
    assert main_sensor_seeds({'sensor_seeds': [1000, 1001]}, config) == [1000, 1001]
    for seed in list(MAPPED.values()) + [3]:            # 튜닝 시드도, 기본(개발) 시드도 본 실험에 못 쓴다
        with pytest.raises(ValueError, match='main range'):
            main_sensor_seeds({'sensor_seeds': [1000, seed]}, config)


def test_mapping_is_part_of_the_config_hash():
    a = sensor_config(seed=3, tuning_seeds=MAPPED)
    b = sensor_config(seed=3, tuning_seeds=dict(seed_check_a=2001, seed_check_b=2003))
    assert config_sha256(a) != config_sha256(b)
