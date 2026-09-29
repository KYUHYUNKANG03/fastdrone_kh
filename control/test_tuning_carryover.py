"""튜닝 기록 승계(control/tuning_carryover.py, arena_tune_repro 승계 모드) — kj 결정(2026-09-28 밤).

  1) 표에 있는 기록(파일 sha256 일치)만 arena.json 해시가 arena_v2에서 통과한다.
  2) 기록 파일이 바뀌면, 표에 없으면, 표가 없으면, 증명이 비트 동일이 아니면 거부한다.
  3) 승계 모드는 보정 대상(M17·F13·GSLQR)이나 모멘트 절 밖이 다른 설정에서는 열리지 않는다.
  4) 튜닝값 로더(tuned_overrides)가 같은 규칙을 쓴다.
"""
from copy import deepcopy
import json
import shutil
import hashlib
import sys

import pytest

from control.arena import ROOT, load_config, config_sha256
from control.arena_tune_repro import carryover_allowed
from control.tuning_carryover import load_table, record_hash_problems, DEFAULT_TABLE

RUN_DIR = ROOT/'results'/'arena'/'tuning'/'retune_v3'
V2 = ROOT/'configs'/'arena_v2.json'


@pytest.fixture(scope='module')
def priors():
    from control.arena_factory import ArenaFactory
    from control.arena_tune import parameter_space
    config = load_config(); gains = ArenaFactory(config).gains
    return {label: parameter_space(config, label, gains)[1] for label in ('CPID', 'GSLQR')}


@pytest.fixture(autouse=True)
def synthetic_records(tmp_path, monkeypatch, priors):
    """Exercise the guard without depending on untracked historical tuning files.

    Synthetic proofs are confined to pytest's temporary directory and patched
    loader. They are test inputs, not evidence permitting real carryover.
    """
    config = load_config()
    folder = tmp_path/'synthetic_records'; folder.mkdir()
    monkeypatch.setattr(sys.modules[__name__], 'RUN_DIR', folder)
    for label, values in priors.items():
        record = dict(controller=label, config_sha256=config_sha256(config), status='complete',
            budget=1, spent=1, scenario_ids=[s['id'] for s in config['tuning']['scenarios']],
            seeds=dict(tuning_range=config['seeds']['tuning'], used=[]),
            objective=config['tuning']['objective'], search=config['tuning']['search'], best_values=values)
        (folder/f'{label}.record.json').write_text(json.dumps(record), encoding='utf-8')
    doc = dict(schema='tuning_carryover/1', base_config_sha256=config_sha256(config),
        target_config_sha256=config_sha256(load_config(V2)), entries={'CPID': dict(
            record_sha256=hashlib.sha256((folder/'CPID.record.json').read_bytes()).hexdigest(),
            proofs={'evaluation_0': {'bit_identical': True}, 'best': {'bit_identical': True}})})
    import control.tuning_carryover as carryover
    monkeypatch.setattr(carryover, 'load_table', lambda *args: deepcopy(doc))
    return doc


@pytest.fixture
def table(synthetic_records):
    return deepcopy(synthetic_records)


def _record(label):
    path = RUN_DIR/f'{label}.record.json'
    return json.loads(path.read_text(encoding='utf-8')), path


def test_carried_record_passes_under_v2(table):
    assert record_hash_problems([_record('CPID')], load_config(V2), table) == []


def test_uncarried_old_record_is_refused_under_v2(table):
    problems = record_hash_problems([_record('GSLQR')], load_config(V2), table)
    assert problems and 'not in the carry-over table' in problems[0]


def test_edited_record_file_is_refused(table, tmp_path):
    record, path = _record('CPID')
    copy = tmp_path/'CPID.record.json'
    shutil.copy(path, copy)
    copy.write_text(copy.read_text(encoding='utf-8') + ' ', encoding='utf-8')
    problems = record_hash_problems([(record, copy)], load_config(V2), table)
    assert any('record file sha256' in p for p in problems)


def test_without_a_table_old_hashes_are_refused():
    assert record_hash_problems([_record('CPID')], load_config(V2), None)


def test_failed_proof_is_refused(table):
    bad = deepcopy(table)
    bad['entries']['CPID']['proofs']['best']['bit_identical'] = False
    assert any('bit-identical' in p for p in record_hash_problems([_record('CPID')], load_config(V2), bad))


def test_same_config_needs_no_table():
    assert record_hash_problems([_record('GSLQR')], load_config(), None) == []


def test_carryover_mode_is_closed_for_corrected_controllers_and_other_differences():
    v2 = load_config(V2)
    assert carryover_allowed(v2, 'CPID') is not None and carryover_allowed(v2, 'V13') is not None
    for label in ('M17', 'F13', 'GSLQR'):
        assert carryover_allowed(v2, label) is None
    other = deepcopy(v2)
    other['tuning']['budget'] = 7
    assert carryover_allowed(other, 'CPID') is None
    assert carryover_allowed(load_config(), 'CPID') is None


def test_tuned_overrides_uses_the_same_rule(table):
    from control.arena_design_check import tuned_overrides
    overrides, used = tuned_overrides(load_config(V2), RUN_DIR, ['CPID'])
    assert 'CPID' in used
    with pytest.raises(ValueError, match='not in the carry-over table'):
        tuned_overrides(load_config(V2), RUN_DIR, ['GSLQR'])
