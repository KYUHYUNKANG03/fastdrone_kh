import json
import pytest
from scripts import sensor_group_report as report


def manifest(root, **changes):
    data = dict(complete=True, expected_trials=1,
        contract=dict(runtime_source_sha256={'runtime': 'abc'}),
        records=[dict(trial_id='baseline/nominal/V13/case/seed_3', variant='baseline',
            fault='nominal', controller='V13', sensor_seed=3, scenario_id='case',
            passed=False, tracking_pass=False, model_domain_valid=False,
            campaign_timed_out=True, failure_reasons=['campaign_timeout'])])
    data.update(changes)
    path = root/'experiment.json'
    path.write_text(json.dumps(data))
    return path


def setup(monkeypatch, tmp_path):
    monkeypatch.setattr(report, 'ROOT', tmp_path)
    monkeypatch.setattr(report, 'runtime_source_hashes', lambda: {'runtime': 'abc'})


def test_timeout_survives_report_without_a_trace(tmp_path, monkeypatch):
    setup(monkeypatch, tmp_path)
    data = report.summarize([manifest(tmp_path)], tmp_path/'report')
    row = data['records'][0]
    assert row['campaign_timed_out'] and not row['passed']
    assert row['failure_reasons'] == ['campaign_timeout']
    assert row['trace'] is None and row['diagnostics'] is None
    assert row['simulated_seconds'] is None


@pytest.mark.parametrize('change,reason', [
    ({'complete':False}, 'incomplete'), ({'expected_trials':2}, 'incomplete'),
    ({'contract':{'runtime_source_sha256':{}}}, 'runtime differs')])
def test_rejects_incomplete_or_different_runtime(tmp_path, monkeypatch, change, reason):
    setup(monkeypatch, tmp_path)
    with pytest.raises(ValueError, match=reason):
        report.summarize([manifest(tmp_path, **change)], tmp_path/'report')


def test_duplicate_trials_cannot_inflate_report_counts(tmp_path, monkeypatch):
    setup(monkeypatch, tmp_path)
    source = manifest(tmp_path)
    with pytest.raises(ValueError, match='duplicate trial'):
        report.summarize([source, source], tmp_path/'report')
