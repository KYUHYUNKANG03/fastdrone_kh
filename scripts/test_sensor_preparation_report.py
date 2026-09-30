import json
import pytest

from scripts import sensor_preparation as prep, sensor_preparation_report as report
from scripts.test_sensor_preparation import fake_runner


def test_unstarted_plan_reports_all_pending_without_inventing_results(tmp_path):
    source = tmp_path/'source'
    prep.save_plan(source, prep.make_plan(seeds=[3]))
    result = report.summarize(source, tmp_path/'report')
    assert result['expected'] == 43
    assert result['execution_counts'] == {'pending': 43}
    assert all('passed' not in row for row in result['records'])


def test_timeouts_remain_in_denominator_and_pending_remains_separate(tmp_path):
    source = tmp_path/'source'
    prep.save_plan(source, prep.make_plan(seeds=[3]))
    prep.execute(source, 'gnss_startup', fake_runner)
    result = report.summarize(source, tmp_path/'report')
    assert result['execution_counts'] == {'execution_incomplete': 18, 'pending': 25}
    assert sum(r.get('campaign_timed_out', False) for r in result['records']) == 18
    assert sum(r.get('paired_outcome') == 'execution_incomplete' for r in result['records']) == 16


@pytest.mark.parametrize('field,value', [('input_config_sha256', 'wrong'), ('truth_trial_id', 'other'),
                                        ('controller', 'CPID'), ('sensor_seed', 1000)])
def test_changed_result_identity_rejected(tmp_path, field, value):
    source = tmp_path/'source'
    prep.save_plan(source, prep.make_plan(seeds=[3]))
    data = prep.execute(source, 'gnss_startup', fake_runner)
    data['records'][0][field] = value
    (source/'results.json').write_text(json.dumps(data))
    with pytest.raises(ValueError):
        report.summarize(source, tmp_path/'report')


def test_nonzero_command_exit_is_not_hidden_by_a_trial_verdict(tmp_path):
    source = tmp_path/'source'
    prep.save_plan(source, prep.make_plan(seeds=[3]))
    def runner(*args, **kwargs):
        result = fake_runner(*args, **kwargs)
        result['records'][0].update(campaign_timed_out=False, passed=True,
            simulated_seconds=12., returncode=1)
        return result
    prep.execute(source, 'gnss_startup', runner)
    result = report.summarize(source, tmp_path/'report')
    assert result['execution_counts']['execution_incomplete'] == 18
    assert sum(r.get('paired_outcome') == 'execution_incomplete' for r in result['records']) == 16
