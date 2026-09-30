from copy import deepcopy
import json

import pytest

from control.arena import load_config
from scripts import sensor_preparation as prep


def test_plan_has_all_controllers_truth_controls_and_disjoint_seeds():
    plan = prep.make_plan()
    assert plan['counts'] == {'integration': 45, 'gnss_startup': 48}
    tasks = {t['trial_id']: t for t in plan['tasks']}
    assert len(tasks) == 93
    assert sum(t['feedback'] == 'truth' for t in tasks.values()) == 9
    assert all(t['seed'] not in range(1000, 3000) for t in tasks.values())
    for t in tasks.values():
        assert t['controller'] != 'CPID' or t['case'].endswith('_VL')
        if t['truth_trial_id']:
            truth = tasks[t['truth_trial_id']]
            assert truth['feedback'] == 'truth'
            assert truth['case'] == t['case'] and truth['controller'] == t['controller']
            a, b = deepcopy(t['config']), deepcopy(truth['config'])
            a.pop('sensor_feedback'); b.pop('sensor_feedback')
            assert a == b


def test_plan_runtime_hash_keys_are_portable_across_operating_systems(monkeypatch):
    baseline = prep.make_plan(seeds=[3])
    windows = {k.replace('/', '\\'): v for k, v in baseline['runtime_source_sha256'].items()}
    monkeypatch.setattr(prep, 'runtime_source_hashes', lambda: windows)
    assert prep.make_plan(seeds=[3]) == baseline


def test_factorial_varies_generated_noise_and_delay_but_not_covariance_or_gains():
    base = load_config(prep.CONFIG)
    before = deepcopy(base)
    for context in ('isolated', 'full'):
        profiles = [prep.profile_for(base, context, noise, delay)
                    for noise in (False, True) for delay in (False, True)]
        for p in profiles:
            assert p['estimator'] == profiles[0]['estimator']
            assert p['estimator']['gnss_pos_sigma'] == .8
            assert p['estimator']['gnss_vel_sigma'] == .15
            assert p['estimator']['trace_updates'] is True
            assert p['gnss']['rate_hz'] == 10.
        assert [p['gnss']['pos_sigma'] for p in profiles] == [0., 0., .8, .8]
        assert [p['gnss']['latency_s'] for p in profiles] == [0., .12, 0., .12]
        for p in profiles:
            p.pop('gnss'); p.pop('name')
        assert all(p == profiles[0] for p in profiles)
    assert base == before


@pytest.mark.parametrize('seeds', [[], [3, 3], [-1], [True], [1000], [2001]])
def test_reject_bad_or_reserved_seeds(seeds):
    with pytest.raises(ValueError):
        prep.make_plan(seeds=seeds)


def test_plan_is_nonexecuting_and_refuses_changes(tmp_path, monkeypatch):
    monkeypatch.setattr(prep, 'run_campaign', lambda *a, **k: pytest.fail('plan ran a simulation'))
    assert prep.main(['plan', '--output', str(tmp_path), '--seeds', '3']) == 0
    assert not (tmp_path/'results.json').exists()
    assert prep.checked_plan(tmp_path)['counts']['gnss_startup'] == 16
    with pytest.raises(ValueError, match='plan changed'):
        prep.save_plan(tmp_path, prep.make_plan(seeds=[4]))
    data = json.loads((tmp_path/'plan.json').read_text())
    data['tasks'][0]['config']['controllers']['V13']['indi_cutoff_hz'] = 5.
    (tmp_path/'plan.json').write_text(json.dumps(data))
    with pytest.raises(ValueError, match='saved plan'):
        prep.checked_plan(tmp_path)


def fake_runner(*args, **kw):
    return dict(records=[dict(controller=kw['controllers'][0], scenario_id=kw['cases'][0],
        feedback=kw['feedback'], sensor_seed=kw['seed'], passed=False,
        campaign_timed_out=True, simulated_seconds=None, failure_reasons=['campaign_timeout'])])


def test_resume_retains_failures_and_reuses_truth_controls(tmp_path):
    prep.save_plan(tmp_path, prep.make_plan(seeds=[3]))
    calls = []
    def runner(*args, **kwargs):
        calls.append(kwargs)
        return fake_runner(*args, **kwargs)
    a = prep.execute(tmp_path, 'gnss_startup', runner)
    assert len(a['records']) == 18  # sixteen factorial cells + two shared truth controls
    assert not a['complete']
    assert all(r['campaign_timed_out'] for r in a['records'])
    assert len(calls) == 18
    assert prep.execute(tmp_path, 'gnss_startup', runner) == a
    assert len(calls) == 18
    b = prep.execute(tmp_path, 'integration', runner)
    assert b['complete'] and len(calls) == 43  # 27 integration + 16 startup
    assert not (tmp_path/'run.lock').exists()


def test_missing_output_saved_before_stop_and_lock_released(tmp_path):
    prep.save_plan(tmp_path, prep.make_plan(seeds=[3]))
    def runner(*args, **kwargs):
        result = fake_runner(*args, **kwargs)
        result['records'][0]['failure_reasons'] = ['no_trial_output']
        return result
    with pytest.raises(RuntimeError, match='no trial output'):
        prep.execute(tmp_path, 'integration', runner)
    assert len(json.loads((tmp_path/'results.json').read_text())['records']) == 1
    assert not (tmp_path/'run.lock').exists()


def test_lock_and_bad_resume_records_are_rejected(tmp_path):
    prep.save_plan(tmp_path, prep.make_plan(seeds=[3]))
    (tmp_path/'run.lock').write_text('another writer')
    with pytest.raises(FileExistsError):
        prep.execute(tmp_path, 'gnss_startup', fake_runner)
    (tmp_path/'run.lock').unlink()
    result = prep.execute(tmp_path, 'gnss_startup', fake_runner)
    result['records'].append(result['records'][0])
    (tmp_path/'results.json').write_text(json.dumps(result))
    with pytest.raises(ValueError, match='duplicate'):
        prep.execute(tmp_path, 'integration', fake_runner)
