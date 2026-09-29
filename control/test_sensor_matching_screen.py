from copy import deepcopy
import json
import pytest
from control.arena import load_config
from control.sensor_matching_screen import variant_config, run

CONFIG = 'configs/arena_sensor_candidate_v6.json'


@pytest.mark.parametrize('variant,path,value', [
    ('startup_guard', ('controllers', 'V13', 'rotor_startup_guard'), True),
    ('gyro_quiet', ('sensor_feedback', 'profile', 'imu', 'gyro_noise_density'), 0.),
    ('rotor_tau2ms', ('sensor_feedback', 'profile', 'rotor_observer', 'tau_s'), .002),
    ('cutoff25', ('controllers', 'V13', 'indi_cutoff_hz'), 25.)])
def test_ablation_changes_exactly_one_parameter_and_profile_label(variant, path, value):
    base = load_config(CONFIG); before = deepcopy(base)
    got = variant_config(base, 'V13', variant)
    expected = deepcopy(base); cursor = expected
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = value
    expected['sensor_feedback']['profile']['name'] = f'{variant}_nominal'
    assert got == expected and base == before


def test_resume_and_failures_are_counted_without_rerunning(tmp_path):
    calls = []
    def fake(*args, **kwargs):
        calls.append(kwargs['seed'])
        return dict(records=[dict(passed=False, tracking_pass=False, campaign_timed_out=True,
                                  failure_reasons=['campaign_timeout'])])
    args = (CONFIG, tmp_path, ['V13'], [3, 4], ['baseline'], ['nominal'], ['gust_lateral_p10_VH'])
    a = run(*args, runner=fake)
    assert a['complete'] and a['expected_trials'] == len(a['records']) == 2
    assert run(*args, runner=fake) == a and calls == [3, 4]
    doc = json.loads((tmp_path/'experiment.json').read_text())
    doc['contract']['runtime_source_sha256'] = {'changed': 'different'}
    (tmp_path/'experiment.json').write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='contract changed'):
        run(*args, runner=fake)


@pytest.mark.parametrize('seeds', [[1000], [2001], [-1]])
def test_development_rejects_reserved_or_invalid_seeds(tmp_path, seeds):
    with pytest.raises(ValueError):
        run(CONFIG, tmp_path, ['V13'], seeds, ['baseline'], ['nominal'], ['gust_lateral_p10_VH'])


def test_fault_changes_only_measurement_model():
    base = load_config(CONFIG)
    for fault in ('baro_drift', 'gnss_outlier', 'baro_outlier', 'gnss_outage', 'gyro_vibration', 'initial_error'):
        changed = variant_config(base, 'V13', 'baseline', fault)
        changed.pop('sensor_feedback')
        expected = deepcopy(base); expected.pop('sensor_feedback')
        assert changed == expected
