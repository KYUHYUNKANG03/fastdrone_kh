from copy import deepcopy
import json
import pytest
import numpy as np
from control.arena import load_config
from control.arena_sensors import SensorSuite
from control.arena_feedback import ArenaSensorFeedback
from control.sensor_matching_screen import GROUP_VARIANTS, sensor_group_profile, variant_config, run

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


@pytest.mark.parametrize('variant', GROUP_VARIANTS)
def test_group_controls_preserve_controller_and_effective_filter_covariance(variant):
    base = load_config(CONFIG); before = deepcopy(base)
    changed = variant_config(base, 'F13', variant)
    assert base == before
    assert {k:v for k,v in changed.items() if k != 'sensor_feedback'} == {
        k:v for k,v in base.items() if k != 'sensor_feedback'}
    nominal, diagnostic = base['sensor_feedback']['profile'], changed['sensor_feedback']['profile']
    x = np.zeros(17); x[9] = 1.
    a, b = [ArenaSensorFeedback(x, p, .002, seed=3).filter for p in (nominal, diagnostic)]
    # Compare covariance for identical inputs, independent of the deliberately
    # different generated samples and preflight calibration mean.
    np.testing.assert_array_equal(a.P, b.P)
    for _ in range(10):
        for f in (a, b):
            f._propagate(np.array([.3, .1, 9.9, .1, -.1, .2]), .002)
    np.testing.assert_array_equal(a.P, b.P)
    for sensor in ('imu', 'gnss', 'barometer', 'magnetometer', 'rpm'):
        assert diagnostic[sensor]['rate_hz'] == nominal[sensor]['rate_hz']
        assert diagnostic[sensor].get('enabled', True) == nominal[sensor].get('enabled', True)


@pytest.mark.parametrize('variant,sensors', [
    ('imu_only', ('imu',)), ('navigation_only', ('gnss', 'barometer', 'magnetometer')),
    ('rotor_only', ('rpm',))])
def test_restored_group_preserves_seeded_packets(variant, sensors):
    nominal = load_config(CONFIG)['sensor_feedback']['profile']
    a, b = [SensorSuite(p, .002, seed=3) for p in (nominal, sensor_group_profile(nominal, variant))]
    x = np.zeros(17); x[9] = 1.; x[13:] = 1500.
    for k in range(101):
        left, right = a.sample(k*.002, x), b.sample(k*.002, x)
        for p, q in zip(left, right):
            assert p.kind == q.kind
            if p.kind in sensors:
                assert (p.sample_time, p.arrival_time, p.sequence) == (q.sample_time, q.arrival_time, q.sequence)
                np.testing.assert_array_equal(p.value, q.value)


def test_quiet_control_clears_injected_errors_and_leaves_input_untouched():
    nominal = deepcopy(load_config(CONFIG)['sensor_feedback']['profile'])
    nominal['imu'].update(gyro_bias=[1., 2., 3.], gyro_vibration=[dict(frequency_hz=80., amplitude_rad_s=[.1]*3)])
    nominal['barometer'].update(bias=2., bias_rate_m_s=.1, outlier_windows=[dict(start_s=1.,end_s=2.,height_offset_m=5.)])
    nominal['gnss']['outage_windows'] = [[0., 2.]]
    before = deepcopy(nominal)
    quiet = sensor_group_profile(nominal, 'quiet_sampled')
    assert nominal == before
    assert quiet['imu']['gyro_bias'] == [0., 0., 0.]
    assert quiet['imu']['gyro_vibration'] == quiet['gnss']['outage_windows'] == []
    assert quiet['barometer']['bias'] == quiet['barometer']['bias_rate_m_s'] == 0.
    assert quiet['barometer']['outlier_windows'] == []
