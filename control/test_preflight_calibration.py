from copy import deepcopy
import numpy as np
import pytest

from control.arena_estimator import NavigationFilter
from control.arena_feedback import ArenaSensorFeedback
from control.arena_sensors import Measurement, SensorSuite, load_sensor_profile
from control.navigation_grid_campaign import navigation_profile
from control.preflight_calibration_experiment import VARIANTS, calibration_profile


def initial_state():
    x = np.zeros(17); x[2] = 20.; x[9] = 1.
    return x


def test_legacy_shared_stream_matches_saved_prechange_samples():
    # Captured from the previous Desktop implementation, seed 73, 100 samples.
    suite = SensorSuite({'schema': 'sensor/1'}, .002, seed=73)
    assert suite.preflight_barometer_bias(20., 100) == -0.03013926500282693
    barometer = [packet.value[0] for k in range(21)
                 for packet in suite.sample(k*.002, initial_state()) if packet.kind == 'barometer']
    np.testing.assert_array_equal(barometer, [19.71140171236508, 19.722124451102726, 19.649232547766097])


@pytest.mark.parametrize('count', [1, 25, 100])
def test_preflight_count_does_not_change_any_flight_packets(count):
    profile = load_sensor_profile({'schema': 'sensor/1', 'estimator': {'preflight_baro_rng': 'independent'}})
    baseline, averaged = (SensorSuite(profile, .002, seed=4) for _ in range(2))
    averaged.preflight_barometer_bias(20., count)
    for k in range(101):
        a, b = baseline.sample(k*.002, initial_state()), averaged.sample(k*.002, initial_state())
        assert len(a) == len(b)
        for left, right in zip(a, b):
            assert (left.kind, left.sequence, left.sample_time, left.arrival_time) == (
                right.kind, right.sequence, right.sample_time, right.arrival_time)
            np.testing.assert_array_equal(left.value, right.value)


def test_calibration_is_installed_before_initial_flight_measurement():
    profile = load_sensor_profile({'schema': 'sensor/1', 'barometer': {'sigma': 0., 'bias': 1.2},
        'estimator': {'preflight_baro_samples': 25, 'preflight_baro_rng': 'independent'}})
    feedback = ArenaSensorFeedback(initial_state(), profile, .002, seed=1)
    assert feedback.filter.baro_bias == pytest.approx(1.2)
    assert feedback.filter._initial_snapshot['baro_bias'] == pytest.approx(1.2)


@pytest.mark.parametrize('learn_bias', [True, False])
def test_delayed_gnss_does_not_reinitialize_preflight_reference(learn_bias):
    profile = load_sensor_profile({'schema': 'sensor/1', 'estimator':
        {'trace_updates': True, 'estimate_baro_bias': learn_bias}})
    filt = NavigationFilter(initial_state(), profile)
    filt.set_initial_baro_bias(.01)
    filt.advance(0., [Measurement('imu', 0., 0., [0., 0., 9.81, 0., 0., 0.], 1)])
    filt.advance(.02, [Measurement('barometer', 0., .02, [19.777], 3)])
    filt.advance(.12, [Measurement('gnss', 0., .12, [0., 0., 20.375, 0., 0., 0.], 2),
                       Measurement('barometer', .08, .1, [19.8], 4)])
    # Inserting the first GNSS before the initial barometer must retain calibration.
    assert filt.baro_bias == .01
    filt.advance(.22, [Measurement('gnss', .1, .22, [0., 0., 20.375, 0., 0., 0.], 5)])
    # Later GNSS-based learning is distinct from reinitializing the first reading.
    expected = .01+(1.-np.exp(-.1))*(19.8-20.375-.01) if learn_bias else .01
    assert filt.baro_bias == pytest.approx(expected)


@pytest.mark.parametrize('variant', VARIANTS)
def test_profiles_only_change_declared_calibration_fields(variant):
    base = navigation_profile('configs/sensors/nominal.json', 'configs/sensors/stress.json', 0., 0.)
    original = deepcopy(base)
    expected = deepcopy(base)
    expected['estimator'].update(VARIANTS[variant])
    expected['estimator']['trace_updates'] = True
    if variant != 'baseline': expected['name'] += '_'+variant
    assert calibration_profile(base, variant) == expected
    assert base == original


def test_invalid_calibration_rng_is_rejected():
    with pytest.raises(ValueError, match='preflight_baro_rng'):
        load_sensor_profile({'schema':'sensor/1', 'estimator':{'preflight_baro_rng':'bad'}})
