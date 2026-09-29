from copy import deepcopy
import numpy as np
import pytest

from control.arena_estimator import NavigationFilter
from control.arena_sensors import Measurement, SensorSuite, load_sensor_profile
from control.navigation_grid_campaign import navigation_profile
from control.navigation_update_experiment import CHANGES, variant_profile


def state():
    x = np.zeros(17)
    x[9] = 1.
    return x


def test_trace_enabled_is_bit_identical_with_delayed_replays():
    base = load_sensor_profile('configs/sensors/nominal.json')
    traced = deepcopy(base)
    traced['estimator']['trace_updates'] = True
    a, b = NavigationFilter(state(), base), NavigationFilter(state(), traced)
    suite = SensorSuite(base, .002, seed=1)
    arrived = set()
    for k in range(151):
        t = k*.002
        packets = [p for p in suite.sample(t, state()) if p.kind != 'rpm']
        for p in packets:
            if p.kind != 'imu' and p.arrival_time <= .3+1e-9:
                arrived.add(p.sequence)
        a.advance(t, packets); b.advance(t, packets)
        np.testing.assert_array_equal(a.state, b.state)
        np.testing.assert_array_equal(a.P, b.P)
        assert a.baro_bias == b.baro_bias and a.diagnostics == b.diagnostics
    assert a.update_trace == a.arrival_trace == []
    assert {r['sequence'] for r in b.update_trace} == arrived
    assert len(b.update_trace) == len(arrived)
    assert all(r['processed_at_s']+1e-9 >= r['arrival_time_s'] for r in b.update_trace)


def test_scalar_innovation_covariance_and_injected_correction_are_logged():
    profile = load_sensor_profile({'schema': 'sensor/1', 'estimator':
                                  {'trace_updates': True, 'initialize_baro_bias': False}})
    filt = NavigationFilter(state(), profile)
    filt.advance(0., [Measurement('barometer', 0., 0., [1.], 1)])
    row = filt.update_trace[0]
    np.testing.assert_allclose(row['innovation'], [1.])
    np.testing.assert_allclose(row['innovation_covariance'], [[.25+.36]])
    assert row['correction_error_state'][2] == pytest.approx(.25/(.25+.36))
    assert row['state_before'][2] == 0.
    assert row['state_after'][2] == pytest.approx(row['correction_error_state'][2])
    assert row['nis'] == pytest.approx(1./.61)


def test_later_replay_does_not_rewrite_first_processing_record():
    profile = load_sensor_profile({'schema': 'sensor/1', 'estimator':
                                  {'trace_updates': True, 'initialize_baro_bias': False}})
    filt = NavigationFilter(state(), profile)
    filt.advance(0., [Measurement('imu', 0., 0., [0., 0., 9.81, 0., 0., 0.], 1)])
    filt.advance(.04, [Measurement('barometer', .04, .04, [.1], 2)])
    original = deepcopy(filt.update_trace[0])
    filt.advance(.1, [Measurement('gnss', .02, .1, [.1, .2, .3, 0., 0., 0.], 3)])
    assert filt.update_trace[0] == original
    assert len(filt.update_trace) == 2
    assert filt.arrival_trace[-1]['replayed_existing_measurements'] == 1
    assert filt.update_trace[-1]['sample_time_s'] == .02
    assert filt.update_trace[-1]['processed_at_s'] == .1


def test_rejected_update_has_no_injected_error_state():
    profile = load_sensor_profile({'schema': 'sensor/1', 'estimator':
                                  {'trace_updates': True, 'initialize_baro_bias': False}})
    filt = NavigationFilter(state(), profile)
    filt.advance(0., [Measurement('barometer', 0., 0., [1000.], 1)])
    row = filt.update_trace[0]
    assert row['status'] == 'rejected'
    np.testing.assert_array_equal(row['correction_error_state'], np.zeros(15))
    np.testing.assert_array_equal(row['state_before'], row['state_after'])


def test_stale_packet_is_logged_once_without_an_update():
    profile = load_sensor_profile({'schema': 'sensor/1', 'estimator':
                                  {'trace_updates': True, 'history_s': .05}})
    filt = NavigationFilter(state(), profile)
    imu = lambda t, seq: Measurement('imu', t, t, [0., 0., 9.81, 0., 0., 0.], seq)
    filt.advance(0., [imu(0., 1)])
    filt.advance(.1, [imu(.1, 2)])
    packet = Measurement('barometer', 0., .2, [.1], 3)
    filt.advance(.2, [packet])
    filt.advance(.21, [packet])
    assert len(filt.update_trace) == 1 and filt.update_trace[0]['status'] == 'stale'
    assert 'correction_error_state' not in filt.update_trace[0]


@pytest.mark.parametrize('variant', CHANGES)
def test_variants_change_only_declared_measurement_fields_and_logging(variant):
    base = navigation_profile('configs/sensors/nominal.json', 'configs/sensors/stress.json', 0., 0.)
    original = deepcopy(base)
    updated = variant_profile(base, variant)
    expected = deepcopy(base)
    for block, fields in CHANGES[variant].items():
        expected[block].update(deepcopy(fields))
    expected['estimator']['trace_updates'] = True
    if variant != 'baseline': expected['name'] += '_'+variant
    assert updated == expected and base == original
    for field, value in base['estimator'].items():
        assert updated['estimator'][field] == value
