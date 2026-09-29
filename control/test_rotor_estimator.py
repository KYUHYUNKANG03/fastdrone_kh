import numpy as np
import pytest
from control.arena_feedback import RotorObserver, ArenaSensorFeedback
from control.arena_sensors import Measurement, SensorSuite, load_sensor_profile


def state():
    x = np.zeros(17); x[9] = 1.; x[13:] = 1000.
    return x


def profile(**options):
    p = load_sensor_profile('configs/sensors/ideal.json')
    p['rpm'].update(latency_s=.004, rate_hz=500.)
    p['rotor_observer'] = dict(source='telemetry_predictor', motor_tau_s=.02,
                              history_s=.05, max_age_s=.01, **options)
    return load_sensor_profile(p)


def packet(sample, arrival, value=1000., sequence=1):
    return Measurement('rpm', sample, arrival, np.full(4, value), sequence)


def flow(n, command, dt, tau=.02):
    return command+(n-command)*np.exp(-dt/tau)


def test_delayed_sample_projects_through_actual_piecewise_commands_without_truth_seed():
    r = RotorObserver(state(), profile(), .002)
    np.testing.assert_array_equal(r.n, np.zeros(4))
    r.update(np.full(4, 1200.), [packet(0., .006)], now=0.)
    assert not r.ready
    r.update(np.full(4, 1200.), [], now=.002)
    r.update(np.full(4, 800.), [], now=.004)
    actual = r.update(np.full(4, 1500.), [], now=.006)
    expected = flow(flow(flow(1000., 1200., .002), 800., .002), 1500., .002)
    np.testing.assert_allclose(actual, expected, atol=1e-12)
    assert r.ready and r.sample_time == 0.
    # Neither a changed future command nor new truth values were used in that estimate.
    future = r.update(np.full(4, 2000.), [], now=.008)
    np.testing.assert_allclose(future, flow(expected, 2000., .002))


def test_sample_inside_command_interval_and_late_old_packet():
    r = RotorObserver(state(), profile(), .002)
    r.update(np.full(4, 1200.), [packet(.001, .006, 1000.)], now=.002)
    result = r.update(np.full(4, 800.), [], now=.006)
    expected = flow(flow(1000., 1200., .001), 800., .004)
    np.testing.assert_allclose(result, expected)
    result = r.update(np.full(4, 1500.), [packet(0., .008, 9000., 2),
                                        packet(.001, .008, 9000., 3)], now=.008)
    np.testing.assert_allclose(result, flow(expected, 1500., .002))
    assert r._projector.counts['old_samples'] == 2


def test_newest_arrived_sample_wins_and_zero_delay_uses_no_future_command():
    r = RotorObserver(state(), profile(), .002)
    result = r.update(np.full(4, 9000.), [packet(.004, .004, 1234., 2),
                                        packet(0., .004, 1000., 1)], now=.004)
    np.testing.assert_array_equal(result, np.full(4, 1234.))
    assert r.sample_time == .004


def test_dropouts_keep_predicting_but_readiness_expires_and_recovers():
    r = RotorObserver(state(), profile(), .002)
    r.update(np.full(4, 1200.), [packet(0., 0.)], now=0.)
    assert r.ready
    result = r.update(np.full(4, 1200.), [], now=.012)
    np.testing.assert_allclose(result, flow(1000., 1200., .012))
    assert not r.ready
    r.update(np.full(4, 1200.), [packet(.014, .016, 1100., 2)], now=.016)
    assert r.ready


def test_motor_model_mismatch_is_visible_and_not_corrected_with_hidden_truth():
    p = profile(); p['rotor_observer']['motor_tau_s'] = .04
    r = RotorObserver(state(), p, .002)
    result = r.update(np.full(4, 1500.), [packet(0., .004)], now=.004)
    np.testing.assert_allclose(result, flow(1000., 1500., .004, tau=.04))
    assert abs(result[0]-flow(1000., 1500., .004, tau=.02)) > 40.


def test_history_is_bounded_and_missing_history_cannot_be_invented():
    r = RotorObserver(state(), profile(), .002)
    for k in range(1, 501):
        r.update(np.full(4, 1200.), [], now=k*.002)
    assert len(r._projector._history) <= 26
    before = r.n.copy()
    r.update(np.full(4, 1200.), [packet(.1, .1, 9000.)], now=1.)
    np.testing.assert_array_equal(r.n, before)
    assert not r.ready and r._projector.counts['outside_history'] == 1
    r.update(np.zeros(4), [packet(1., 100., 9000., 2)], now=1.)
    assert not r._projector._pending


@pytest.mark.parametrize('bad', [packet(.01, .01), packet(0., -.01),
    packet(float('nan'), .004), packet(0., .004, float('nan')),
    Measurement('rpm', 0., .004, [1., 2., 3.], 1)])
def test_invalid_or_future_samples_cannot_change_estimate(bad):
    r = RotorObserver(state(), profile(), .002)
    r.update(np.zeros(4), [bad], now=.004)
    np.testing.assert_array_equal(r.n, np.zeros(4))
    assert not r.ready and r._projector.counts['invalid_packets'] == 1


@pytest.mark.parametrize('bad', [dict(motor_tau_s=0.), dict(history_s=.001),
    dict(max_age_s=float('nan')), dict(prehistory_s=-1.), dict(initial_rpm=[1., 2.])])
def test_invalid_profile_fails_early(bad):
    p = profile(); p['rotor_observer'].update(bad)
    with pytest.raises(ValueError): load_sensor_profile(p)


def test_bad_time_and_command_fail_before_state_change():
    r = RotorObserver(state(), profile(), .002)
    r.update(np.zeros(4), [], now=.002)
    for command, now in (([1., 2.], .004), ([0.]*4, .001), ([0.]*4, float('nan'))):
        with pytest.raises(ValueError):r.update(command, [], now=now)
    np.testing.assert_array_equal(r.n, np.zeros(4))


@pytest.mark.parametrize('source', ['telemetry', 'telemetry_predictor'])
def test_steady_trim_prehistory_has_arrived_measurements_but_no_zero_latency_shortcut(source):
    p = profile(prehistory_s=.02); p['rotor_observer']['source'] = source
    p['rpm'].update(sigma=0., bias=[10.]*4)
    f = ArenaSensorFeedback(state(), p, .002, seed=3)
    f.initial_packets(0., state(), np.zeros(17), initial_command=np.full(4, 1000.))
    assert f.rotors.ready and f.rotors.sample_time == pytest.approx(-.004)
    expected = 1010. if source == 'telemetry' else flow(1010., 1000., .004)
    np.testing.assert_allclose(f.state[13:], expected)
    assert len(f.rotors._projector._pending if source == 'telemetry_predictor' else f.rotors._pending) == 2


def test_prehistory_shorter_than_latency_does_not_make_telemetry_available():
    f = ArenaSensorFeedback(state(), profile(prehistory_s=.002), .002)
    f.initial_packets(0., state(), np.zeros(17), initial_command=np.zeros(4))
    assert not f.rotors.ready


def test_prehistory_does_not_consume_flight_or_barometer_random_streams():
    p = load_sensor_profile('configs/sensors/joint_baro_candidate_v6.json')
    a, b = SensorSuite(p, .002, 3), SensorSuite(p, .002, 3)
    list(b.rotor_prehistory(state()[13:], .02))
    assert a.preflight_barometer_bias(20., 100) == b.preflight_barometer_bias(20., 100)
    for t in (0., .002, .01, .1):
        for x, y in zip(a.sample(t, state()), b.sample(t, state())):
            assert (x.kind, x.sample_time, x.arrival_time) == (y.kind, y.sample_time, y.arrival_time)
            np.testing.assert_array_equal(x.value, y.value)


def test_prehistory_requires_declared_past_command_and_only_initializes_once():
    f = ArenaSensorFeedback(state(), profile(prehistory_s=.02), .002)
    with pytest.raises(ValueError): f.initial_packets(0., state(), np.zeros(17))
    f.initial_packets(0., state(), np.zeros(17), initial_command=np.ones(4)*1000.)
    with pytest.raises(ValueError):
        f.initial_packets(0., state(), np.zeros(17), initial_command=np.ones(4)*1000.)


def test_varying_commands_jitter_reordering_and_loss_match_independent_analytic_plant():
    rng = np.random.default_rng(52)
    r = RotorObserver(state(), profile(), .002)
    truth = np.full(4, 1000.)
    r.update(np.zeros(4), [packet(0., .006)], now=0.)
    for k in range(1, 201):
        now = k*.002
        command = rng.uniform(700., 1600., size=4)
        truth = flow(truth, command, .002)
        packets = []
        if k % 3 == 0 and k % 9 != 0:
            delay = (.006, .002, .01, 0.)[(k//3) % 4]
            packets = [Measurement('rpm', now, now+delay, truth, k+1)]
        estimate = r.update(command, packets, now=now)
        if r.sample_time is not None:
            np.testing.assert_allclose(estimate, truth, rtol=0., atol=2e-10)
    assert r._projector.counts['old_samples'] > 0
    assert len(r._projector._history) <= 26
