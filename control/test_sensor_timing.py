"""Physical timing contracts independent of the closed-loop pass criteria."""
import numpy as np
import pytest

from control.arena_estimator import NavigationFilter
from control.arena_feedback import ArenaSensorFeedback, RotorObserver
from control.arena_sensors import Measurement, SensorSuite, load_sensor_profile


def state():
    x = np.zeros(17)
    x[9] = 1.
    x[13:] = 1000.
    return x


def test_rpm_is_unavailable_before_arrival_and_delivered_without_new_sample():
    p = load_sensor_profile('configs/sensors/ideal.json')
    p['rpm'].update(rate_hz=100., latency_s=.006)
    feedback = ArenaSensorFeedback(state(), p, .002)
    feedback.initial_packets(0., state(), np.zeros(17))
    np.testing.assert_array_equal(feedback.state[13:], np.zeros(4))
    for t in (.002, .004):
        feedback.step(t, state(), np.zeros(17), np.zeros(4))
        np.testing.assert_array_equal(feedback.state[13:], np.zeros(4))
    feedback.step(.006, state(), np.zeros(17), np.zeros(4))
    np.testing.assert_array_equal(feedback.state[13:], state()[13:])


def test_predictor_ignores_telemetry():
    p = load_sensor_profile('configs/sensors/ideal.json')
    p['rotor_observer'].update(source='predictor', tau_s=.02)
    feedback = ArenaSensorFeedback(state(), p, .002)
    feedback.initial_packets(0., state(), np.zeros(17))
    np.testing.assert_array_equal(feedback.state[13:], np.zeros(4))
    feedback.step(.002, state(), np.zeros(17), np.full(4, 100.))
    np.testing.assert_allclose(feedback.state[13:], 100.*(1.-np.exp(-.002/.02)))


def test_imu_noise_uses_sensor_period_not_plant_step():
    p = load_sensor_profile('configs/sensors/nominal.json')
    p['imu'].update(rate_hz=250., accel_bias_rw=0., gyro_bias_rw=0.)
    suites = [SensorSuite(p, dt, seed=42) for dt in (.002, .004)]
    for t in np.arange(0., .024, .004):
        values = [next(m.value for m in s.sample(t, state()) if m.kind == 'imu') for s in suites]
        np.testing.assert_array_equal(*values)


def test_imu_noise_density_scales_with_sample_period():
    fast = load_sensor_profile('configs/sensors/nominal.json')
    fast['imu'].update(accel_bias_rw=0., gyro_bias_rw=0.)
    slow = load_sensor_profile(fast)
    slow['imu']['rate_hz'] = 250.
    packets = [next(m.value for m in SensorSuite(p, .002, seed=3).sample(0., state())
                    if m.kind == 'imu') for p in (fast, slow)]
    force = np.array([0., 0., 9.81, 0., 0., 0.])
    np.testing.assert_allclose(packets[0]-force, np.sqrt(2.)*(packets[1]-force), atol=1e-14)


def test_measurement_between_imu_samples_updates_at_its_sample_time():
    p = load_sensor_profile('configs/sensors/ideal.json')
    p['estimator']['estimate_baro_bias'] = False
    x = state()
    x[3] = 2.
    filt = NavigationFilter(x, p)
    filt.advance(0., [Measurement('imu', 0., 0., [0, 0, 9.81, 0, 0, 0], 1)])
    filt.advance(.05, [Measurement('gnss', .05, .05, [.1, 0, 0, 2, 0, 0], 2)])
    assert filt.time == pytest.approx(.05)
    np.testing.assert_allclose(filt.state[:6], [.1, 0, 0, 2, 0, 0], atol=1e-10)


def test_late_arrival_replay_matches_in_order_updates():
    p = load_sensor_profile('configs/sensors/ideal.json')
    p['estimator']['estimate_baro_bias'] = False
    x = state()
    x[3] = 2.
    ordered, delayed = [NavigationFilter(x, p) for _ in range(2)]
    for index, t in enumerate((0., .04, .08, .12)):
        imu = Measurement('imu', t, t, [1, 0, 9.81, 0, 0, 0], index+1)
        ordered.advance(t, [imu])
        delayed.advance(t, [imu])
        if t == .04:
            value = [.15125, 0, 0, 2.05, 0, 0]
            ordered.advance(.05, [Measurement('gnss', .05, .05, value, 20)])
            delayed.advance(.05, [Measurement('gnss', .05, .12, value, 20)])
    np.testing.assert_allclose(ordered.state, delayed.state, atol=1e-12)
    np.testing.assert_allclose(ordered.P, delayed.P, atol=1e-12)


def test_ideal_profile_has_no_hidden_bias_random_walk():
    p = load_sensor_profile('configs/sensors/ideal.json')
    suite = SensorSuite(p, .002, seed=4)
    for t in (0., .002, .004):
        z = next(m.value for m in suite.sample(t, state()) if m.kind == 'imu')
        np.testing.assert_array_equal(z, [0, 0, 9.81, 0, 0, 0])


def test_rotor_filter_uses_measurement_interval_and_rejects_old_samples():
    p = load_sensor_profile('configs/sensors/ideal.json')
    p['rotor_observer']['tau_s'] = .02
    rotor = RotorObserver(state(), p, .002)
    def rpm(sample, arrival, value, sequence):
        return Measurement('rpm', sample, arrival, np.full(4, value), sequence)
    rotor.update(np.zeros(4), [rpm(0., .006, 100., 1)], now=.006)
    result = rotor.update(np.zeros(4), [rpm(.01, .016, 200., 2)], now=.016)
    np.testing.assert_allclose(result, 100.+100.*(1.-np.exp(-.01/.02)))
    unchanged = rotor.update(np.zeros(4), [rpm(.005, .02, 900., 3)], now=.02)
    np.testing.assert_array_equal(result, unchanged)


def test_bias_walk_is_independent_of_plant_step_and_continues_through_loss():
    p = load_sensor_profile('configs/sensors/nominal.json')
    p['imu'].update(rate_hz=250., accel_noise_density=0., gyro_noise_density=0.)
    outage = load_sensor_profile(p)
    outage['imu']['outage_windows'] = [[.004, .012]]
    suites = [SensorSuite(p, .002, seed=8), SensorSuite(p, .004, seed=8),
              SensorSuite(outage, .002, seed=8)]
    for t in (0., .004, .008, .012):
        packets = [[m for m in s.sample(t, state()) if m.kind == 'imu'] for s in suites]
        np.testing.assert_array_equal(packets[0][0].value, packets[1][0].value)
        if t in (0., .012):
            np.testing.assert_array_equal(packets[0][0].value, packets[2][0].value)
        else:
            assert packets[2] == []
