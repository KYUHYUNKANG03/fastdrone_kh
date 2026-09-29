import numpy as np

from control.arena_estimator import NavigationFilter
from control.arena_sensors import Measurement, SensorSuite, load_sensor_profile


def state():
    x = np.zeros(17); x[6:10] = [0., 0., 0., 1.]; return x


def test_sensor_seed_reproduces_packets():
    p = load_sensor_profile('configs/sensors/nominal.json')
    x = state(); xd = np.zeros(17)
    a = SensorSuite(p, .002, seed=11); b = SensorSuite(p, .002, seed=11)
    for k in range(30):
        ma = a.sample(k*.002, x, xd); mb = b.sample(k*.002, x, xd)
        assert [(m.kind, m.sample_time, m.arrival_time) for m in ma] == [(m.kind, m.sample_time, m.arrival_time) for m in mb]
        assert all(np.array_equal(u.value, v.value) for u, v in zip(ma, mb))


def test_latency_is_preserved_in_packet_metadata():
    p = load_sensor_profile({'schema': 'sensor/1', 'gnss': {'latency_s': .2}})
    packets = SensorSuite(p, .002, seed=2).sample(0., state(), np.zeros(17))
    gnss = next(m for m in packets if m.kind == 'gnss')
    assert gnss.arrival_time - gnss.sample_time == .2


def test_filter_tracks_static_measurements_without_nan():
    p = load_sensor_profile('configs/sensors/ideal.json')
    x = state(); xd = np.zeros(17)
    sensors = SensorSuite(p, .002, seed=3); filt = NavigationFilter(x, p)
    for k in range(100):
        t = k*.002
        filt.advance(t, sensors.sample(t, x, xd))
    assert np.all(np.isfinite(filt.state))
    assert np.linalg.norm(filt.state[:3] - x[:3]) < .1


def test_delayed_measurement_replays_from_sample_time():
    p = load_sensor_profile('configs/sensors/ideal.json')
    filt = NavigationFilter(state(), p)
    imu0 = Measurement('imu', 0., 0., np.r_[0., 0., 9.81, 0., 0., 0.], 1)
    delayed = Measurement('gnss', 0., .1, np.r_[.01, 0., 0., 0., 0., 0.], 2)
    filt.advance(0., [imu0, delayed])
    imu1 = Measurement('imu', .1, .1, np.r_[0., 0., 9.81, 0., 0., 0.], 3)
    filt.advance(.1, [imu1])
    assert filt.diagnostics['replays'] >= 1
    assert filt.diagnostics['delayed_updates'] >= 1
    assert filt.state[0] > 1e-8


def test_gnss_calibrates_constant_barometer_bias():
    p = load_sensor_profile('configs/sensors/ideal.json')
    p['barometer'].update(enabled=True, rate_hz=50., sigma=0., bias=1., latency_s=0.)
    p['estimator'].update(estimate_baro_bias=True, baro_bias_tau_s=.2,
                          baro_sigma=.05, gnss_pos_sigma=.01, gnss_vel_sigma=.01)
    x = state(); sensors = SensorSuite(p, .002, seed=4); filt = NavigationFilter(x, p)
    for k in range(1001):
        t = k*.002
        filt.advance(t, sensors.sample(t, x, np.zeros(17)))
    assert filt.baro_bias > .8
    assert abs(filt.state[2]) < .15


def test_preflight_barometer_average_reduces_noise():
    p = load_sensor_profile('configs/sensors/stress.json')
    suite = SensorSuite(p, .002, seed=8)
    estimate = suite.preflight_barometer_bias(20., 1000)
    assert abs(estimate-p['barometer']['bias']) < .1


def test_gnss_outage_preserves_later_noise_and_other_sensor_streams():
    base = load_sensor_profile('configs/sensors/nominal.json')
    outage = load_sensor_profile(base)
    outage['gnss']['outage_windows'] = [[.1, .2]]
    clear_suite = SensorSuite(base, .1, seed=19)
    outage_suite = SensorSuite(outage, .1, seed=19)
    clear_at_end = outage_at_end = None
    for t in (0., .1, .2):
        clear_packets = clear_suite.sample(t, state(), np.zeros(17))
        outage_packets = outage_suite.sample(t, state(), np.zeros(17))
        clear_other = {p.kind: p.value for p in clear_packets if p.kind != 'gnss'}
        outage_other = {p.kind: p.value for p in outage_packets if p.kind != 'gnss'}
        assert clear_other.keys() == outage_other.keys()
        for kind in clear_other:
            assert np.array_equal(clear_other[kind], outage_other[kind])
        if t == .2:
            clear_at_end = next(p.value for p in clear_packets if p.kind == 'gnss')
            outage_at_end = next(p.value for p in outage_packets if p.kind == 'gnss')
    assert np.array_equal(clear_at_end, outage_at_end)
