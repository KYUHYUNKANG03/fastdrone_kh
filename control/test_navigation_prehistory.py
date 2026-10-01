"""D5 common navigation prehistory (estimator.navigation_prehistory_s).

Pre-registered checks (SENSOR_DECISIONS 'D5 결정' 검증 2): (a) flight packets
unchanged, (b) no reset to truth at zero, (c) the filter sees packets only,
(d) determinism, (e) profile hash unchanged without the field, (f) packets in
transit at zero stay queued, (g) reverse: other prehistory noise changes the
handover estimate. (h) extra: sampling grid, noise-free formulas, guards.
"""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from control.arena import load_config
from control.arena_estimator import NavigationFilter
from control.arena_feedback import ArenaSensorFeedback
from control.arena_joint_estimator import JointNavigationFilter
from control.arena_sensors import SensorSuite, load_sensor_profile
from control.sensor_binding import FeedbackBinding, resolve_feedback
from control.sensor_matching_screen import sensor_group_profile

DT = .002
V9 = 'configs/arena_rotor_projected_development_v9.json'
# Computed with the code before this change (8bc6b4e) and equal to the
# sensor_profile_sha256 recorded by the E1 isolation trials (2026-10-01).
V9_PROFILE_SHA256 = '1091868104c933deebc1abe04f7f64729a84450066eeab33dd82aea1a8779701'
V9_NAVIGATION_ONLY_SHA256 = 'ebf1c5c1136e94b8a257625577f5bba6dba5766e80aa70769ee29288d9606d82'


def v9_profile(prehistory=None):
    profile = deepcopy(resolve_feedback(load_config(V9)).profile)
    if prehistory is not None:
        profile['estimator']['navigation_prehistory_s'] = prehistory
    return load_sensor_profile(profile)


def trim_state():
    """Synthetic steady cruise: tilted attitude, 20 m/s, zero acceleration."""
    x = np.zeros(17)
    x[0:3] = [3., -2., 20.]
    x[3:6] = [20., 1., 0.]
    x[6:10] = Rotation.from_euler('xyz', [.02, -.1, .3]).as_quat()
    x[13:17] = 1200.
    return x


def truth_at(x0, t):
    x = x0.copy()
    x[0:3] = x0[0:3]+x0[3:6]*t
    return x


def started(profile, seed=3, x0=None):
    x0 = trim_state() if x0 is None else x0
    feedback = ArenaSensorFeedback(x0, profile, DT, seed=seed)
    packets = feedback.initial_packets(0., x0, np.zeros(17), initial_command=x0[13:17])
    return feedback, packets


def fly(feedback, x0, steps):
    """Advance the flight part; returns every flight packet in order."""
    out = list(feedback._last_packets)
    for k in range(1, steps+1):
        feedback.step(k*DT, truth_at(x0, k*DT), np.zeros(17), x0[13:17])
        out.extend(feedback._last_packets)
    return out


def same_packets(left, right):
    assert len(left) == len(right)
    for a, b in zip(left, right):
        assert (a.kind, a.sample_time, a.arrival_time, a.sequence) == (b.kind, b.sample_time, b.arrival_time, b.sequence)
        np.testing.assert_array_equal(a.value, b.value)


def stress_profile(prehistory=None):
    """Dropouts, bias walks, a vibration tone, baro drift and the shared preflight stream."""
    p = {'schema': 'sensor/1',
         'imu': {'dropout_prob': .05, 'accel_bias': [.01, -.02, .03], 'gyro_bias': [.001, 0., -.001],
                 'gyro_vibration': [{'frequency_hz': 80., 'amplitude_rad_s': [.01, .02, .03]}]},
         'gnss': {'dropout_prob': .1}, 'barometer': {'dropout_prob': .05, 'bias_rate_m_s': .01},
         'magnetometer': {'dropout_prob': .05},
         'estimator': {'preflight_baro_samples': 20, 'preflight_baro_rng': 'shared'}}
    if prehistory is not None:
        p['estimator']['navigation_prehistory_s'] = prehistory
    return load_sensor_profile(p)


# (a) ---------------------------------------------------------------------------
@pytest.mark.parametrize('make', [v9_profile, stress_profile])
def test_a_flight_packets_and_calibration_identical_with_prehistory(make):
    x0 = trim_state()
    off, first_off = started(make())
    on, first_on = started(make(1.))
    same_packets(first_off, first_on)
    same_packets(fly(off, x0, 500), fly(on, x0, 500))


def test_a_existing_seven_streams_are_the_same_children():
    suite = SensorSuite(v9_profile(), DT, seed=3)
    old = np.random.SeedSequence(3).spawn(7)
    reference = [np.random.default_rng(s) for s in old]
    actual = [suite._rng[n] for n in ('imu', 'gnss', 'barometer', 'magnetometer', 'rpm')]
    actual += [suite._preflight_rng, suite._rotor_prehistory_rng]
    for ref, got in zip(reference, actual):
        np.testing.assert_array_equal(ref.normal(size=8), got.normal(size=8))


def test_a_preflight_calibration_unchanged_by_prehistory():
    for make in (v9_profile, stress_profile):
        assert (ArenaSensorFeedback(trim_state(), make(), DT, seed=3).filter.baro_bias
                == ArenaSensorFeedback(trim_state(), make(2.), DT, seed=3).filter.baro_bias)


# (b) ---------------------------------------------------------------------------
def test_b_handover_estimate_is_not_reset_to_truth():
    x0 = trim_state()
    f, _ = started(v9_profile(1.))
    assert np.linalg.norm(f.state[0:3]-x0[0:3]) > 1e-3
    assert np.linalg.norm(f.state[3:6]-x0[3:6]) > 1e-4
    p0 = .5**2
    assert np.all(np.diag(f.filter.P)[:3] < .5*p0)       # converged, not the declared P0


# (c) ---------------------------------------------------------------------------
def test_c_filter_state_is_reproduced_from_the_packets_alone():
    x0, profile, duration = trim_state(), v9_profile(1.), 1.
    f, _ = started(profile)
    suite = SensorSuite(profile, DT, seed=3)
    start = x0.copy(); start[0:3] -= duration*x0[3:6]
    filt = JointNavigationFilter(start, profile)
    filt.set_initial_baro_bias(suite.preflight_barometer_bias(start[2], 100), time=-duration)
    for tick, packets in suite.navigation_prehistory(x0, np.zeros(17), duration):
        filt.advance(tick, packets)
    list(suite.rotor_prehistory(x0[13:17], .1))       # same flight sequence numbers as initial_packets
    filt.advance(0., [m for m in suite.sample(0., x0, np.zeros(17)) if m.kind != 'rpm'])
    for name in ('p', 'v', 'q', 'ba', 'bg', 'P'):
        np.testing.assert_array_equal(getattr(f.filter, name), getattr(filt, name))
    assert f.filter.baro_bias == filt.baro_bias


# (d) ---------------------------------------------------------------------------
def test_d_same_seed_is_deterministic():
    x0 = trim_state()
    a, first_a = started(v9_profile(1.))
    b, first_b = started(v9_profile(1.))
    same_packets(first_a, first_b)
    np.testing.assert_array_equal(a.filter.P, b.filter.P)
    same_packets(fly(a, x0, 100), fly(b, x0, 100))
    np.testing.assert_array_equal(a.state, b.state)
    c, _ = started(v9_profile(1.), seed=4)
    assert not np.array_equal(a.filter.p, c.filter.p)


# (e) ---------------------------------------------------------------------------
def profile_hash(profile):
    return FeedbackBinding('sensors', profile, 3).metadata['sensor_profile_sha256']


def test_e_profile_hash_unchanged_without_the_field():
    assert 'navigation_prehistory_s' not in v9_profile()['estimator']
    assert profile_hash(v9_profile()) == V9_PROFILE_SHA256
    navigation = dict(sensor_group_profile(v9_profile(), 'navigation_only'), name='navigation_only_nominal')
    assert profile_hash(load_sensor_profile(navigation)) == V9_NAVIGATION_ONLY_SHA256


def test_e_field_is_covered_by_the_hash():
    assert profile_hash(v9_profile(3.)) != V9_PROFILE_SHA256
    assert profile_hash(v9_profile(0.)) != V9_PROFILE_SHA256     # explicit zero is a different contract
    assert profile_hash(v9_profile(3.)) != profile_hash(v9_profile(5.))


# (f) ---------------------------------------------------------------------------
def test_f_packets_in_transit_at_zero_stay_queued_then_apply_at_their_sample_time():
    x0 = trim_state()
    f, _ = started(v9_profile(1.))
    queued = [m for m in f.filter._pending if m.sample_time < 0.]
    gnss = [m for m in queued if m.kind == 'gnss']
    assert gnss and all(m.arrival_time > 0. for m in queued)
    late = max(gnss, key=lambda m: m.sample_time)
    assert late.sample_time == pytest.approx(-.1) and late.arrival_time == pytest.approx(.02)
    assert late.sequence not in f.filter._seen
    assert f.diagnostics['navigation_prehistory']['in_transit_at_zero'] == len(queued)
    fly(f, x0, 10)                                       # to t = 0.02 s
    assert late.sequence in f.filter._seen
    assert any(r['packet'] is late for r in f.filter._events)


# (g) ---------------------------------------------------------------------------
@pytest.mark.parametrize('stream', ['gnss', 'barometer', 'imu'])
def test_g_other_prehistory_noise_changes_handover_but_not_flight_packets(stream):
    x0 = trim_state()
    base, first_base = started(v9_profile(1.))
    other = ArenaSensorFeedback(x0, v9_profile(1.), DT, seed=3)
    other.sensors._navigation_prehistory_rng[stream] = np.random.default_rng(12345)
    first_other = other.initial_packets(0., x0, np.zeros(17), initial_command=x0[13:17])
    assert not np.array_equal(base.state[:10], other.state[:10])
    same_packets(first_base, first_other)
    same_packets(fly(base, x0, 50), fly(other, x0, 50))


# (h) ---------------------------------------------------------------------------
def test_h_sampling_grid_counts_times_and_sequences():
    x0 = trim_state()
    suite = SensorSuite(v9_profile(), DT, seed=3)
    ticks = list(suite.navigation_prehistory(x0, np.zeros(17), 1.))
    assert len(ticks) == 500 and ticks[0][0] == pytest.approx(-1.) and ticks[-1][0] == pytest.approx(-DT)
    packets = [m for _, group in ticks for m in group]
    counts = {k: sum(m.kind == k for m in packets) for k in ('imu', 'gnss', 'barometer', 'magnetometer')}
    assert counts == {'imu': 500, 'gnss': 10, 'barometer': 50, 'magnetometer': 100}
    assert all(m.sample_time < 0. for m in packets)
    assert min(m.sample_time for m in packets if m.kind == 'gnss') == pytest.approx(-1.)
    sequences = [m.sequence for m in packets]
    assert sequences == sorted(sequences) and sequences[-1] == -1 and len(set(sequences)) == len(packets)
    assert suite._seq == 0 and all(v == 0 for v in suite._next.values())


def quiet_profile(prehistory=None):
    profile = dict(sensor_group_profile(v9_profile(), 'quiet_sampled'), name='quiet')
    if prehistory is not None:
        profile['estimator'] = dict(profile['estimator'], navigation_prehistory_s=prehistory)
    return load_sensor_profile(profile)


def test_h_noise_free_truth_kinematics_and_handover():
    """Sign check: prehistory GNSS = p0 + v0 t, and a noise-free handover equals truth."""
    x0 = trim_state()
    suite = SensorSuite(quiet_profile(), DT, seed=3)
    for t, group in suite.navigation_prehistory(x0, np.zeros(17), 1.):
        for m in group:
            if m.kind == 'gnss':
                np.testing.assert_allclose(m.value, np.r_[x0[0:3]+x0[3:6]*t, x0[3:6]], atol=1e-12)
    f, _ = started(quiet_profile(1.))
    np.testing.assert_allclose(f.state[0:6], x0[0:6], atol=1e-6)
    assert abs(abs(np.dot(f.state[6:10], x0[6:10]))-1.) < 1e-9


def test_h_noise_free_values_follow_the_flight_formulas():
    x0, t = trim_state(), -.5
    p = load_sensor_profile({'schema': 'sensor/1',
        'imu': {'accel_noise_density': 0., 'gyro_noise_density': 0., 'accel_bias_rw': 0., 'gyro_bias_rw': 0.,
                'accel_bias': [.01, -.02, .03], 'gyro_bias': [.001, 0., -.001],
                'gyro_vibration': [{'frequency_hz': 80., 'amplitude_rad_s': [.01, .02, .03],
                                    'phase_rad': [0., 1., 2.]}]},
        'gnss': {'pos_sigma': 0., 'vel_sigma': 0., 'pos_bias': [.1, .2, .3], 'vel_bias': [.01, .02, .03]},
        'barometer': {'sigma': 0., 'bias': .4, 'bias_rate_m_s': .05},
        'magnetometer': {'sigma': 0., 'bias': [.5, .6, .7]}})
    packets = {m.kind: m for tick, group in SensorSuite(p, DT, seed=1).navigation_prehistory(
        x0, np.zeros(17), 1.) if tick == pytest.approx(t) for m in group}
    R = Rotation.from_quat(x0[6:10]).as_matrix()
    tone = np.array([.01, .02, .03])*np.sin(2.*np.pi*80.*packets['imu'].sample_time+np.array([0., 1., 2.]))
    np.testing.assert_allclose(packets['imu'].value, np.r_[R.T@[0., 0., 9.81]+[.01, -.02, .03],
                                                           np.array([.001, 0., -.001])+tone], atol=1e-12)
    position = x0[0:3]+x0[3:6]*packets['gnss'].sample_time
    np.testing.assert_allclose(packets['gnss'].value, np.r_[position+[.1, .2, .3], x0[3:6]+[.01, .02, .03]],
                               atol=1e-12)
    np.testing.assert_allclose(packets['barometer'].value, [position[2]+.4+.05*packets['barometer'].sample_time],
                               atol=1e-12)
    np.testing.assert_allclose(packets['magnetometer'].value, R.T@[20., 0., 40.]+[.5, .6, .7], atol=1e-12)
    for m in packets.values():
        assert m.arrival_time == pytest.approx(m.sample_time+p[m.kind]['latency_s'])


def test_h_bias_walk_ends_at_the_configured_bias():
    x0 = trim_state()
    p = load_sensor_profile({'schema': 'sensor/1', 'imu': {
        'accel_noise_density': 0., 'gyro_noise_density': 0., 'accel_bias_rw': .1, 'gyro_bias_rw': .1,
        'accel_bias': [.5, .5, .5], 'gyro_bias': [.2, .2, .2]}})
    imu = [m for _, g in SensorSuite(p, DT, seed=2).navigation_prehistory(x0, np.zeros(17), 2.)
           for m in g if m.kind == 'imu']
    f_b = Rotation.from_quat(x0[6:10]).as_matrix().T@[0., 0., 9.81]
    last, first = imu[-1].value-np.r_[f_b, x0[10:13]], imu[0].value-np.r_[f_b, x0[10:13]]
    # One step from the configured bias at -dt, a two-second walk at -T.
    assert np.all(np.abs(last-np.r_[[.5]*3, [.2]*3]) < 6*.1*np.sqrt(DT))
    assert np.max(np.abs(first-np.r_[[.5]*3, [.2]*3])) > 6*.1*np.sqrt(DT)


@pytest.mark.parametrize('bad', [-1., float('nan'), float('inf'), True, '1'])
def test_h_invalid_field_rejected(bad):
    p = v9_profile()
    p['estimator']['navigation_prehistory_s'] = bad
    with pytest.raises(ValueError):
        load_sensor_profile(p)


def test_h_duration_must_be_a_plant_step_multiple():
    with pytest.raises(ValueError):
        started(v9_profile(.003))


def test_h_guards():
    x0 = trim_state()
    f = ArenaSensorFeedback(x0, v9_profile(1.), DT, seed=3)
    with pytest.raises(ValueError):
        f.step(DT, x0, np.zeros(17), x0[13:17])
    with pytest.raises(ValueError):
        f.initial_packets(DT, x0, np.zeros(17), initial_command=x0[13:17])
    moved = x0.copy(); moved[0] += 1.
    with pytest.raises(ValueError):
        f.initial_packets(0., moved, np.zeros(17), initial_command=x0[13:17])
    f.initial_packets(0., x0, np.zeros(17), initial_command=x0[13:17])
    with pytest.raises(ValueError):
        f.initial_packets(0., x0, np.zeros(17), initial_command=x0[13:17])


def test_h_explicit_zero_is_the_old_path():
    x0 = trim_state()
    a, first_a = started(v9_profile())
    b, first_b = started(v9_profile(0.))
    same_packets(first_a, first_b)
    np.testing.assert_array_equal(a.filter.P, b.filter.P)
    np.testing.assert_array_equal(a.state, b.state)
    assert 'navigation_prehistory' not in b.diagnostics


@pytest.mark.parametrize('duration', [None, 1.])
def test_h_legacy_calibration_time_is_the_prehistory_start(duration):
    p = {'schema': 'sensor/1', 'estimator': {'preflight_baro_samples': 10}}
    if duration is not None:
        p['estimator']['navigation_prehistory_s'] = duration
    f = ArenaSensorFeedback(trim_state(), load_sensor_profile(p), DT, seed=3)
    assert isinstance(f.filter, NavigationFilter) and not isinstance(f.filter, JointNavigationFilter)
    expected = 0. if duration is None else -duration
    assert f.filter._baro_bias_time == expected
    assert f.filter._initial_snapshot['baro_bias_time'] == expected


def test_h_start_state_is_minus_T_on_the_cruise_and_diagnostics_report_handover():
    x0 = trim_state()
    f = ArenaSensorFeedback(x0, v9_profile(2.), DT, seed=3)
    np.testing.assert_array_equal(f.filter.p, x0[0:3]-2.*x0[3:6])
    f.initial_packets(0., x0, np.zeros(17), initial_command=x0[13:17])
    record = f.diagnostics['navigation_prehistory']
    assert record['duration_s'] == 2. and record['counts']['gnss'] == 20
    np.testing.assert_allclose(record['sigma_position_m'], np.sqrt(np.diag(f.filter.P)[0:3]))
