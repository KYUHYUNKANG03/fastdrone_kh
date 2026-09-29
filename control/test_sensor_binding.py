from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from control.arena import load_config, validate_config, config_sha256
from control.arena_sensors import load_sensor_profile, SensorSuite
from control.sensor_binding import resolve_feedback, main_sensor_seeds


def bound_config():
    config = load_config('configs/arena_v2.json')
    config['sensor_feedback'] = dict(schema='sensor_feedback/1', mode='sensors', seed=2001,
                                     profile=load_sensor_profile('configs/sensors/ideal.json'))
    return config


def test_old_config_truth_and_explicit_truth_override():
    assert resolve_feedback(load_config()).metadata == {'feedback': 'truth'}
    assert resolve_feedback(bound_config(), mode='truth').mode == 'truth'


def test_inline_profile_is_hashed_and_resolution_does_not_modify_input():
    config = bound_config(); before = deepcopy(config)
    a = resolve_feedback(config)
    assert config == before and a.seed == 2001
    a.profile['imu']['gyro_bias'][0] = 1.
    assert config == before
    config['sensor_feedback']['profile']['imu']['gyro_noise_density'] = .001
    assert config_sha256(config) != config_sha256(before)
    assert resolve_feedback(config).metadata['sensor_profile_sha256'] != resolve_feedback(before).metadata['sensor_profile_sha256']


@pytest.mark.parametrize('seed', [-1, 2.5, True, '2001'])
def test_invalid_seed_is_rejected(seed):
    with pytest.raises(ValueError, match='seed'):
        resolve_feedback(bound_config(), seed=seed)


def test_external_or_missing_inline_profile_is_rejected():
    c = bound_config(); c['sensor_feedback']['profile'] = 'configs/sensors/ideal.json'
    with pytest.raises(ValueError, match='inline'):
        validate_config(c)


@pytest.mark.parametrize('seeds', [None, [], [2001], [1000, 1000], [999], [True]])
def test_main_seeds_cannot_reuse_training_or_undeclared_range(seeds):
    with pytest.raises(ValueError):
        main_sensor_seeds({'sensor_seeds': seeds}, bound_config())


def test_valid_main_seeds_and_oversampled_profile():
    assert main_sensor_seeds({'sensor_seeds': [1000, 1001]}, bound_config()) == [1000, 1001]
    c = bound_config(); c['sensor_feedback']['profile']['imu']['rate_hz'] = 1000.
    with pytest.raises(ValueError, match='sampling rate'):
        validate_config(c)


def test_vibration_is_gyro_measurement_error_and_preserves_other_noise():
    base = load_sensor_profile('configs/sensors/nominal.json')
    changed = deepcopy(base)
    changed['imu']['gyro_vibration'] = [dict(frequency_hz=50., amplitude_rad_s=[.1, .2, .3], phase_rad=[.2, .3, .4])]
    a, b = SensorSuite(base, .002, 3), SensorSuite(changed, .002, 3)
    x = np.zeros(17); x[9] = 1.; before = x.copy()
    for k in range(30):
        t = k*.002
        for left, right in zip(a.sample(t, x), b.sample(t, x)):
            assert left.kind == right.kind and left.sample_time == right.sample_time
            if left.kind == 'imu':
                np.testing.assert_array_equal(left.value[:3], right.value[:3])
                np.testing.assert_allclose(right.value[3:]-left.value[3:],
                    np.array([.1, .2, .3])*np.sin(2*np.pi*50*t+np.array([.2, .3, .4])), atol=1e-15)
            else:
                np.testing.assert_array_equal(left.value, right.value)
    np.testing.assert_array_equal(x, before)


@pytest.mark.parametrize('section,key,value', [('imu', 'rate_hz', float('nan')),
    ('gnss', 'pos_sigma', -1.), ('barometer', 'latency_s', float('inf')),
    ('estimator', 'preflight_baro_samples', 1.2), ('estimator', 'baro_sigma', 0.)])
def test_invalid_sensor_numbers_are_rejected(section, key, value):
    with pytest.raises(ValueError):
        load_sensor_profile({'schema': 'sensor/1', section: {key: value}})


def test_underresolved_vibration_is_rejected():
    with pytest.raises(ValueError, match='Nyquist'):
        load_sensor_profile({'schema': 'sensor/1', 'imu': {'gyro_vibration': [
            dict(frequency_hz=300., amplitude_rad_s=[1., 0., 0.])]}})


@pytest.fixture(scope='module')
def actual_factory():
    from control.arena_factory import ArenaFactory
    return ArenaFactory(bound_config())


def test_shared_runner_inherits_binding_and_truth_remains_bit_identical(actual_factory):
    from control.validation_suite import run_trial
    from control.validation_metrics import Acceptance
    from control.mission_profiles import GustProfile
    from control.arena_factory import ArenaFactory
    p = GustProfile(cruise_speed=20., settle=.04, gust_duration=.04, recovery=.04)
    case = dict(factors={}, gust_direction='lateral', gust_peak=.1)
    factory = actual_factory
    inherited, a, _ = run_trial(factory, 'CPID', p, case, Acceptance())
    explicit, b, _ = run_trial(factory, 'CPID', p, case, Acceptance(), feedback='sensors',
                              sensor_profile=factory.config['sensor_feedback']['profile'], sensor_seed=2001)
    assert inherited['feedback'] == 'sensors' and inherited['sensor_seed'] == 2001
    assert inherited['trajectory_sha256'] == explicit['trajectory_sha256']
    assert 'xs_est' in a
    old = deepcopy(factory.config); old.pop('sensor_feedback')
    truth, _, _ = run_trial(ArenaFactory(old, factory.p, model=factory.model), 'CPID', p, case, Acceptance())
    overridden, _, _ = run_trial(factory, 'CPID', p, case, Acceptance(), feedback='truth')
    assert truth['trajectory_sha256'] == overridden['trajectory_sha256']


def test_additional_delay_applies_to_sensor_observation(actual_factory, monkeypatch):
    import control.validation_suite as suite
    from control.validation_metrics import Acceptance
    from control.mission_profiles import GustProfile
    p = GustProfile(cruise_speed=20., settle=.04, gust_duration=.04, recovery=.04)
    case = dict(factors={}, gust_direction='lateral', gust_peak=.1,
                extra_params={'state_delay_s': .01, 'rotor_delay_s': .004})
    observations = []
    original = suite.DelayedObservation
    class CaptureDelay(original):
        def observe(self, x):
            delayed = super().observe(x)
            observations.append(delayed.copy())
            return delayed
    monkeypatch.setattr(suite, 'DelayedObservation', CaptureDelay)
    row, result, _ = suite.run_trial(actual_factory, 'CPID', p, case, Acceptance())
    assert row['feedback_delay_s'] == {'state': .01, 'rotor': .004}
    assert len(observations) > 5
    for k, x in enumerate(observations):
        np.testing.assert_array_equal(x[:13], result['xs_est'][max(0, k-5), :13])
        np.testing.assert_array_equal(x[13:], result['xs_est'][max(0, k-2), 13:])


def test_tuning_parallel_inherits_same_sensor_profile_and_seed(actual_factory):
    from control.arena_tune import Evaluator
    c = deepcopy(actual_factory.config)
    c['sensor_feedback']['profile'] = load_sensor_profile('configs/sensors/nominal.json')
    template = next(s for s in c['scenarios'] if s['type'] == 'gust')
    c['tuning']['scenarios'] = [dict(template, id=f'sensor_integration_{i}', speed='V_L',
                                    times_s=[.04, .04, .04]) for i in range(2)]
    records = []
    for workers in (1, 2):
        evaluator = Evaluator(c, actual_factory.p, actual_factory.model, 'CPID', workers)
        try:
            records.append(evaluator({}))
        finally:
            evaluator.close()
    assert records[0] == records[1]
    assert all(row.get('sensor_seed') == 2001 and row.get('sensor_profile_sha256') for row in records[0][1])
    c['sensor_feedback']['seed'] = 2002
    evaluator = Evaluator(c, actual_factory.p, actual_factory.model, 'CPID', 1)
    try:
        changed = evaluator({})
    finally:
        evaluator.close()
    assert changed[1][0]['trajectory_sha256'] != records[0][1][0]['trajectory_sha256']


def test_tuning_record_records_used_sensor_seed(actual_factory, tmp_path):
    import json
    from control.arena_tune import tune_controller, check_tuning_records, check_resume_compatible
    c = deepcopy(actual_factory.config)
    template = next(s for s in c['scenarios'] if s['type'] == 'gust')
    c['tuning']['scenarios'] = [dict(template, id='sensor_record_check', speed='V_L', times_s=[.04]*3)]
    record = tune_controller(c, 'CPID', 1, tmp_path/'tuning', native=actual_factory.p, model=actual_factory.model)
    assert record['seeds']['used'] == [2001] and record['sensor_seed'] == 2001
    assert record['sensor_profile_sha256'] == resolve_feedback(c).metadata['sensor_profile_sha256']
    assert check_tuning_records([record], c) == []
    check_resume_compatible(c, 'CPID', tmp_path/'tuning')
    record['sensor_runtime_source_sha256'] = {'control/changed.py': 'different'}
    (tmp_path/'tuning'/'CPID.record.json').write_text(json.dumps(record))
    assert any('runtime source' in message for message in check_tuning_records([record], c))
    with pytest.raises(RuntimeError, match='runtime source'):
        check_resume_compatible(c, 'CPID', tmp_path/'tuning')


def test_distributed_ids_include_seeds_and_pass_them_to_shared_runner(monkeypatch):
    from control import main_distributed as md
    from control import main_experiment as me
    from control.validation_metrics import Acceptance
    from control.mission_profiles import GustProfile
    import control.validation_suite as suite
    c = bound_config(); profile = GustProfile(cruise_speed=20., settle=.04, gust_duration=.04, recovery=.04)
    s = SimpleNamespace(id='paired', profile=profile, type='gust', cases=[{'factors': {}}], window=[0., .12])
    camp = object.__new__(md.Campaign)
    camp.spec = dict(timing_mac_wall_per_sim_s={'V13': 1.})
    camp.base = c; camp.sensor_seeds = [1000, 1001]
    camp.batches = {'gust': SimpleNamespace(config=c, controllers=['V13'], variant=None)}
    camp.scenarios = lambda name: {'paired': s}
    camp._trim = {'paired': (None, {})}; camp.native = {'n_max': 1.}
    camp.hashes_for = lambda label: {}; camp.factory = lambda name: 'factory'
    monkeypatch.setattr(md, 'excluded_from', lambda *args: None)
    monkeypatch.setattr(me, 'authority_boundary', lambda *args: {})
    import control.validation_metrics as metrics
    monkeypatch.setattr(metrics, 'paper_evaluate', lambda *args, **kwargs: {})
    received = []
    def fake(*args, **kwargs):
        received.append(kwargs['sensor_seed']); return ({'passed': True}, {}, [])
    monkeypatch.setattr(suite, 'run_trial', fake)
    trials = camp.trials()
    assert len(trials) == 2 and len({t['trial_id'] for t in trials}) == 2
    for t in trials:
        row, _ = camp.run_trial(t)
        assert row['sensor_seed'] == t['sensor_seed']
    assert received == [1000, 1001]
