from copy import deepcopy
import json
import numpy as np
import pytest

from control.arena import load_config
from control.sensor_binding import runtime_source_hashes
from control.uncertainty import perturb_params
from scripts.sensor_envelope_screen import CONFIG, CONDITIONS, condition_case, condition_config, make_tasks, run


def test_physical_motor_mismatch_keeps_observer_and_controllers_fixed():
    base = load_config(CONFIG)
    before = deepcopy(base)
    for name, factor in [('plant_tau10ms', .5), ('plant_tau40ms', 2.)]:
        config = condition_config(base, name)
        assert config['controllers'] == base['controllers']
        assert config['nmpc_common'] == base['nmpc_common']
        assert config['sensor_feedback']['profile']['rotor_observer'] == base['sensor_feedback']['profile']['rotor_observer']
        scenario = next(s for s in config['scenarios'] if s['id'] == condition_case(name))
        assert scenario['perturbation'] == {'motor_tau': factor}
        # Exercise the production physical-parameter map, not just JSON fields.
        assert perturb_params({'tau_m': .02}, scenario['perturbation'])['tau_m'] == .02*factor
    assert base == before


@pytest.mark.parametrize('condition', ['rpm_outage200ms', 'rpm_latency20ms', 'rpm_rate100hz',
                                      'gnss_outage1s', 'initial_error'])
def test_sensor_faults_cannot_change_aircraft_or_controller(condition):
    base = load_config(CONFIG)
    actual = condition_config(base, condition)
    expected = deepcopy(base)
    actual.pop('sensor_feedback'); expected.pop('sensor_feedback')
    assert actual == expected


def test_truth_is_deduplicated_across_sensor_conditions_and_seeds():
    tasks = make_tasks(load_config(CONFIG), ['V13', 'F13'], [3, 4],
                       ['lateral_high', 'rpm_outage200ms', 'initial_error'])
    truth = {t['trial_id']: t for t in tasks if t['feedback']=='truth'}
    sensors = [t for t in tasks if t['feedback']=='sensors']
    assert len(truth) == 2 and len(sensors) == 12
    for t in sensors:
        paired = truth[t['truth_trial_id']]
        assert paired['controller'] == t['controller']
        assert paired['config']['scenarios'] == t['config']['scenarios']
        assert paired['config']['sensor_feedback'] == dict(schema='sensor_feedback/1', mode='truth', seed=0)
        assert t['config']['sensor_feedback']['seed'] == t['seed']


@pytest.mark.parametrize('seeds', [[1000], [2001], [-1], [True], [], [3, 3]])
def test_seed_guards(seeds):
    with pytest.raises(ValueError):
        make_tasks(load_config(CONFIG), ['V13'], seeds, ['lateral_low'])


def test_failure_retention_partial_resume_and_changed_contract(tmp_path):
    calls = []
    def fake(config, *args, **kwargs):
        calls.append(kwargs['feedback'])
        return dict(records=[dict(feedback=kwargs['feedback'], passed=False,
            tracking_pass=False, model_domain_valid=None, campaign_timed_out=True,
            failure_reasons=['campaign_timeout'])])
    args = (CONFIG, tmp_path, ['V13'], [3], ['lateral_low'])
    doc = run(*args, runner=fake)
    assert doc['complete'] and doc['expected_trials'] == len(doc['records']) == 2
    assert run(*args, runner=fake) == doc
    assert calls == ['truth', 'sensors']
    # Simulate interruption after the truth control, then resume only fusion.
    doc['records'] = doc['records'][:1]; doc['complete'] = False
    (tmp_path/'experiment.json').write_text(json.dumps(doc))
    assert run(*args, runner=fake)['complete']
    assert calls == ['truth', 'sensors', 'sensors']
    with pytest.raises(ValueError, match='contract changed'):
        run(*args, timeout_s=99., runner=fake)


def test_source_provenance_keeps_active_reproduction_valid():
    reference = json.loads((CONFIG.parents[1]/'scripts/data/sensor_reproduction_v13.json').read_text())
    assert reference['runtime_source_sha256'] == runtime_source_hashes()


def test_all_conditions_satisfy_runner_arena_facts_and_truth_case_identity():
    from control.arena import check_confirmed_facts
    base = load_config(CONFIG)
    for condition in CONDITIONS:
        config = condition_config(base, condition)
        assert check_confirmed_facts(config) == []
        assert condition_case(condition) in {s['id'] for s in config['scenarios']}
    tasks = make_tasks(base, ['V13'], [3], ['lateral_low', 'vertical_high'])
    assert len({t['trial_id'] for t in tasks}) == 4


def test_no_trial_output_is_recorded_and_stops_batch(tmp_path):
    def fake(*args, **kwargs):
        return dict(records=[dict(passed=False, failure_reasons=['no_trial_output'])])
    with pytest.raises(RuntimeError, match='inspect campaign.log'):
        run(CONFIG, tmp_path, ['V13'], [3], ['lateral_low'], runner=fake)
    doc = json.loads((tmp_path/'experiment.json').read_text())
    assert not doc['complete'] and len(doc['records']) == 1


def test_outage_removes_only_scheduled_rotor_packets_and_preserves_noise_stream():
    from control.arena_sensors import SensorSuite
    base = load_config(CONFIG)
    profiles = [condition_config(base, c)['sensor_feedback']['profile']
                for c in ('lateral_high', 'rpm_outage200ms')]
    nominal, fault = [SensorSuite(p, .002, seed=3) for p in profiles]
    x = np.zeros(17); x[9] = 1.; x[13:] = 1500.
    missing = 0
    for k in range(1871):
        t = k*.002
        a, b = [{p.kind:p for p in suite.sample(t, x)} for suite in (nominal, fault)]
        if 3.5 <= t < 3.7:
            assert 'rpm' in a and 'rpm' not in b
            missing += 1
        else:
            assert set(a) == set(b)
        for kind, packet in b.items():
            assert packet.sample_time == a[kind].sample_time
            assert packet.arrival_time == a[kind].arrival_time
            np.testing.assert_array_equal(packet.value, a[kind].value)
    assert missing == 100
