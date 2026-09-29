from copy import deepcopy
import pytest
from control.arena import load_config
from scripts.sensor_rotor_screen import condition_config, run_screen


@pytest.mark.parametrize('condition,latency,tau', [
    ('rotor_unfiltered', .004, 1e-6), ('rotor_no_latency', 0., .02),
    ('rotor_direct', 0., 1e-6)])
def test_timing_conditions_only_change_declared_fields(condition, latency, tau):
    base = load_config('configs/arena_sensor_candidate_v6.json'); before = deepcopy(base)
    actual = condition_config(base, condition)
    expected = deepcopy(base)
    profile = expected['sensor_feedback']['profile']
    profile['name'] = condition
    profile['rpm']['latency_s'] = latency
    profile['rotor_observer']['tau_s'] = tau
    assert actual == expected and base == before


def test_contract_checks_conditions_and_resume(tmp_path, monkeypatch):
    from scripts import sensor_rotor_screen as screen
    calls = []
    monkeypatch.setattr(screen, 'run', lambda *args: calls.append(args))
    args = ('configs/arena_sensor_candidate_v6.json', tmp_path, ['V13'], [3],
            ['rotor_direct'], ['gust_lateral_p10_VH'])
    run_screen(*args); run_screen(*args)
    assert len(calls) == 2  # Nested matching runner owns completed-trial resume.
    with pytest.raises(ValueError, match='contract changed'):
        run_screen(*args[:-2], ['rotor_unfiltered'], args[-1])
    with pytest.raises(ValueError, match='unknown rotor'):
        condition_config(load_config(args[0]), 'unknown')
