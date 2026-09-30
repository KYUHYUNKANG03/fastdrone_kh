from copy import deepcopy
import pytest
from control.arena import load_config
from control.sensor_matching_screen import sensor_group_profile
from scripts import sensor_navigation_screen as screen


@pytest.mark.parametrize('sensor', screen.SENSORS)
def test_restores_one_measurement_block_with_filter_and_controller_fixed(sensor):
    base = load_config('configs/arena_rotor_projected_development_v9.json')
    before = deepcopy(base)
    config = screen.condition_config(base, sensor)
    expected = sensor_group_profile(base['sensor_feedback']['profile'], 'quiet_sampled')
    expected[sensor] = deepcopy(base['sensor_feedback']['profile'][sensor])
    expected['name'] = 'nav_startup_'+sensor+'_only'
    assert config['sensor_feedback']['profile'] == expected
    assert base == before
    config.pop('sensor_feedback'); before.pop('sensor_feedback')
    assert config == before


def test_resume_contract_cannot_change_sensor_axis(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(screen, 'run', lambda *args: calls.append(args))
    args = ('configs/arena_rotor_projected_development_v9.json', tmp_path,
            ['V13'], [3], ['gnss'], ['gust_lateral_p10_VL'])
    screen.run_screen(*args); screen.run_screen(*args)
    assert len(calls) == 2  # Inner runner resumes completed records.
    with pytest.raises(ValueError, match='contract changed'):
        screen.run_screen(*args[:4], ['barometer'], args[-1])
