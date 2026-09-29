import json
import numpy as np
import pytest

from control.sensor_filter_report import startup, build_report


def test_startup_counts_commands_using_availability_at_the_command_time(tmp_path):
    directory = tmp_path/'run'
    directory.mkdir()
    states = np.zeros((5, 17))
    states[:, 13:] = 1200.
    commands = np.full((4, 4), 1200.)
    commands[1] = 0.
    np.savez(directory/'trial.npz', ts=np.arange(5)*.002, xs=states, xs_est=states,
             us=commands, indi_rotor_ready=[False, False, True, True],
             indi_path=['fallback_init', 'A1', 'A1', 'A1'])
    result = startup({'run_dir': str(directory)}, tmp_path/'experiment.json')
    assert result['first_rotor_ready_s'] == .004
    assert result['allocation_before_rotor_ready'] == 1
    assert result['all_zero_commands_before_rotor_ready'] == 1
    assert result['minimum_startup_command_rpm'] == 0.
    assert result['minimum_startup_actual_rpm'] == pytest.approx(1200*60/(2*np.pi))


def test_report_rejects_unfinished_campaign(tmp_path):
    source = tmp_path/'experiment.json'
    source.write_text(json.dumps({'complete': False, 'records': []}))
    with pytest.raises(ValueError, match='incomplete experiment'):
        build_report([source], tmp_path/'report')
