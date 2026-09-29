import json
import numpy as np
import pytest
from control.preflight_calibration_report import build_report, startup_metrics


def fixture():
    t = np.array([0., .02, .12, .14])
    x = np.zeros((4,17)); x[:,2] = 20.; x[:,13:] = 11000.*2*np.pi/60.
    estimate = x.copy(); estimate[:,2] += [0., .1, .2, .2]
    data = dict(ts=t, xs=x, xs_est=estimate, nu_d=np.array([[10.], [9.], [6.]]))
    updates = [dict(sensor='barometer', batch_id=0, baro_bias_after=.01),
               dict(sensor='gnss', batch_id=1, correction_error_state=[0.,0.,.1]+[0.]*12)]
    arrivals = [dict(batch_id=0, baro_bias_before=.01, baro_bias_after=.01),
                dict(batch_id=1, baro_bias_before=.01, baro_bias_after=.01)]
    links = [dict(batch_id=i, processed_at_s=t[i+1], height_state_change_m=h,
                  baro_bias_before=.01, baro_bias_after=.01, next_solve_time_s=t[i+1],
                  thrust_before_next_solve_N=10.-i, thrust_at_next_solve_N=9.-3*i,
                  all_sensors_since_previous_solve=['barometer','gnss']) for i,h in enumerate([.1,.13])]
    return data, dict(updates=updates, arrivals=arrivals), links


def test_replay_metrics_distinguish_anchored_bias_state_jump_and_gnss_injection():
    result = startup_metrics(*fixture())
    assert result['maximum_startup_height_estimation_error_m'] == pytest.approx(.2)
    assert result['maximum_startup_thrust_step_N'] == 3.
    assert result['minimum_startup_actual_rpm'] == pytest.approx(11000.)
    gnss = result['first_gnss_arrival']
    assert gnss['batch_bias_change_m'] == 0.
    assert gnss['batch_height_change_m'] == .13
    assert gnss['sample_time_gnss_height_injection_m'] == .1


def test_startup_window_excludes_later_changes():
    result = startup_metrics(*fixture(), horizon=.05)
    assert result['maximum_startup_height_estimation_error_m'] == pytest.approx(.1)
    assert result['maximum_startup_thrust_step_N'] == 1.
    assert result['maximum_startup_arrival_height_change_m'] == .1


def test_report_rejects_incomplete_campaign(tmp_path):
    source = tmp_path/'experiment.json'; source.write_text(json.dumps({'complete':False, 'records':[]}))
    with pytest.raises(ValueError, match='incomplete'):
        build_report([source], tmp_path/'report')
