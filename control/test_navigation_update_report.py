import json
import numpy as np
import pytest

from control.navigation_update_report import build_report, link_arrivals, summarize_updates


def test_arrivals_link_forward_to_actual_solver_and_include_other_sensor_updates():
    trace = dict(ts=np.arange(7)*.01, us=np.zeros((6, 4)),
                 nu_d=np.column_stack(([10., 10., 5., 5., 8., 8.], np.zeros((6, 3)))))
    updates = [dict(sequence=1, sensor='barometer', processed_at_s=.01),
               dict(sequence=2, sensor='magnetometer', processed_at_s=.015),
               dict(sequence=3, sensor='gnss', processed_at_s=.04),
               dict(sequence=4, sensor='barometer', processed_at_s=.06)]
    batches = [dict(batch_id=i, processed_at_s=r['processed_at_s'], sequences=[r['sequence']],
                    state_before=[0.]*13, state_after=[0.]*13) for i, r in enumerate(updates)]
    links = link_arrivals(trace, dict(updates=updates, arrivals=batches),
                          [dict(t=0.), dict(t=.02), dict(t=.04)])
    assert links[0]['control_time_s'] == .01
    assert links[0]['next_solve_time_s'] == .02
    assert links[0]['thrust_before_next_solve_N'] == 10.
    assert links[0]['thrust_at_next_solve_N'] == 5.
    assert links[0]['all_sensors_since_previous_solve'] == ['barometer', 'magnetometer']
    assert links[2]['next_solve_time_s'] == .04
    assert links[-1]['control_time_s'] is None
    assert links[-1]['next_solve_time_s'] is None
    assert links[-1]['thrust_at_next_solve_N'] is None


def test_update_statistics_keep_rejections_and_stale_packets():
    base = dict(sensor='gnss', processed_at_s=.1, nis=2., correction_error_state=[0.]*15)
    data = dict(updates=[dict(base, status='accepted'), dict(base, status='rejected'),
                         dict(sensor='gnss', processed_at_s=.2, status='stale')])
    result = summarize_updates(data)['gnss']
    assert result['packets'] == 3
    assert result['accepted'] == result['rejected'] == result['stale'] == 1
    assert result['mean_nis'] == 2.


def test_report_requires_completed_campaign(tmp_path):
    source = tmp_path/'experiment.json'
    source.write_text(json.dumps({'complete': False, 'records': []}))
    with pytest.raises(ValueError, match='incomplete campaign'):
        build_report([source], tmp_path/'output')
