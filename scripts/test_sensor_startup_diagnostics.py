import numpy as np
import pytest

from scripts.sensor_startup_diagnostics import grouped_counts, nominal_repeats, startup_metrics


def test_startup_window_and_historical_injection_are_distinct():
    truth = np.zeros((4, 17)); truth[:, 9] = 1.; truth[:, 13:] = 11000*2*np.pi/60
    estimate = truth.copy(); estimate[1, :3] = [3., 4., 0.]
    estimate[2, 3:6] = [0., 0., .2]; estimate[3, 0] = 100.
    truth[1, 13] = 9000*2*np.pi/60
    injection = [0.]*16; injection[2] = .1; injection[3] = .15
    update = dict(sensor='gnss', status='accepted', sample_time_s=0.,
                  processed_at_s=.12, correction_error_state=injection, nis=1.)
    nav = dict(updates=[update])
    result = startup_metrics(dict(ts=np.array([0., .12, .5, .6]), xs=truth, xs_est=estimate), nav)
    assert result['peak_position_error_m'] == 5.  # the later 100 m error is outside this window
    assert result['peak_velocity_error_m_s'] == .2
    assert result['minimum_actual_rotor_rpm'] == pytest.approx(9000.)
    assert result['max_gnss_position_injection_m'] == .1  # not the arrival-state error
    assert result['max_gnss_velocity_injection_m_s'] == .15
    assert result['first_gnss_update']['sample_time_s'] == 0.
    assert result['first_gnss_update']['processed_at_s'] == .12


def test_rejected_packet_is_not_reported_as_an_injection():
    xs = np.zeros((2, 17)); xs[:, 9] = 1.
    result = startup_metrics(dict(ts=np.array([0., .002]), xs=xs, xs_est=xs.copy()),
        dict(updates=[dict(sensor='gnss', status='rejected', processed_at_s=0., nis=300.)]))
    assert result['max_gnss_position_injection_m'] is None
    assert result['recorded_end_s'] == .002
    assert result['update_statistics']['gnss']['rejected'] == 1


def test_execution_failure_and_shared_trajectories_do_not_inflate_success_counts():
    common = dict(stage='gnss_startup',controller='V13',case='low',condition='quiet',
                  execution_status='recorded',passed=True,tracking_pass=True,model_domain_valid=True,
                  trajectory_sha256='same')
    rows = [dict(common), dict(common), dict(common, execution_status='execution_incomplete'),
            dict(common, execution_status='pending', trajectory_sha256=None),
            dict(common,passed=False,model_domain_valid=False,stop_reason='solver',trajectory_sha256='failure')]
    group, = grouped_counts(rows)
    assert group['planned'] == 5 and group['recorded'] == 3
    assert group['tracking_pass'] == 3 and group['full_pass'] == 2
    assert group['pending'] == 1 and group['execution_incomplete'] == 1
    assert group['early_stops'] == 1 and group['unique_trajectories'] == 2


def test_domain_only_failure_is_visible_outside_legacy_tuning_penalty():
    common = dict(stage='integration',controller='V13',case='low',condition='nominal',
        execution_status='recorded',passed=False,tracking_pass=True,model_domain_valid=False,
        paper_failed=False,stop_reason=None)
    rows = [common, dict(common,stop_reason='solver'), dict(common,paper_failed=True),
            dict(common,execution_status='execution_incomplete'), dict(common,paper_failed=None)]
    group, = grouped_counts(rows)
    assert group['domain_failures_not_penalized_by_legacy_rule'] == 1


def test_nominal_repeats_require_a_recorded_trajectory_and_matching_verdicts():
    common = dict(controller='V13', case='low', seed=3, execution_status='recorded',
                  trajectory_sha256='same', passed=False, model_domain_valid=False)
    original = dict(common, trial_id='serial', stage='integration', condition='nominal')
    repeat = dict(common, trial_id='logged', stage='gnss_startup', condition='gnss_full_noise1_delay1')
    check, = nominal_repeats([original, repeat])
    assert check['comparison_available'] and check['trajectory_bit_identical'] and check['verdicts_identical']
    check, = nominal_repeats([original, dict(repeat, passed=True)])
    assert check['trajectory_bit_identical'] and not check['verdicts_identical']
    check, = nominal_repeats([original, dict(repeat, trajectory_sha256='different')])
    assert not check['trajectory_bit_identical']
    check, = nominal_repeats([original, dict(repeat, execution_status='execution_incomplete')])
    assert not check['comparison_available'] and check['trajectory_bit_identical'] is None
