import json
import numpy as np
import pytest

from scripts import sensor_envelope_report as report
from scripts.sensor_envelope_screen import CONFIG, run


@pytest.mark.parametrize('truth,sensor,label', [
    (True, True, 'both_pass'), (True, False, 'fusion_only_failure'),
    (False, True, 'truth_only_failure'), (False, False, 'both_fail')])
def test_paired_outcomes_keep_truth_failures_distinct(truth, sensor, label):
    a = dict(passed=truth, simulated_seconds=12.)
    b = dict(passed=sensor, simulated_seconds=3.)
    assert report.paired_outcome(a, b) == label
    b['campaign_timed_out'] = True
    assert report.paired_outcome(a, b) == 'execution_incomplete'


def test_trace_reports_stale_allocations_and_estimation_error(tmp_path):
    path = tmp_path/'trace.npz'
    xs = np.zeros((5, 17)); xs[:, 9] = 1.; xs[:, 13:] = 1000.
    estimates = xs.copy(); estimates[:, 13:] += 10.
    np.savez(path, ts=np.arange(5)*.002, xs=xs, xs_est=estimates,
        indi_rotor_ready=np.array([True, False, False, True]),
        indi_rotor_sample_time_s=[-.004, -.06, -.06, .002],
        indi_path=['A1', 'A1', 'guard', 'A1'])
    result = report.trace_metrics(path, 'sensors')
    assert result['estimation_rms']['rotor_rad_s'] == 20.  # four-axis vector norm
    availability = result['rotor_availability']
    assert availability['allocation_not_ready_ticks'] == 1
    assert availability['not_ready_seconds'] == pytest.approx(.004)
    assert availability['max_sample_age_s'] == pytest.approx(.064)
    assert availability['ready_at_final_command'] is True
    # Truth traces intentionally have no estimated-state requirement.
    assert report.trace_metrics(path, 'truth') == {}


def fake_campaign(*args, **kwargs):
    return dict(records=[dict(feedback=kwargs['feedback'], controller=kwargs['controllers'][0],
        sensor_seed=kwargs['seed'], passed=False, simulated_seconds=None, run_dir=None,
        campaign_timed_out=True, failure_reasons=['campaign_timeout'])])


def test_report_keeps_timeouts_and_rejects_altered_pairing(tmp_path, monkeypatch):
    source = tmp_path/'screen'
    doc = run(CONFIG, source, ['V13'], [3], ['lateral_low'], runner=fake_campaign)
    monkeypatch.setattr(report, 'relative', lambda p: str(p))
    monkeypatch.setattr(report, 'write_report', lambda *args: None)
    result = report.summarize([source/'experiment.json'], tmp_path/'report')
    assert len(result['records']) == 2
    assert result['records'][1]['paired_outcome'] == 'execution_incomplete'
    doc['records'][1]['truth_trial_id'] = 'unrelated_truth'
    (source/'experiment.json').write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='pairing differs'):
        report.summarize([source/'experiment.json'], tmp_path/'report')


def test_report_refuses_incomplete_and_mismatched_inputs(tmp_path, monkeypatch):
    source = tmp_path/'screen'
    doc = run(CONFIG, source, ['V13'], [3], ['lateral_low'], runner=fake_campaign)
    monkeypatch.setattr(report, 'relative', lambda p: str(p))
    doc['complete'] = False
    path = source/'experiment.json'
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='incomplete'):
        report.summarize([path], tmp_path/'report')
    doc['complete'] = True
    doc['records'][0]['input_config_sha256'] = 'altered'
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='input configuration'):
        report.summarize([path], tmp_path/'report')


def test_manifest_profile_path_and_record_profile_name_are_distinct():
    from control.arena import load_config
    from control.sensor_binding import resolve_feedback
    config = load_config(CONFIG)
    record = resolve_feedback(config).metadata
    manifest = dict(record, sensor_profile='/some/run/profiles/candidate.json',
                    sensor_profile_config=config['sensor_feedback']['profile'])
    assert report.verify_feedback_binding(config, manifest, record).mode == 'sensors'
    manifest['sensor_profile_sha256'] = 'wrong'
    with pytest.raises(ValueError, match='feedback binding'):
        report.verify_feedback_binding(config, manifest, record)


def test_duplicate_sources_and_changed_driver_are_rejected(tmp_path, monkeypatch):
    source = tmp_path/'screen'
    doc = run(CONFIG, source, ['V13'], [3], ['lateral_low'], runner=fake_campaign)
    path = source/'experiment.json'
    monkeypatch.setattr(report, 'relative', lambda p: str(p))
    with pytest.raises(ValueError, match='duplicate trial across sources'):
        report.summarize([path, path], tmp_path/'report')
    doc['contract']['driver_sha256'] = 'old-driver'
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='driver differs'):
        report.summarize([path], tmp_path/'report')


@pytest.mark.parametrize('mode,reason', [
    ('empty', 'no_executed_steps'), ('nonfinite', 'nonfinite_xs'),
    ('quaternion', 'invalid_truth_quaternion'), ('estimate', 'nonfinite_estimated_states')])
def test_failed_trace_metrics_are_explicitly_unavailable(tmp_path, mode, reason):
    xs = np.zeros((1 if mode == 'empty' else 3, 17)); xs[:, 9] = 1.
    estimate = xs.copy()
    if mode == 'nonfinite':
        xs[-1, 0] = np.nan
    if mode == 'quaternion':
        xs[-1, 9] = 0.
    if mode == 'estimate':
        estimate[-1, 0] = np.inf
    path = tmp_path/'trace.npz'
    np.savez(path, xs=xs, xs_est=estimate, ts=np.arange(len(xs))*.002)
    result = report.trace_metrics(path, 'sensors')
    assert result == {'diagnostics_unavailable': reason}
    json.dumps(result, allow_nan=False)


def test_corrupt_shapes_still_raise_instead_of_hiding_diagnostics(tmp_path):
    path = tmp_path/'trace.npz'
    np.savez(path, ts=np.arange(3), xs=np.zeros((2, 17)))
    with pytest.raises(ValueError, match='malformed'):
        report.trace_metrics(path, 'truth')
