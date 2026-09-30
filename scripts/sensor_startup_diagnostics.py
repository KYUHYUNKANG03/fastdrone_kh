"""Analyze the prepared GNSS factorial after provenance-checked development runs.

First-processing corrections are evaluated at the packet's historical sample
time. They are timing evidence, not additive attribution of controller effort.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from control.navigation_update_report import summarize_updates
from scripts.sensor_preparation_report import summarize

PATTERN = re.compile(r'gnss_(isolated|full)_noise([01])_delay([01])$')


def startup_metrics(trace, navigation, until_s=.5):
    """Keep error magnitudes and packet corrections distinct; no verdict edits."""
    ts, truth, estimate = trace['ts'], trace['xs'], trace['xs_est']
    n = min(len(ts), len(estimate))
    early = ts[:n] <= until_s+1e-9
    error = estimate[:n]-truth[:n]
    angles = (Rotation.from_quat(truth[:n, 6:10]).inv()*
              Rotation.from_quat(estimate[:n, 6:10])).magnitude()
    updates = [u for u in navigation['updates'] if u['sensor'] == 'gnss']
    early_updates = [u for u in updates if u['processed_at_s'] <= until_s+1e-9]
    injections = [u['correction_error_state'] for u in early_updates
                  if u['status'] == 'accepted' and 'correction_error_state' in u]
    first = updates[0] if updates else None
    def peak(values):
        return float(np.max(values[early])) if early.any() else None
    return dict(window_end_s=until_s, recorded_end_s=float(ts[-1]),
        peak_position_error_m=peak(np.linalg.norm(error[:, :3], axis=1)),
        peak_velocity_error_m_s=peak(np.linalg.norm(error[:, 3:6], axis=1)),
        peak_attitude_error_deg=peak(np.rad2deg(angles)),
        peak_height_error_m=peak(np.abs(error[:, 2])),
        minimum_actual_rotor_rpm=float(truth[ts <= until_s+1e-9, 13:].min()*60/(2*np.pi)),
        first_gnss_update=first,
        max_gnss_position_injection_m=max((float(np.linalg.norm(d[:3])) for d in injections), default=None),
        max_gnss_velocity_injection_m_s=max((float(np.linalg.norm(d[3:6])) for d in injections), default=None),
        early_gnss_updates=early_updates, update_statistics=summarize_updates(navigation))


def grouped_counts(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row['stage'], row['controller'], row['case'], row['condition'])].append(row)
    summaries = []
    for key, members in sorted(grouped.items()):
        summaries.append(dict(stage=key[0], controller=key[1], case=key[2], condition=key[3],
            planned=len(members), recorded=sum(r['execution_status'] == 'recorded' for r in members),
            execution_incomplete=sum(r['execution_status'] == 'execution_incomplete' for r in members),
            pending=sum(r['execution_status'] == 'pending' for r in members),
            tracking_pass=sum(r.get('tracking_pass') is True and r['execution_status'] == 'recorded' for r in members),
            domain_pass=sum(r.get('model_domain_valid') is True and r['execution_status'] == 'recorded' for r in members),
            full_pass=sum(r.get('passed') is True and r['execution_status'] == 'recorded' for r in members),
            acceptance_failures_not_penalized_by_legacy_rule=sum(
                r['execution_status'] == 'recorded' and r.get('passed') is False
                and not r.get('stop_reason') and r.get('paper_failed') is False for r in members),
            domain_failures_not_penalized_by_legacy_rule=sum(
                r['execution_status'] == 'recorded' and r.get('model_domain_valid') is False
                and not r.get('stop_reason') and r.get('paper_failed') is False for r in members),
            early_stops=sum(bool(r.get('stop_reason')) for r in members),
            unique_trajectories=len({r['trajectory_sha256'] for r in members if r.get('trajectory_sha256')})))
    return summaries


def nominal_repeats(rows):
    """Compare full nominal factorial cells with their serial integration runs."""
    originals = {(r['controller'], r['case'], r['seed']): r for r in rows
                 if r['stage'] == 'integration' and r['condition'] == 'nominal'}
    checks = []
    for row in rows:
        if row['stage'] != 'gnss_startup' or row['condition'] != 'gnss_full_noise1_delay1':
            continue
        original = originals[(row['controller'], row['case'], row['seed'])]
        available = all(r['execution_status'] == 'recorded' and r.get('trajectory_sha256')
                        for r in (original, row))
        verdict_keys = ('passed', 'tracking_pass', 'model_domain_valid', 'stop_reason',
                        'paper_failed', 'simulated_seconds')
        checks.append(dict(trial_id=row['trial_id'], original_trial_id=original['trial_id'],
            comparison_available=bool(available),
            trajectory_bit_identical=(original['trajectory_sha256'] == row['trajectory_sha256']
                                      if available else None),
            verdicts_identical=(all(original.get(k) == row.get(k) for k in verdict_keys)
                                if available else None)))
    return checks


def analyze(source, output):
    output = Path(output).resolve()
    validated = output/'validated'
    data = summarize(source, validated)
    rows = data['records']
    if any(r['execution_status'] == 'pending' for r in rows):
        raise ValueError('startup conclusions require the completed preparation plan; pending trials remain')
    diagnostics = []
    for row in rows:
        if row['stage'] != 'gnss_startup':
            continue
        match = PATTERN.fullmatch(row['condition'])
        if match is None:
            raise ValueError('unrecognized GNSS factorial condition')
        detail = {k: row[k] for k in ('trial_id', 'controller', 'case', 'condition', 'seed', 'execution_status')}
        detail.update(context=match[1], noise=int(match[2]), delay=int(match[3]))
        if row.get('trace') and not row.get('diagnostics_unavailable'):
            path = validated/row['trace']
            navigation = json.loads(path.with_suffix('.navigation.json').read_text(encoding='utf-8'))
            with np.load(path, allow_pickle=False) as trace:
                detail.update(startup_metrics(trace, navigation))
            detail['domain_first_s'] = row.get('domain_diagnostics', {}).get('domain_first_s')
        else:
            detail['diagnostics_unavailable'] = row.get('diagnostics_unavailable', 'no_trace')
        diagnostics.append(detail)
    groups = grouped_counts(rows)
    summary = dict(schema='sensor_startup_diagnostics/1', stage='DEVELOPMENT',
        plan_sha256=data['plan_sha256'], expected=data['expected'], execution_counts=data['execution_counts'],
        analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        groups=groups, startup=diagnostics, nominal_repeats=nominal_repeats(rows),
        note='Fixed gains/covariances, development seeds only. Three seeds are not a hardware reliability '
             'estimate. Quiet repetitions may have identical trajectories. No startup exclusion from acceptance.')
    (output/'diagnostics.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    write_report(summary, output)
    plot_startup(rows, validated, output)
    return summary


def write_report(summary, output):
    lines = ['# Sensor integration and GNSS startup development study', '', summary['note'], '',
        'Counts retain all planned cases and executed failures. Execution errors and early simulation '
        'stops are different outcomes. The detailed validated report keeps duration, partial RMSE, '
        'stop reasons, source hashes, estimation errors and model-domain diagnostics.', '',
        '| Stage | Controller | Case | Condition | Recorded / planned | Track / domain / full pass | Early stops | Unique trajectories | Acceptance / domain failures unpenalized by legacy rule |',
        '|---|---|---|---|---:|---|---:|---:|---:|']
    for g in summary['groups']:
        lines.append(f'| {g["stage"]} | {g["controller"]} | {g["case"]} | {g["condition"]} | '
            f'{g["recorded"]} / {g["planned"]} | {g["tracking_pass"]} / {g["domain_pass"]} / '
            f'{g["full_pass"]} | {g["early_stops"]} | {g["unique_trajectories"]} | '
            f'{g["acceptance_failures_not_penalized_by_legacy_rule"]} / '
            f'{g["domain_failures_not_penalized_by_legacy_rule"]} |')
    lines += ['', 'The final column applies the current tuning failure rule '
        '`stop_reason or paper_failed` to these development records. It is not an actual '
        'tuning evaluation, and these cases are not moved into the tuning set. Its first count '
        'covers all failed acceptance verdicts; the second is the domain-failure subset and '
        'must not be added to the first. These trials still incur their ordinary RMSE cost; '
        'the counts concern the binary failure penalty only. Model-domain validity and some tracking acceptance '
        'checks, such as pre-gust settling, are separate from the inherited paper-failure rule. '
        'Declare their intended treatment before final tuning; this report does not change the objective.', '']
    lines += ['## Nominal repeat checks', '',
        'These full-context cells retain nominal noise, delay, covariance and controller settings, '
        'while enabling detailed update logging. Compare them with the corresponding serial '
        'integration trials; unavailable comparisons are not passes.', '',
        '| Factorial trial | Comparison available | Trajectory bit-identical | Verdicts identical |',
        '|---|---|---|---|']
    for check in summary['nominal_repeats']:
        lines.append(f'| {check["trial_id"]} | {check["comparison_available"]} | '
                     f'{check["trajectory_bit_identical"]} | {check["verdicts_identical"]} |')
    lines += ['', '## Startup diagnostics', '',
        'Errors and minima below cover the first 0.5 s, or the recorded prefix if it stopped sooner. '
        'The first GNSS update is logged at its first processing; its injection occurs at the '
        'historical sample time. Arrival also triggers propagation/replay, and several measurements '
        'can affect the next controller solve. Timing alone does not isolate a causal effect.', '',
        '| Controller | Condition | Seed | Peak position / velocity error | First GNSS processing [s] | First domain crossing [s] | Minimum actual RPM |',
        '|---|---|---:|---|---:|---:|---:|']
    fmt = lambda x: '—' if x is None else f'{x:.4g}'
    for r in summary['startup']:
        first = r.get('first_gnss_update') or {}
        lines.append(f'| {r["controller"]} | {r["condition"]} | {r["seed"]} | '
            f'{fmt(r.get("peak_position_error_m"))} m / {fmt(r.get("peak_velocity_error_m_s"))} m/s | '
            f'{fmt(first.get("processed_at_s"))} | {fmt(r.get("domain_first_s"))} | '
            f'{fmt(r.get("minimum_actual_rotor_rpm"))} |')
    lines += ['', '![Startup actual rotor speeds](startup_rotor.png)', '',
        'Each thin curve is one development seed; colors identify the GNSS noise/delay cell. '
        'The dashed line is the unchanged assumed 10,000 RPM propeller-model boundary. '
        'Isolated and full contexts differ in other sensor errors and rotor policy; compare '
        'factorial axes within each context. A model-domain crossing is not proof of hardware instability.', '',
        '[All trial records and separate execution statuses](validated/report.md)', '']
    (output/'report.md').write_text('\n'.join(lines), encoding='utf-8')


def plot_startup(rows, validated, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), layout='constrained', sharex=True)
    colors = {(0, 0): '#18795d', (0, 1): '#2470a0', (1, 0): '#df8b23', (1, 1): '#aa3157'}
    for i, controller in enumerate(('V13', 'F13')):
        for j, context in enumerate(('isolated', 'full')):
            ax = axes[i, j]
            for row in rows:
                match = PATTERN.fullmatch(row['condition'])
                if (not match or row['controller'] != controller or match[1] != context
                        or not row.get('trace') or row.get('diagnostics_unavailable')):
                    continue
                with np.load(validated/row['trace'], allow_pickle=False) as trace:
                    keep = trace['ts'] <= .5+1e-9
                    ax.plot(trace['ts'][keep], trace['xs'][keep, 13:].min(axis=1)*60/(2*np.pi),
                        color=colors[(int(match[2]), int(match[3]))], linewidth=1., alpha=.7)
            ax.axhline(10000., color='#444444', linestyle='--', linewidth=1.)
            ax.set_title(f'{controller} · {context} context')
            ax.set_xlabel('Time [s]'); ax.set_ylabel('Minimum actual rotor speed [RPM]')
            ax.grid(alpha=.2)
    from matplotlib.lines import Line2D
    fig.legend([Line2D([0], [0], color=c) for c in colors.values()],
        [f'noise={n}, delay={d}' for n, d in colors], loc='outside lower center', ncol=4, frameon=False)
    fig.suptitle('GNSS startup factorial · fixed controller/filter settings · seeds 3, 4, 5')
    fig.savefig(output/'startup_rotor.png', dpi=160)
    fig.savefig(output/'startup_rotor.svg')
    plt.close(fig)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(argv)
    result = analyze(a.source, a.output)
    print(json.dumps(dict(expected=result['expected'], execution_counts=result['execution_counts']), indent=2))


if __name__ == '__main__':
    main()
