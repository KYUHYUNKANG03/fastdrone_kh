"""Relate first-processing navigation innovations to observed state/thrust changes.

Arrival jumps include propagation and fixed-lag replay. They are deliberately
not attributed as additive, independent sensor effects.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np

from control.sensor_feedback_diagnostics import diagnose, trace_path


def link_arrivals(trace, navigation, solve_log):
    times = trace['ts'][:len(trace['us'])]
    requested = trace['nu_d'][:, 0]
    solves = np.asarray([r['t'] for r in solve_log], dtype=float)
    updates = {r['sequence']: r for r in navigation['updates']}
    rows = []
    for batch in navigation['arrivals']:
        processed = batch['processed_at_s']
        index = int(np.searchsorted(times, processed-1e-9))
        solver_index = int(np.searchsorted(solves, processed-1e-9))
        row = dict(batch)
        row['sensors'] = sorted({updates[s]['sensor'] for s in batch['sequences']})
        row['height_state_change_m'] = batch['state_after'][2]-batch['state_before'][2]
        row['velocity_state_change_m_s'] = (np.asarray(batch['state_after'][3:6])-
                                            np.asarray(batch['state_before'][3:6])).tolist()
        row['control_time_s'] = float(times[index]) if index < len(times) else None
        row['next_solve_time_s'] = float(solves[solver_index]) if solver_index < len(solves) else None
        row['thrust_before_next_solve_N'] = row['thrust_at_next_solve_N'] = None
        row['all_sensors_since_previous_solve'] = []
        if solver_index < len(solves):
            solve_time = solves[solver_index]
            step = int(np.searchsorted(times, solve_time-1e-9))
            if step < len(times):
                row['thrust_before_next_solve_N'] = float(requested[step-1]) if step > 0 else None
                row['thrust_at_next_solve_N'] = float(requested[step])
                previous = solves[solver_index-1] if solver_index > 0 else -np.inf
                row['all_sensors_since_previous_solve'] = sorted({r['sensor'] for r in navigation['updates']
                    if previous+1e-9 < r['processed_at_s'] <= solve_time+1e-9})
        rows.append(row)
    return rows


def summarize_updates(navigation):
    stats = {}
    for sensor in ('barometer', 'gnss', 'magnetometer'):
        entries = [r for r in navigation['updates'] if r['sensor'] == sensor]
        first_half = [r for r in entries if r['processed_at_s'] <= .5+1e-9]
        finite_nis = [r['nis'] for r in entries if r.get('nis') is not None and np.isfinite(r['nis'])]
        dz = [r['correction_error_state'][2] for r in first_half if 'correction_error_state' in r]
        stats[sensor] = dict(packets=len(entries), accepted=sum(r['status'] == 'accepted' for r in entries),
                             rejected=sum(r['status'] == 'rejected' for r in entries),
                             stale=sum(r['status'] == 'stale' for r in entries),
                             mean_nis=float(np.mean(finite_nis)) if finite_nis else None,
                             maximum_abs_sample_time_height_correction_before_0p5_s_m=
                             float(np.max(np.abs(dz))) if dz else None)
    return stats


def build_report(sources, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    outcomes, links, selected = [], [], []
    for source in map(lambda p: Path(p).resolve(), sources):
        campaign = json.loads(source.read_text())
        if campaign.get('complete') is not True:
            raise ValueError(f'incomplete campaign: {source}')
        for record in campaign['records']:
            path = trace_path(record, source)
            if path is None:
                outcomes.append(dict(record, trace_available=False))
                continue
            navigation_path = path.with_suffix('.navigation.json')
            navigation = json.loads(navigation_path.read_text())
            solve_log = json.loads(path.with_suffix('.solver.json').read_text())
            with np.load(path, allow_pickle=False) as data:
                arrivals = link_arrivals(data, navigation, solve_log)
                early = data['ts'] <= .5+1e-9
                n_est = min(len(data['xs_est']), len(data['xs']))
                height_error = data['xs_est'][:n_est, 2]-data['xs'][:n_est, 2]
                peak_error = float(np.max(np.abs(height_error[early[:n_est]])))
                minimum_rpm = float(data['xs'][early, 13:].min()*60/(2*np.pi))
                jumps = [abs(r['height_state_change_m']) for r in arrivals if r['processed_at_s'] <= .5+1e-9]
            outcome = diagnose(record, source)
            outcome.update(trace_available=True, navigation_trace=str(navigation_path),
                           maximum_startup_height_estimation_error_m=peak_error,
                           minimum_startup_actual_rpm=minimum_rpm,
                           maximum_startup_arrival_height_change_m=max(jumps, default=0.),
                           update_statistics=summarize_updates(navigation))
            outcomes.append(outcome)
            links.append(dict(controller=record['controller'], variant=record['variant'],
                              seed=record['sensor_seed'], arrivals=arrivals))
            if record['controller'] == 'V13' and record['sensor_seed'] == 1:
                for r in navigation['updates']:
                    if r['sensor'] in ('barometer', 'gnss') and r['processed_at_s'] <= .14+1e-9:
                        selected.append(dict(variant=record['variant'], **r))
    (output/'navigation_outcomes.json').write_text(json.dumps(outcomes, indent=2)+'\n')
    (output/'arrival_control_links.json').write_text(json.dumps(links, indent=2)+'\n')
    (output/'startup_updates_seed1.json').write_text(json.dumps(selected, indent=2)+'\n')
    groups = defaultdict(list)
    for row in outcomes:
        groups[(row['controller'], row['variant'])].append(row)
    lines = ['# Navigation measurement isolation', '',
             'Controller settings, plant, scenario, and estimator covariance tuning are fixed. '
             'Noise-free and zero-latency rows are diagnostic controls. Bias rows add +1 m to barometer '
             'height or GNSS height. Counts include failed outcomes.', '',
             '| Controller | Variant | Trials | Tracking / full pass | Velocity RMSE [m/s] | '
             'Altitude RMSE [m] | Peak startup height-estimation error [m] | Domain outside [%] |',
             '|---|---|---:|---|---:|---:|---:|---:|']
    for (controller, variant), rows in groups.items():
        def mean(key, factor=1.):
            values = [r[key]*factor for r in rows if r.get(key) is not None]
            return f'{np.mean(values):.3f}' if values else '—'
        lines.append(f'| {controller} | {variant} | {len(rows)} | '
                     f"{sum(r.get('tracking_pass') is True for r in rows)} / "
                     f"{sum(r.get('passed') is True for r in rows)} | {mean('rmse_velocity')} | "
                     f"{mean('rmse_z')} | {mean('maximum_startup_height_estimation_error_m')} | "
                     f"{mean('prop_domain_outside_fraction', 100.)} |")
    lines += ['', 'Peak startup error is computed within the first 0.5 s; acceptance always includes '
              'the entire trial. Table entries are means over the tested seeds, not confidence intervals.', '',
              '## Early update evidence: V13, seed 1', '',
              'Each packet is listed at its first processing. Its correction is at the historical '
              'sample time; later replays are not counted as new packets. NIS is diagnostic only, '
              'not a consistency certification.', '',
              '| Variant | Sensor | Sample / arrival / processing [ms] | Innovation in height [m] | '
              'Height error-state injection [m] | Barometer bias after update [m] | Status |',
              '|---|---|---|---:|---:|---:|---|']
    for row in selected:
        index = 0 if row['sensor'] == 'barometer' else 2
        residual = row.get('innovation', [None]*6)[index]
        correction = row.get('correction_error_state', [None]*15)[2]
        fmt = lambda x: '—' if x is None else f'{x:.5f}'
        times = ' / '.join(f'{row[k]*1000.:.1f}' for k in ('sample_time_s','arrival_time_s','processed_at_s'))
        lines.append(f"| {row['variant']} | {row['sensor']} | {times} | {fmt(residual)} | "
                     f"{fmt(correction)} | {fmt(row.get('baro_bias_after'))} | {row['status']} |")
    lines += ['', '## Interpretation limits', '',
              '- Arrival-state differences include propagation and replay of earlier measurements. '
              'Several sensors may contribute to the state used at the next NMPC solve.',
              '- `arrival_control_links.json` records the next actual solver time and thrust request. '
              'It lists all sensor kinds processed since the preceding solve; this is timing evidence, '
              'not a per-sensor causal decomposition of thrust.',
              '- Estimator covariance remains fixed even when generated measurement noise is zero. '
              'These rows isolate sensor errors rather than retune the estimator.',
              '- Navigation still begins at the known trim state. Two seeds and one lateral-gust '
              'case cannot establish hardware specifications or flight readiness.', '']
    (output/'navigation_report.md').write_text('\n'.join(lines))
    plot_startup(outcomes, output)
    return outcomes


def plot_startup(outcomes, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    variants = {'baseline': '#2563eb', 'baro_noise_zero': '#0d9488',
                'gnss_noise_zero': '#d97706', 'both_noise_zero': '#9333ea'}
    selected = [r for r in outcomes if r.get('trace_available') and r['controller'] == 'V13'
                and r['sensor_seed'] == 1 and r['variant'] in variants]
    if len(selected) != len(variants):
        return
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), layout='constrained', sharex=True)
    for row in selected:
        with np.load(row['trace'], allow_pickle=False) as data:
            t = data['ts']; keep = t <= .5+1e-9; controls = t[:-1] <= .5+1e-9
            options = dict(color=variants[row['variant']], label=row['variant'], linewidth=1.4)
            n_est = min(len(data['xs_est']), len(data['xs']))
            est_keep = keep[:n_est]
            axes[0].plot(t[:n_est][est_keep]*1000.,
                         (data['xs_est'][:n_est, 2]-data['xs'][:n_est, 2])[est_keep], **options)
            axes[1].step(t[:-1][controls]*1000., data['nu_d'][controls, 0], where='post', **options)
            axes[2].plot(t[keep]*1000., data['xs'][keep, 13:].min(axis=1)*60/(2*np.pi), **options)
    axes[0].set_ylabel('Height estimation error [m]')
    axes[1].set_ylabel('NMPC thrust request [N]')
    axes[2].set_ylabel('Minimum actual rotor RPM')
    axes[2].axhline(selected[0]['assumed_minimum_rpm'], color='#555555', linestyle='--', linewidth=1)
    axes[2].set_xlabel('Time [ms]')
    for ax in axes:
        ax.grid(alpha=.2)
    fig.suptitle('Navigation noise isolation · V13 · seed 1\nController and estimator tuning fixed')
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=2, frameon=False)
    fig.savefig(output/'navigation_startup.png', dpi=160)
    fig.savefig(output/'navigation_startup.svg')
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sources', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    rows = build_report(args.sources, args.output)
    print(f'Wrote {len(rows)} outcomes to {args.output}')


if __name__ == '__main__':
    main()
