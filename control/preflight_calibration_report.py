"""Report paired barometer calibration experiments without changing verdicts."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np

from control.navigation_update_report import link_arrivals, summarize_updates
from control.sensor_feedback_diagnostics import diagnose, trace_path


def startup_metrics(data, navigation, links, horizon=.5):
    n = min(len(data['xs_est']), len(data['xs']))
    keep = data['ts'][:n] <= horizon+1e-9
    errors = data['xs_est'][:n, 2]-data['xs'][:n, 2]
    control_times = data['ts'][:len(data['nu_d'])]
    thrust = data['nu_d'][control_times <= horizon+1e-9, 0]
    updates, arrivals = navigation['updates'], navigation['arrivals']
    first_gnss = next((r for r in updates if r['sensor'] == 'gnss'), None)
    first_baro = next((r for r in updates if r['sensor'] == 'barometer'), None)
    batch = next((r for r in links if first_gnss and r['batch_id'] == first_gnss['batch_id']), None)
    bias0 = arrivals[0]['baro_bias_before'] if arrivals else None
    jumps = [abs(r['height_state_change_m']) for r in links if r['processed_at_s'] <= horizon+1e-9]
    result = dict(maximum_startup_height_estimation_error_m=float(np.max(np.abs(errors[keep]))),
                  minimum_startup_actual_rpm=float(data['xs'][data['ts'] <= horizon+1e-9, 13:].min()*60/(2*np.pi)),
                  maximum_startup_thrust_step_N=float(np.max(np.abs(np.diff(thrust)))) if len(thrust) > 1 else 0.,
                  maximum_startup_arrival_height_change_m=max(jumps, default=0.),
                  bias_before_flight_updates_m=bias0,
                  bias_after_first_barometer_m=first_baro.get('baro_bias_after') if first_baro else None,
                  bias_at_final_arrival_m=arrivals[-1]['baro_bias_after'] if arrivals else None,
                  first_gnss_arrival=None)
    if batch is not None:
        result['first_gnss_arrival'] = dict(
            processing_time_s=batch['processed_at_s'],
            bias_before_m=batch['baro_bias_before'], bias_after_m=batch['baro_bias_after'],
            batch_bias_change_m=batch['baro_bias_after']-batch['baro_bias_before'],
            batch_height_change_m=batch['height_state_change_m'],
            sample_time_gnss_height_injection_m=first_gnss.get('correction_error_state', [None]*15)[2],
            next_solve_time_s=batch['next_solve_time_s'],
            thrust_before_solve_N=batch['thrust_before_next_solve_N'],
            thrust_at_solve_N=batch['thrust_at_next_solve_N'],
            all_sensors_since_previous_solve=batch['all_sensors_since_previous_solve'])
    return result


def build_report(sources, output):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    outcomes, all_links = [], []
    for source in (Path(p).resolve() for p in sources):
        campaign = json.loads(source.read_text())
        if not campaign.get('complete'):
            raise ValueError(f'incomplete campaign: {source}')
        for record in campaign['records']:
            path = trace_path(record, source)
            if path is None:
                outcomes.append(dict(record, trace_available=False))
                continue
            nav = json.loads(path.with_suffix('.navigation.json').read_text())
            solver = json.loads(path.with_suffix('.solver.json').read_text())
            with np.load(path, allow_pickle=False) as data:
                links = link_arrivals(data, nav, solver)
                startup = startup_metrics(data, nav, links)
            row = diagnose(record, source)
            row.update(trace_available=True, **startup, update_statistics=summarize_updates(nav))
            outcomes.append(row)
            all_links.append(dict(controller=record['controller'], variant=record['variant'],
                                  seed=record['sensor_seed'], arrivals=links))
    (output/'preflight_outcomes.json').write_text(json.dumps(outcomes, indent=2)+'\n')
    (output/'arrival_control_links.json').write_text(json.dumps(all_links, indent=2)+'\n')
    groups = defaultdict(list)
    for row in outcomes: groups[(row['controller'], row['variant'])].append(row)
    lines = ['# Preflight barometer calibration comparison', '',
        'Identical sensor specifications, paired flight-noise streams, fixed controller and covariance tuning. '
        'Baseline calibrates from the first arriving flight barometer sample. Preflight variants average '
        'independent stationary samples at a known reference altitude before the mission. '
        'Only `preflight100_frozen` disables ongoing GNSS-based barometer-bias learning.', '',
        '| Controller | Variant | Trials | Tracking / full passes | Altitude RMSE [m] | '
        'Velocity RMSE [m/s] | Peak startup height error [m] | Largest startup thrust step [N] | Domain outside [%] |',
        '|---|---|---:|---|---:|---:|---:|---:|---:|']
    for (controller, variant), rows in groups.items():
        def mean(key, factor=1.):
            values = [r[key]*factor for r in rows if r.get(key) is not None]
            return f'{np.mean(values):.4f}' if values else '—'
        lines.append(f'| {controller} | {variant} | {len(rows)} | '
            f"{sum(r.get('tracking_pass') is True for r in rows)} / {sum(r.get('passed') is True for r in rows)} | "
            f"{mean('rmse_z')} | {mean('rmse_velocity')} | {mean('maximum_startup_height_estimation_error_m')} | "
            f"{mean('maximum_startup_thrust_step_N')} | {mean('prop_domain_outside_fraction', 100.)} |")
    lines += ['', 'Startup metrics use the first 0.5 s; verdicts use the complete trial. '
              'Values are two-seed means, not confidence limits. Missing or failed outcomes remain in pass denominators.', '',
              '## First delayed GNSS arrival, V13', '',
              '| Variant | Seed | Initial preflight bias [m] | Bias after first barometer [m] | '
              'Bias change at GNSS arrival [m] | Total height change at arrival [m] | Thrust before / at solve [N] |',
              '|---|---:|---:|---:|---:|---:|---|']
    for row in outcomes:
        if row['controller'] != 'V13' or not row.get('trace_available'): continue
        first = row['first_gnss_arrival']
        if first is None: continue
        fmt = lambda x: '—' if x is None else f'{x:.5f}'
        lines.append(f"| {row['variant']} | {row['sensor_seed']} | {fmt(row['bias_before_flight_updates_m'])} | "
                     f"{fmt(row['bias_after_first_barometer_m'])} | {fmt(first['batch_bias_change_m'])} | "
                     f"{fmt(first['batch_height_change_m'])} | {fmt(first['thrust_before_solve_N'])} / "
                     f"{fmt(first['thrust_at_solve_N'])} |")
    lines += ['', '## Limits', '',
        '- Calibration is installed in the replay baseline. Zero bias change at the first GNSS arrival '
        'does not imply zero navigation-state correction; GNSS still corrects position and velocity.',
        '- Arrival changes include propagation, all new measurements, and replayed older updates. '
        'They are not a per-sensor causal decomposition of thrust.',
        '- Preflight samples assume white independent noise, fixed barometer bias, and exact reference altitude. '
        'At 50 Hz, counts 1/25/100 correspond to nominal acquisition budgets 0.02/0.5/2 s. '
        'Preflight dynamics, packet latency/dropout, pressure transients, and reference uncertainty are not simulated.',
        '- The 15-state covariance excludes barometer-calibration uncertainty. Averaging lowers its initial '
        'random error but does not reduce in-flight measurement noise or GNSS errors.',
        '- Frozen bias learning is an isolation experiment. It does not model compensation for later pressure drift.',
        '- Only one lateral-gust case and two seeds per controller were tested. No default is promoted.', '']
    (output/'preflight_report.md').write_text('\n'.join(lines))
    plot_startup(outcomes, output)
    return outcomes


def plot_startup(outcomes, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'baseline':'#475569', 'preflight1':'#d97706', 'preflight25':'#2563eb',
              'preflight100':'#0d9488', 'preflight100_frozen':'#9333ea'}
    selected = [r for r in outcomes if r.get('trace_available') and r['sensor_seed'] == 1]
    controllers = [c for c in ('V13','F13') if any(r['controller'] == c for r in selected)]
    if not controllers: return
    fig, axes = plt.subplots(4, len(controllers), figsize=(13, 12), squeeze=False, layout='constrained', sharex=True)
    for col, controller in enumerate(controllers):
        for row in selected:
            if row['controller'] != controller: continue
            with np.load(row['trace'], allow_pickle=False) as data:
                t = data['ts']; n = min(len(t), len(data['xs_est'])); keep = t[:n] <= .5+1e-9
                options = dict(color=colors[row['variant']], label=row['variant'], linewidth=1.3,
                               linestyle='--' if row['variant'].endswith('frozen') else '-')
                axes[0,col].plot(t[:n][keep]*1000., (data['xs_est'][:n,2]-data['xs'][:n,2])[keep], **options)
                ct = t[:len(data['nu_d'])]; ck = ct <= .5+1e-9
                axes[1,col].step(ct[ck]*1000., data['nu_d'][ck,0], where='post', **options)
                tk = t <= .5+1e-9
                axes[2,col].plot(t[tk]*1000., data['xs'][tk,13:].min(axis=1)*60/(2*np.pi), **options)
            nav = json.loads(Path(row['trace']).with_suffix('.navigation.json').read_text())
            early = [a for a in nav['arrivals'] if a['processed_at_s'] <= .5+1e-9]
            bt = [0.] + [a['processed_at_s']*1000. for a in early]
            bv = [row['bias_before_flight_updates_m']] + [a['baro_bias_after'] for a in early]
            axes[3,col].step(bt, bv, where='post', **options)
        axes[0,col].set_title(controller+' · seed 1')
        axes[2,col].axhline(10000., color='#555555', linestyle=':', linewidth=1)
        axes[3,col].set_xlabel('Time [ms]')
        for ax in axes[:,col]: ax.grid(alpha=.2)
    for ax, label in zip(axes[:,0], ['Height estimation error [m]', 'NMPC thrust request [N]',
                                    'Minimum actual rotor RPM', 'Barometer bias estimate [m]']):
        ax.set_ylabel(label)
    handles, labels = axes[0,0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=3, frameon=False)
    fig.suptitle('Preflight calibration with paired flight noise\nController and covariance tuning fixed')
    fig.savefig(output/'preflight_startup.png', dpi=160)
    fig.savefig(output/'preflight_startup.svg')
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
