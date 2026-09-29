"""Summarize filter experiments and trace startup commands against telemetry availability."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np

from control.sensor_feedback_diagnostics import diagnose, trace_path


def startup(record, source):
    path = trace_path(record, source)
    with np.load(path, allow_pickle=False) as data:
        commands, times = data['us'], data['ts'][:-1]
        ready, paths = data['indi_rotor_ready'], data['indi_path']
        early = times < .5
        available = np.flatnonzero(ready)
        return dict(first_rotor_ready_s=float(times[available[0]]) if len(available) else None,
                    allocation_before_rotor_ready=int(np.sum(~ready & np.isin(paths, ['A0', 'A1']))),
                    all_zero_commands_before_rotor_ready=int(np.sum(~ready & np.all(commands == 0., axis=1))),
                    minimum_startup_command_rpm=float(commands[early].min()*60/(2*np.pi)),
                    minimum_startup_actual_rpm=float(data['xs'][:-1, 13:][early].min()*60/(2*np.pi)),
                    first_paths=paths[:4].tolist(),
                    first_commands_rad_s=commands[:4].tolist(),
                    first_estimated_rotors_rad_s=data['xs_est'][:4, 13:].tolist())


def build_report(sources, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for source in map(lambda p: Path(p).resolve(), sources):
        campaign = json.loads(source.read_text())
        if campaign.get('complete') is not True:
            raise ValueError(f'incomplete experiment: {source}')
        for record in campaign['records']:
            row = diagnose(record, source)
            if row is None:
                rows.append(dict(record, diagnostic_available=False))
                continue
            row.update(startup(record, source), diagnostic_available=True)
            rows.append(row)
    (output/'filter_diagnostics.json').write_text(json.dumps(rows, indent=2)+'\n')
    groups = defaultdict(list)
    for row in rows:
        groups[(row.get('scenario_id'), row['profile'], row['controller'], row['variant'])].append(row)
    lines = ['# INDI filter and startup experiment', '',
             'The sensor profile is fixed within each GNSS level. Variants change the controller only. '
             'The bundled controller defaults and acceptance criteria remain unchanged.', '',
             'RMS derivative-error and command-slew diagnostics exclude the first 0.5 s. '
             'Tracking and propulsion-domain verdicts include the complete trial. '
             'Command slew is a vector-norm RMS of the four rotor commands, not actual motor acceleration.', '',
             '| Case | Profile | Controller | Variant | Trials | Track / full pass | '
             'RMSE v [m/s] | Filtered derivative error [rad/s²] | Command slew [rad/s²] | Domain outside [%] |',
             '|---|---|---|---|---:|---|---:|---:|---:|---:|']
    for (case, profile, controller, variant), entries in groups.items():
        def mean(key, scale=1.):
            values = [r[key]*scale for r in entries if r.get(key) is not None]
            return f'{np.mean(values):.3f}' if values else '—'
        lines.append(f'| {case} | {profile} | {controller} | {variant} | {len(entries)} | '
                     f"{sum(r.get('tracking_pass') is True for r in entries)} / "
                     f"{sum(r.get('passed') is True for r in entries)} | {mean('rmse_velocity')} | "
                     f"{mean('angular_accel_error_filtered_rms_rad_s2')} | {mean('command_slew_rms_rad_s2')} | "
                     f"{mean('prop_domain_outside_fraction', 100.)} |")
    lines += ['', '## Startup evidence', '',
              '| Profile | Controller | Variant | Seed | First RPM available [ms] | '
              'Allocations before RPM available | All-zero commands before RPM available | Min actual RPM before 0.5 s |',
              '|---|---|---|---:|---:|---:|---:|---:|']
    for r in rows:
        if not r['diagnostic_available']:
            continue
        ready = r['first_rotor_ready_s']
        ready_text = 'never' if ready is None else f'{ready*1000.:.1f}'
        lines.append(f"| {r['profile']} | {r['controller']} | {r['variant']} | {r['sensor_seed']} | "
                     f"{ready_text} | {r['allocation_before_rotor_ready']} | "
                     f"{r['all_zero_commands_before_rotor_ready']} | {r['minimum_startup_actual_rpm']:.1f} |")
    lines += ['', '## Scope', '',
              '- S1 applies a midpoint and the same LPF to the rotor-force path. '
              'This does not compensate for upstream ESC transport latency or rotor-observer filtering.',
              '- INDI still computes control effectiveness and adds the rotor increment at the current '
              'observed rotor state. S1 is not complete actuator-state time alignment.',
              '- F13 aligned50 isolates switching from S0 to S1 at 50 Hz. '
              'Compare its lower-cutoff variants against aligned50 to isolate the cutoff effect.',
              '- Nominal analog time constants are 3.18 ms at 50 Hz, 6.37 ms at 25 Hz, '
              'and 12.73 ms at 12.5 Hz. Actual phase delay depends on the discrete filter and frequency.',
              '- The startup guard is availability gating only. It neither estimates telemetry accuracy '
              'nor handles stale telemetry after initialization.',
              '- Few seeds and low-speed gust cases do not establish sensor requirements or flight readiness.', '']
    (output/'filter_report.md').write_text('\n'.join(lines))
    plot_startup(rows, output)
    return rows


def plot_startup(rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected = [r for r in rows if r.get('diagnostic_available') and r['controller'] == 'V13'
                and r['profile'] == 'nav_imu0_gnss0' and r['sensor_seed'] == 1
                and r['variant'] in ('baseline', 'startup_guard')]
    if len(selected) != 2:
        return
    arrivals = [r['first_rotor_ready_s'] for r in selected if r['first_rotor_ready_s'] is not None]
    first_ready = max(arrivals) if arrivals else None
    minimum_rpm = selected[0]['assumed_minimum_rpm']
    fig, panels = plt.subplots(2, 2, figsize=(11, 7.8), layout='constrained')
    axes = panels[0]
    for row in selected:
        color = '#2563eb' if row['variant'] == 'baseline' else '#0d9488'
        with np.load(row['trace'], allow_pickle=False) as trace:
            t, xs, cmd = trace['ts'], trace['xs'], trace['us']
            for ax, stop in zip(axes, [.016, .5]):
                actual = t <= stop
                control = t[:-1] <= stop
                ax.plot(t[actual]*1000, xs[actual, 13:].min(axis=1)*60/(2*np.pi),
                        color=color, label=row['variant']+' actual')
                ax.step(t[:-1][control]*1000, cmd[control].min(axis=1)*60/(2*np.pi),
                        where='post', color=color, linestyle=':', label=row['variant']+' command')
            keep = t[:-1] <= .5
            panels[1, 0].plot(t[:-1][keep]*1000,
                              (trace['xs_est'][:-1, 2]-xs[:-1, 2])[keep],
                              color=color, label=row['variant'])
            panels[1, 1].step(t[:-1][keep]*1000, trace['nu_d'][keep, 0], where='post',
                              color=color, label=row['variant'])
    for ax in axes:
        if first_ready is not None:
            ax.axvline(first_ready*1000., color='#555555', linestyle='--', linewidth=1)
        ax.axhline(minimum_rpm, color='#b45309', linestyle='--', linewidth=1)
        ax.set_xlabel('Time [ms]')
        ax.set_ylabel('Minimum across four rotors [mechanical RPM]')
        ax.grid(alpha=.2)
    axes[0].set_title(f'First telemetry available by {first_ready*1000.:g} ms'
                      if first_ready is not None else 'No arrived telemetry')
    axes[1].set_title(f'Startup; assumed model minimum = {minimum_rpm:,.0f} RPM')
    panels[1, 0].set_title('Navigation height error during startup')
    panels[1, 0].set_ylabel('Estimated minus true height [m]')
    panels[1, 1].set_title('NMPC requested total thrust')
    panels[1, 1].set_ylabel('Requested thrust [N]')
    for ax in panels[1]:
        ax.set_xlabel('Time [ms]')
        ax.grid(alpha=.2)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=2, frameon=False)
    fig.suptitle('V13 · nominal matched GNSS · seed 1 · lateral gust case')
    fig.savefig(output/'startup_transient.png', dpi=170)
    fig.savefig(output/'startup_transient.svg')
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
