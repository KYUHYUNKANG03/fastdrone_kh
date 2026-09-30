"""Report paired envelope trials, retaining failures and verifying provenance."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from control.arena import config_sha256
from control.sensor_binding import resolve_feedback, runtime_source_hashes
from control.sensor_feedback_diagnostics import diagnose, trace_path
from control.validation_suite import plant_truth
from models.team_light.control.baseline_v2 import baseline_params, parameter_hash


def relative(path):
    return str(Path(path).resolve().relative_to(ROOT))


def trace_metrics(path, feedback):
    """Estimation metrics cover recorded time, including startup and failures."""
    result = {}
    with np.load(path, allow_pickle=False) as trace:
        times, truth = trace['ts'], trace['xs']
        if times.ndim != 1 or truth.shape != (len(times), 17):
            raise ValueError('malformed truth trace shape')
        if len(times) < 2:
            return dict(diagnostics_unavailable='no_executed_steps')
        # Divergence is a legitimate saved failure. Retain its verdict instead
        # of dropping it or failing the entire report on SciPy/min/NaN errors.
        for key in ('ts', 'xs', 'us', 'wind'):
            if key in trace and not np.all(np.isfinite(trace[key])):
                return dict(diagnostics_unavailable='nonfinite_'+key)
        if np.any(np.diff(times) <= 0):
            raise ValueError('non-increasing trace timestamps')
        if np.any(np.linalg.norm(truth[:, 6:10], axis=1) == 0):
            return dict(diagnostics_unavailable='invalid_truth_quaternion')
        if feedback == 'sensors':
            if trace['xs_est'].ndim != 2 or trace['xs_est'].shape[1:] != (17,):
                raise ValueError('malformed estimate trace shape')
            if len(trace['xs_est']) == 0:
                return dict(diagnostics_unavailable='no_estimated_states')
            if not np.all(np.isfinite(trace['xs_est'])):
                return dict(diagnostics_unavailable='nonfinite_estimated_states')
            if np.any(np.linalg.norm(trace['xs_est'][:, 6:10], axis=1) == 0):
                return dict(diagnostics_unavailable='invalid_estimated_quaternion')
            count = min(len(trace['xs']), len(trace['xs_est']))
            truth, estimate = trace['xs'][:count], trace['xs_est'][:count]
            error = estimate-truth
            rms = lambda a: float(np.sqrt(np.mean(np.sum(a*a, axis=1))))
            angle = (Rotation.from_quat(truth[:, 6:10]).inv()*
                     Rotation.from_quat(estimate[:, 6:10])).magnitude()
            result['estimation_rms'] = dict(position_m=rms(error[:, :3]),
                velocity_m_s=rms(error[:, 3:6]), body_rate_rad_s=rms(error[:, 10:13]),
                rotor_rad_s=rms(error[:, 13:17]),
                attitude_deg=float(np.rad2deg(np.sqrt(np.mean(angle*angle)))))
            probe_keys = {'indi_rotor_ready', 'indi_rotor_sample_time_s', 'indi_path'}
            present = probe_keys.intersection(trace.files)
            if not present:
                # M17/GSLQR/CPID have state estimates but no INDI probe.
                result['rotor_availability_unavailable'] = 'controller_has_no_indi_probe'
                return result
            if present != probe_keys:
                raise ValueError('incomplete INDI availability trace')
            ready = trace['indi_rotor_ready'].astype(bool)
            times = trace['ts'][:len(ready)]
            sample = trace['indi_rotor_sample_time_s']
            age = times-sample
            finite = np.isfinite(age)
            # Command ticks are plant ticks; elapsed duration accounts for stops.
            dt = np.diff(trace['ts'])[:len(ready)]
            stale = ~ready
            allocations = np.isin(trace['indi_path'], ['A0', 'A1'])
            stale_indices = np.flatnonzero(stale)
            result['rotor_availability'] = dict(
                not_ready_ticks=int(stale.sum()), not_ready_seconds=float(dt[stale].sum()),
                allocation_not_ready_ticks=int(np.sum(stale & allocations)),
                max_sample_age_s=float(age[finite].max()) if finite.any() else None,
                first_not_ready_s=float(times[stale_indices[0]]) if stale.any() else None,
                last_not_ready_s=float(times[stale_indices[-1]]) if stale.any() else None,
                ready_at_final_command=bool(ready[-1]) if len(ready) else None)
    return result


def paired_outcome(truth, sensors):
    # These are descriptive labels, not causal attribution or significance.
    if any(r.get('campaign_timed_out') or r.get('simulated_seconds') is None for r in (truth, sensors)):
        return 'execution_incomplete'
    if truth.get('passed') and sensors.get('passed'):
        return 'both_pass'
    if truth.get('passed'):
        return 'fusion_only_failure'
    if sensors.get('passed'):
        return 'truth_only_failure'
    return 'both_fail'


def verify_feedback_binding(config, manifest, record):
    binding = resolve_feedback(config)
    for key, expected in binding.metadata.items():
        # The manifest stores a file path here; records use the profile name.
        actual = (manifest.get('sensor_profile_config', {}).get('name')
                  if key == 'sensor_profile' else manifest.get(key))
        if actual != expected or record.get(key) != expected:
            raise ValueError('trial feedback binding differs from task')
    return binding


def summarize(sources, output):
    hashes = runtime_source_hashes()
    rows, contracts, configs, identities = [], [], {}, set()
    for source in map(lambda p: Path(p).resolve(), sources):
        doc = json.loads(source.read_text(encoding='utf-8'))
        contract = doc['contract']
        if contract.get('schema') != 'sensor_envelope_screen/1' or contract.get('stage') != 'DEVELOPMENT':
            raise ValueError('expected an envelope DEVELOPMENT screen')
        if contract['runtime_source_sha256'] != hashes:
            raise ValueError('runtime differs from recorded screen')
        driver_hash = hashlib.sha256((ROOT/'scripts/sensor_envelope_screen.py').read_bytes()).hexdigest()
        if contract.get('driver_sha256') != driver_hash:
            raise ValueError('driver differs from recorded screen')
        tasks = {t['trial_id']: t for t in contract['tasks']}
        records = {r['trial_id']: r for r in doc['records']}
        if (not doc.get('complete') or len(tasks) != doc['expected_trials']
                or len(records) != len(doc['records']) or set(records) != set(tasks)):
            raise ValueError('incomplete or duplicate trial records')
        compact = deepcopy(contract)
        compact['source'] = relative(source)
        configs[contract['base_config_sha256']] = compact.pop('base_config')
        for task in compact['tasks']:
            config = task.pop('config')
            task['config_sha256'] = config_sha256(config)
            configs[task['config_sha256']] = config
        contracts.append(compact)
        local_rows = {}
        for trial_id, record in records.items():
            task = tasks[trial_id]
            config = task['config']
            identity = (trial_id, config_sha256(config))
            if identity in identities:
                raise ValueError('duplicate trial across sources')
            identities.add(identity)
            if record.get('input_config_sha256') != config_sha256(config):
                raise ValueError('record input configuration differs from task')
            if record.get('feedback') != task['feedback']:
                raise ValueError('record feedback differs from task')
            row = {k:v for k,v in record.items() if k != 'run_dir'}
            row.update(source=relative(source), run_dir=relative(record['run_dir']) if record.get('run_dir') else None)
            path = trace_path(record, source)
            row['trace'] = relative(path) if path else None
            if path:
                manifest = json.loads((path.parent/'manifest.json').read_text(encoding='utf-8'))
                if manifest.get('config_sha256') != config_sha256(config):
                    raise ValueError('trial manifest configuration differs from task')
                if any(manifest.get('source_sha256', {}).get(k) != v for k,v in hashes.items()):
                    raise ValueError('trial runtime differs from screen')
                binding = verify_feedback_binding(config, manifest, record)
                if record.get('controller') != task['controller'] or record.get('scenario_id') != task['case']:
                    raise ValueError('trial controller/scenario differs from task')
                scenario = next(s for s in config['scenarios'] if s['id']==task['case'])
                if record.get('factors', {}) != scenario.get('perturbation', {}):
                    raise ValueError('physical perturbation differs from task')
                plant = plant_truth(baseline_params(), record)
                if parameter_hash(plant) != record['truth_parameter_sha256']:
                    raise ValueError('physical parameter hash differs from trial')
                row['physical_motor_tau_s'] = float(plant['tau_m'])
                row['observer_motor_tau_s'] = (binding.profile['rotor_observer']['motor_tau_s']
                                               if binding.profile else None)
                row.update(trace_metrics(path, task['feedback']))
                if 'diagnostics_unavailable' not in row:
                    detail = diagnose(record, source)
                    row['domain_diagnostics'] = {k:v for k,v in detail.items() if k not in ('source', 'trace')}
            local_rows[trial_id] = row
            rows.append(row)
        for row in local_rows.values():
            if row['feedback'] == 'sensors':
                expected_truth = tasks[row['trial_id']]['truth_trial_id']
                if row.get('truth_trial_id') != expected_truth:
                    raise ValueError('record truth pairing differs from task')
                truth = local_rows[expected_truth]
                if (truth.get('truth_parameter_sha256') and row.get('truth_parameter_sha256')
                        and truth['truth_parameter_sha256'] != row['truth_parameter_sha256']):
                    raise ValueError('paired trials did not use the same physical plant')
                row['paired_outcome'] = paired_outcome(truth, row)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    data = dict(schema='sensor_envelope_report/1', stage='DEVELOPMENT',
                report_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                contracts=contracts, configurations=configs, records=rows)
    (output/'outcomes.json').write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    write_report(rows, output)
    return data


def write_report(rows, output):
    by_id = {(r['source'], r['trial_id']): r for r in rows}
    pairs = [r for r in rows if r['feedback']=='sensors']
    number = lambda v: '—' if v is None else f'{v:.3f}'
    verdict = lambda r: f"{r.get('tracking_pass')} / {r.get('model_domain_valid')}"
    lines = ['# Paired sensor envelope development screen', '',
        '**Fixed gains; development seeds; not final tuning or paper evidence.**', '',
        'Each fused-feedback run is paired with the same controller, physical scenario and '
        'acceptance thresholds under truth feedback. Deterministic truth controls are shared '
        'across sensor-only variants and seeds; those shared controls are not independent repeats. '
        'The observer assumes 20 ms throughout. Physical motor perturbations change the plant only. '
        'All other sensor settings retain the v9 candidate, including steady-trim rotor prehistory.', '',
        'Track/domain columns show separate verdicts. Full pass requires both. An early-stop RMSE '
        'covers only the recorded prefix and must not rank above a full trial on that basis. '
        'Truth failure prevents attributing the corresponding failure solely to sensing. '
        'The original paper criteria and failure reasons are also retained in the JSON.', '',
        '| Controller | Condition | Seed | Truth track/domain | Fusion track/domain | Duration truth/fusion [s] | Fusion z / v RMSE | Pair outcome |',
        '|---|---|---:|---|---|---|---|---|']
    for row in pairs:
        truth = by_id[(row['source'], row['truth_trial_id'])]
        lines.append(f"| {row['controller']} | {row['condition']} | {row['sensor_seed']} | "
            f"{verdict(truth)} | {verdict(row)} | {number(truth.get('simulated_seconds'))} / "
            f"{number(row.get('simulated_seconds'))} | {number(row.get('rmse_z'))} / "
            f"{number(row.get('rmse_velocity'))} | {row['paired_outcome']} |")
    lines += ['', '## Rotor availability', '',
        'Readiness is an age/availability diagnostic. The current controller keeps operating '
        'while it is false; this is not a validated fallback policy.', '',
        '| Controller | Condition | Seed | Physical / observer tau [ms] | Max sample age [ms] | Not-ready time [s] | Allocations while not ready |',
        '|---|---|---:|---|---:|---:|---:|']
    for row in pairs:
        a = row.get('rotor_availability', {})
        ms = lambda value: number(value*1000 if value is not None else None)
        lines.append(f"| {row['controller']} | {row['condition']} | {row['sensor_seed']} | "
            f"{ms(row.get('physical_motor_tau_s'))} / {ms(row.get('observer_motor_tau_s'))} | "
            f"{ms(a.get('max_sample_age_s'))} | {number(a.get('not_ready_seconds'))} | "
            f"{a.get('allocation_not_ready_ticks', '—')} |")
    lines += ['', '![Paired outcomes](comparison.png)', '',
        'The JSON embeds source hashes, deduplicated full configurations, all trial verdicts, '
        'pair identities, and relative local trace paths. Missing-output and timed-out trials '
        'remain in the record. Raw traces are kept locally. Use the recorded condition/seed '
        'axes to reproduce into a new output directory.', '']
    (output/'report.md').write_text('\n'.join(lines), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, max(4, .5*len(pairs)+1.8)), layout='constrained')
    y = np.arange(len(pairs))
    for offset, mode in [(-.18, 'truth'), (.18, 'fusion')]:
        values = [by_id[(r['source'], r['truth_trial_id'])] if mode=='truth' else r for r in pairs]
        ax.barh(y+offset, [r.get('simulated_seconds') or 0 for r in values], height=.32,
            color=['#18795d' if r.get('passed') else '#b14434' for r in values],
            hatch='//' if mode=='truth' else None, label=mode)
    ax.set_yticks(y, [f"{r['controller']} · {r['condition']} · seed {r['sensor_seed']}" for r in pairs])
    ax.invert_yaxis(); ax.set_xlabel('Recorded duration [s] · hatched = truth, solid = fusion')
    ax.set_title('Development envelope · green = tracking + model domain pass\n'
                 'red = failure; longer duration alone is not success')
    fig.savefig(output/'comparison.png', dpi=150)
    plt.close(fig)
    if plot_low_speed_boundary(pairs, by_id, output):
        with (output/'report.md').open('a', encoding='utf-8') as file:
            file.write('\n## Low-speed propulsion-domain boundary\n\n'
                'Minimum actual rotor speed across all four rotors, from the saved plant traces. '
                'The horizontal line is the unchanged assumed lower propeller-data boundary. '
                'This plot diagnoses a model-domain crossing; it does not establish hardware instability.\n\n'
                '![Low-speed rotor boundary](low_speed_domain.png)\n')


def plot_low_speed_boundary(pairs, by_id, output):
    selected = [r for r in pairs if r['condition']=='lateral_low' and r.get('trace')
                and r.get('domain_diagnostics')
                and by_id[(r['source'], r['truth_trial_id'])].get('domain_diagnostics')]
    if not selected:
        return False
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(selected), 2, squeeze=False,
                            figsize=(11, 3.1*len(selected)), layout='constrained')
    for panel, row in zip(axes, selected):
        truth = by_id[(row['source'], row['truth_trial_id'])]
        for record, label, color in [(truth, 'truth feedback', '#555555'),
                                      (row, 'fused feedback', '#087f8c')]:
            with np.load(ROOT/record['trace'], allow_pickle=False) as trace:
                time, speed = trace['ts'], trace['xs'][:, 13:17].min(axis=1)*60/(2*np.pi)
            for ax, stop in zip(panel, [.5, float(time[-1])]):
                keep = time <= stop
                ax.plot(time[keep], speed[keep], color=color, label=label, linewidth=1.)
        for ax in panel:
            ax.axhline(row['domain_diagnostics']['assumed_minimum_rpm'], color='#b14434',
                       linestyle='--', label='assumed model minimum')
            ax.set_xlabel('Time [s]'); ax.set_ylabel('Minimum actual rotor speed [RPM]')
            ax.grid(alpha=.2)
        panel[0].set_title(f"{row['controller']} · startup · seed {row['sensor_seed']}")
        panel[1].set_title(f"{row['controller']} · full recorded trial")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=3, frameon=False)
    fig.savefig(output/'low_speed_domain.png', dpi=150)
    plt.close(fig)
    return True


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sources', type=Path, nargs='+')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    report = summarize(a.sources, a.output)
    print(f"Recorded {len(report['records'])} trials in {a.output}")


if __name__ == '__main__':
    main()
