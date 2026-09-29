"""Summarize completed development screens without dropping failed trials.

Raw trajectories stay local. The JSON output embeds experiment contracts and
relative trace paths so published metrics retain their configuration provenance.
Run against the same runtime revision that produced the campaigns.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from control.sensor_binding import runtime_source_hashes
from control.sensor_feedback_diagnostics import diagnose, trace_path
from control.sensor_filter_report import startup


def summarize(sources, output):
    output = Path(output).resolve()
    contracts, rows = [], []
    hashes = runtime_source_hashes()
    ids = set()
    for source in map(lambda p: Path(p).resolve(), sources):
        doc = json.loads(source.read_text(encoding='utf-8'))
        if not doc.get('complete') or len(doc['records']) != doc['expected_trials']:
            raise ValueError(f'incomplete experiment: {source}')
        if doc['contract']['runtime_source_sha256'] != hashes:
            raise ValueError(f'runtime differs from recorded experiment: {source}')
        contracts.append(dict(source=str(source.relative_to(ROOT)), **doc['contract']))
        for record in doc['records']:
            identity = (doc['contract'].get('base_config_sha256'), record['trial_id'])
            if identity in ids:
                raise ValueError(f'duplicate trial: {record["trial_id"]}')
            ids.add(identity)
            keys = ('trial_id', 'variant', 'fault', 'controller', 'scenario_id', 'sensor_seed',
                    'sensor_profile', 'sensor_profile_sha256', 'config_sha256', 'truth_parameter_sha256',
                    'controller_model_sha256', 'trajectory_sha256', 'simulated_seconds',
                    'tracking_pass', 'model_domain_valid', 'passed', 'failure_reasons',
                    'campaign_timed_out', 'stop_reason', 'paper_failed', 'paper_reasons',
                    'rmse_z', 'rmse_velocity', 'max_omega', 'prop_domain_outside_fraction',
                    'optimizer_calls', 'optimizer_failures', 'command_total_variation',
                    'estimator_diagnostics')
            row = {k: record.get(k) for k in keys}
            row['condition'] = (record.get('sensor_profile') if record['variant'] == 'baseline'
                                and str(record.get('sensor_profile', '')).startswith('rotor_')
                                else record['variant'])
            row['source'] = str(source.relative_to(ROOT))
            path = trace_path(record, source)
            row['trace'] = str(path.relative_to(ROOT)) if path else None
            detail = diagnose(record, source)
            row['diagnostics'] = ({k:v for k,v in detail.items() if k not in ('source', 'trace')}
                                  if detail else None)
            if path:
                manifest = json.loads((path.parent/'manifest.json').read_text(encoding='utf-8'))
                if any(manifest.get('source_sha256', {}).get(k) != v for k, v in hashes.items()):
                    raise ValueError(f'trial runtime differs from experiment: {path}')
                if manifest.get('sensor_profile_sha256') != record.get('sensor_profile_sha256'):
                    raise ValueError(f'trial sensor profile differs from experiment: {path}')
                row['config_sha256'] = manifest['config_sha256']
                row['controller_settings'] = manifest.get('controller_settings', {})
                row['startup'] = startup(record, source)
                with np.load(path, allow_pickle=False) as trace:
                    n = min(len(trace['xs']), len(trace['xs_est']))
                    truth, estimate = trace['xs'][:n], trace['xs_est'][:n]
                    error = estimate-truth
                    angle = (Rotation.from_quat(truth[:, 6:10]).inv() *
                             Rotation.from_quat(estimate[:, 6:10])).magnitude()
                    rms = lambda a: float(np.sqrt(np.mean(np.sum(a*a, axis=1))))
                    row['estimation_rms'] = dict(position_m=rms(error[:, :3]),
                        velocity_m_s=rms(error[:, 3:6]), body_rate_rad_s=rms(error[:, 10:13]),
                        rotor_rad_s=rms(error[:, 13:]),
                        attitude_deg=float(np.rad2deg(np.sqrt(np.mean(angle*angle)))))
            rows.append(row)
    output.mkdir(parents=True, exist_ok=True)
    data = dict(schema='sensor_group_report/1', stage='DEVELOPMENT', contracts=contracts, records=rows)
    (output/'outcomes.json').write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    text = ['# Sensor feedback group comparisons', '',
        '**Development diagnosis; not final tuning, hardware specifications, or paper results.**', '',
        'Each variant uses the same plant and unchanged controller defaults. `quiet_sampled` '
        'removes generated sensor noise, biases, dropouts and transport latency, and uses a '
        'near-instant rotor measurement filter. `imu_only`, `navigation_only`, and `rotor_only` '
        'restore the corresponding nominal group. `baseline` restores all groups. '
        'Sampling rates, sensor clipping, estimator type, initial-state policy, navigation '
        'measurement covariances, process-noise assumptions and preflight covariance stay fixed. '
        'The quiet controls deliberately overestimate sensor uncertainty; they are diagnostic controls, '
        'not claims of perfectly accurate state estimation. No controller receives truth-state feedback. '
        'The optional timing conditions keep all nominal sensor errors: `rotor_unfiltered` removes '
        'only extra telemetry smoothing (tau = 1 microsecond), `rotor_no_latency` removes only '
        'telemetry transport latency, and `rotor_direct` removes both. Neither changes physical motor lag. '
        '`rotor_projected_*` uses a telemetry anchor at its sample time and recorded commands to '
        'predict the present rotor state, retaining nominal sensor noise and 4 ms latency. '
        '`warm` conditions replay 0.1 s of noisy steady-trim rotor prehistory; `cold` conditions '
        'start with no arrived rotor measurement. `rotor_telemetry_warm` keeps the 20 ms measurement '
        'filter; `rotor_unfiltered_warm` uses 1 microsecond. Projected tau10ms/tau40ms conditions '
        'change only the declared observer motor time constant from its nominal 20 ms.', '',
        'All outcomes, including early stops and timeouts, are retained. RMSE on a stopped run '
        'covers only its recorded prefix and is not directly comparable with full-duration RMSE. '
        'These runs do not estimate failure probabilities or a maximum usable sensor specification.', '',
        '| Controller | Variant | Seed | Duration [s] | Tracking | Model domain | z RMSE [m] | v RMSE [m/s] | Solver failures |',
        '|---|---|---:|---:|---|---|---:|---:|---:|']
    number = lambda v: '—' if v is None else f'{v:.3f}'
    for r in rows:
        text.append(f"| {r['controller']} | {r['condition']} | {r['sensor_seed']} | "
                    f"{number(r['simulated_seconds'])} | {r['tracking_pass']} | {r['model_domain_valid']} | "
                    f"{number(r['rmse_z'])} | {number(r['rmse_velocity'])} | {r['optimizer_failures']} |")
    text += ['', 'Cold delayed-telemetry cases start without an available measurement; warm '
             'cases use an explicit steady-trim measurement/command prehistory. This is not a '
             'simulation of a complete takeoff or real preflight procedure. Startup diagnostics '
             'record allocations made before first telemetry arrival. Compare warm/cold and '
             'observer changes separately before interpreting any ESC latency tolerance.', '',
             'Raw traces remain in the local checkout at the relative paths in `outcomes.json`. '
             'The JSON includes exact experiment contracts and runtime hashes. Reproduce each campaign '
             'with the axes in its contract, using a new output directory.', '', '![Recorded outcomes](comparison.png)', '']
    (output/'report.md').write_text('\n'.join(text), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, max(4, .32*len(rows)+1.7)), layout='constrained')
    labels = [f"{r['controller']} · {r['condition']} · seed {r['sensor_seed']}" for r in rows]
    ax.barh(labels, [r['simulated_seconds'] or 0 for r in rows],
            color=['#18795d' if r['passed'] else '#b14434' for r in rows])
    ax.invert_yaxis()
    ax.set_xlabel('Recorded simulation duration [s]')
    ax.set_title('Development screen · green = tracking + model domain pass\n'
                 'red = failure; full duration alone is not success')
    fig.savefig(output/'comparison.png', dpi=150)
    plt.close(fig)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sources', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.sources, args.output)
    print(f"Saved {len(result['records'])} development outcomes to {args.output}")


if __name__ == '__main__':
    main()
