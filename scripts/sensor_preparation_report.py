"""Report a prepared DEVELOPMENT plan, including pending and failed trials."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.sensor_preparation import checked_plan
from scripts.sensor_reproduce import canonical_source_hashes
from scripts.sensor_envelope_report import trace_metrics, verify_feedback_binding, paired_outcome
from control.arena import config_sha256
from control.sensor_feedback_diagnostics import diagnose, trace_path
from control.validation_suite import plant_truth
from models.team_light.control.baseline_v2 import baseline_params, parameter_hash


def summarize(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    plan = checked_plan(source)
    results_file = source/'results.json'
    data = json.loads(results_file.read_text(encoding='utf-8')) if results_file.exists() else {
        'plan_sha256': config_sha256(plan), 'records': []}
    if data.get('plan_sha256') != config_sha256(plan):
        raise ValueError('result plan hash differs')
    tasks = {t['trial_id']: t for t in plan['tasks']}
    records = {r['trial_id']: r for r in data['records']}
    if len(records) != len(data['records']) or set(records)-set(tasks):
        raise ValueError('duplicate or unknown recorded trials')
    rows = []
    for tid, task in tasks.items():
        row = {k: v for k, v in task.items() if k != 'config'}
        record = records.get(tid)
        if record is None:
            row['execution_status'] = 'pending'
            rows.append(row)
            continue
        if record.get('input_config_sha256') != task['config_sha256']:
            raise ValueError('record input configuration differs')
        if any(record.get(k) != task[k] for k in ('controller', 'feedback', 'truth_trial_id')):
            raise ValueError('record identity differs')
        if record.get('sensor_seed') != task['seed']:
            raise ValueError('record seed differs')
        if record.get('scenario_id') not in (None, task['case']):
            raise ValueError('record scenario differs')
        row.update({k: v for k, v in record.items() if k != 'run_dir'})
        row['execution_status'] = ('execution_incomplete' if record.get('campaign_timed_out')
            or record.get('returncode', 0) != 0
            or record.get('simulated_seconds') is None else 'recorded')
        path = trace_path(record, results_file)
        if path is not None:
            manifest = json.loads((path.parent/'manifest.json').read_text(encoding='utf-8'))
            if manifest.get('config_sha256') != task['config_sha256']:
                raise ValueError('trace configuration differs')
            if any(canonical_source_hashes(manifest.get('source_sha256', {})).get(k) != v
                   for k, v in plan['runtime_source_sha256'].items()):
                raise ValueError('trace runtime differs')
            verify_feedback_binding(task['config'], manifest, record)
            scenario = next(s for s in task['config']['scenarios'] if s['id'] == task['case'])
            if record.get('factors', {}) != scenario.get('perturbation', {}):
                raise ValueError('physical perturbation differs')
            if parameter_hash(plant_truth(baseline_params(), record)) != record.get('truth_parameter_sha256'):
                raise ValueError('physical parameter hash differs')
            row['trace'] = os.path.relpath(path, output)
            row.update(trace_metrics(path, task['feedback']))
            if 'diagnostics_unavailable' not in row:
                row['domain_diagnostics'] = {k: v for k, v in diagnose(record, results_file).items()
                                             if k not in ('source', 'trace')}
        elif row['execution_status'] == 'recorded':
            row['diagnostics_unavailable'] = 'missing_or_ambiguous_trace'
        rows.append(row)
    by_id = {r['trial_id']: r for r in rows}
    for row in rows:
        if row['truth_trial_id']:
            truth = by_id[row['truth_trial_id']]
            statuses = (truth['execution_status'], row['execution_status'])
            row['paired_outcome'] = ('pending' if 'pending' in statuses else
                'execution_incomplete' if 'execution_incomplete' in statuses else paired_outcome(truth, row))
    counts = Counter(r['execution_status'] for r in rows)
    report = dict(schema='sensor_preparation_report/1', stage='DEVELOPMENT',
        plan_sha256=config_sha256(plan), source=os.path.relpath(source, output),
        report_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        expected=len(rows), execution_counts=dict(counts), records=rows,
        note='Pending cases are not successes or executed failures. Failures remain in the executed '
             'denominator. Shared deterministic controls and quiet repeats are not independent trials. '
             'Tracking and propulsion-domain verdicts remain separate. No hardware limit or reliability claim.')
    if output.exists() and any(output.iterdir()):
        raise ValueError('report output must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    (output/'outcomes.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    lines = ['# Prepared sensor validation', '', '**Development only; fixed team gains.**', '',
        f'Planned: {len(rows)}. Recorded: {counts["recorded"]}. Execution incomplete: '
        f'{counts["execution_incomplete"]}. Pending: {counts["pending"]}.', '', report['note'], '',
        'RMSE covers only recorded time. Read duration and stop reason before comparing failures. '
        'Diagnostic availability never changes an original trial verdict.', '',
        '| Stage | Controller | Case / condition | Seed | Execution | Track / domain | Duration [s] | Pair |',
        '|---|---|---|---:|---|---|---:|---|']
    for r in rows:
        lines.append(f'| {r["stage"]} | {r["controller"]} | {r["case"]} / {r["condition"]} | '
            f'{r["seed"]} | {r["execution_status"]} | {r.get("tracking_pass", "—")} / '
            f'{r.get("model_domain_valid", "—")} | {r.get("simulated_seconds", "—")} | '
            f'{r.get("paired_outcome", "control")} |')
    (output/'report.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='folder containing plan.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    report = summarize(args.source, args.output)
    print(json.dumps(dict(expected=report['expected'], execution_counts=report['execution_counts']), indent=2))


if __name__ == '__main__':
    main()
