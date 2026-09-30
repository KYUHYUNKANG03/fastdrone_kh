"""One/two-worker V13/F13 execution of an unchanged prepared GNSS stage.

The immutable scientific plan is retained. A separate executor contract records
this orchestration source and worker count. One parent owns the results/lock;
workers run independent subprocesses and write distinct task directories.
M17 and integration are intentionally not supported by this executor.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.arena import config_sha256
from control.sensor_campaign import run_campaign
from control.sensor_matching_screen import write_atomic
from scripts.sensor_preparation import checked_plan
from scripts.sensor_preflight import audit, configure_process


def execute(output, runner=run_campaign, workers=2):
    if workers not in (1, 2):
        raise ValueError('workers must be 1 or 2')
    output = Path(output).resolve()
    plan = checked_plan(output)
    selected = {t['trial_id'] for t in plan['tasks'] if t['stage'] == 'gnss_startup'}
    selected |= {t['truth_trial_id'] for t in plan['tasks']
                 if t['trial_id'] in selected and t['truth_trial_id']}
    tasks = [t for t in plan['tasks'] if t['trial_id'] in selected]
    if any(t['controller'] not in ('V13', 'F13') for t in tasks):
        raise ValueError('parallel executor only supports V13/F13')
    contract = dict(schema='sensor_parallel_executor/1', stage='DEVELOPMENT', workers=workers,
        plan_sha256=config_sha256(plan), task_ids=[t['trial_id'] for t in tasks],
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        note='Independent single-threaded child simulations; one parent result writer. No M17.')
    lock = output/'run.lock'
    with lock.open('x', encoding='utf-8') as file:
        file.write(json.dumps(dict(pid=os.getpid(), stage='gnss_startup', executor='parallel'))+'\n')
    try:
        path = output/'parallel_executor.json'
        if path.exists() and json.loads(path.read_text(encoding='utf-8')) != contract:
            raise ValueError('parallel executor contract changed; use a new directory')
        write_atomic(path, contract)
        return _execute(output, plan, tasks, runner, workers)
    finally:
        lock.unlink()


def _execute(output, plan, tasks, runner, workers):
    path = output/'results.json'
    records = {}
    if path.exists():
        previous = json.loads(path.read_text(encoding='utf-8'))
        if previous.get('plan_sha256') != config_sha256(plan):
            raise ValueError('result plan hash differs')
        records = {r['trial_id']: r for r in previous['records']}
        all_tasks = {t['trial_id']: t for t in plan['tasks']}
        if len(records) != len(previous['records']) or set(records)-set(all_tasks):
            raise ValueError('duplicate or unknown recorded trials')
        if any(r.get('input_config_sha256') != all_tasks[tid]['config_sha256'] for tid, r in records.items()):
            raise ValueError('recorded input configuration differs')

    def save():
        # Stable task order irrespective of worker completion order.
        doc = dict(schema='sensor_preparation_results/1', stage='DEVELOPMENT',
            plan_sha256=config_sha256(plan), expected_trials=len(plan['tasks']),
            complete=len(records) == len(plan['tasks']),
            records=[records[t['trial_id']] for t in plan['tasks'] if t['trial_id'] in records])
        write_atomic(path, doc)
        return doc

    def run_one(task):
        tid = task['trial_id']
        storage_id = 't_'+hashlib.sha256(tid.encode()).hexdigest()[:20]
        folder = output/'inputs'/storage_id
        folder.mkdir(parents=True, exist_ok=True)
        cfg, profile = folder/'arena.json', folder/'sensors.json'
        write_atomic(cfg, task['config'])
        if task['feedback'] == 'sensors':
            write_atomic(profile, task['config']['sensor_feedback']['profile'])
        result = runner(cfg, profile, output/'runs'/storage_id, cases=[task['case']],
            controllers=[task['controller']], seed=task['seed'], feedback=task['feedback'],
            timeout_s=plan['timeout_s'])
        if len(result['records']) != 1:
            raise ValueError(f'{tid}: expected one recorded trial')
        return dict(result['records'][0], trial_id=tid, stage=task['stage'],
            condition=task['condition'], truth_trial_id=task['truth_trial_id'],
            input_config_sha256=task['config_sha256'])

    pending = [t for t in tasks if t['trial_id'] not in records]
    save()
    with ThreadPoolExecutor(max_workers=workers) as pool:
        # Bounded batches: stop scheduling after a broken command, but preserve
        # the other in-flight result before raising. Completed failures persist.
        for start in range(0, len(pending), workers):
            pair = pending[start:start+workers]
            futures = []
            for task in pair:
                print('START '+task['trial_id'], flush=True)
                futures.append(pool.submit(run_one, task))
            errors = []
            for task, future in zip(pair, futures):
                try:
                    row = future.result()
                    records[task['trial_id']] = row
                    save()
                    print(f'DONE {task["trial_id"]}: passed={row.get("passed")} '
                          f'duration={row.get("simulated_seconds")}', flush=True)
                    if 'no_trial_output' in row.get('failure_reasons', []):
                        errors.append(task['trial_id']+': no trial output')
                except Exception as exc:
                    errors.append(f'{task["trial_id"]}: {type(exc).__name__}: {exc}')
            if errors:
                raise RuntimeError('; '.join(errors))
    return save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='existing preparation plan directory')
    parser.add_argument('--workers', type=int, choices=(1, 2), default=2)
    args = parser.parse_args(argv)
    configure_process()
    plan = checked_plan(args.output)
    check = audit(plan['base_config'])
    if not check['development_ready']:
        print(json.dumps(check, indent=2))
        return 1
    execute(args.output, workers=args.workers)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
