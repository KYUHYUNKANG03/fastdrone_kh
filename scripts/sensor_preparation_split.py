"""Split prepared stages on one host, then merge identical-plan records safely.

Snapshots reference existing absolute trace paths; this is not a cross-machine
archive tool. Snapshotting an active parent is read-only. Merge requires all
participating writers to have stopped and locks both directories.
"""
import argparse
from contextlib import ExitStack, contextmanager
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.arena import config_sha256
from control.sensor_matching_screen import write_atomic
from scripts.sensor_preparation import checked_plan, save_plan


def read_records(folder, plan):
    path = folder/'results.json'
    text = path.read_text(encoding='utf-8') if path.exists() else None
    doc = json.loads(text) if text else dict(plan_sha256=config_sha256(plan), records=[])
    if doc.get('plan_sha256') != config_sha256(plan):
        raise ValueError('result plan hash differs')
    tasks = {t['trial_id']: t for t in plan['tasks']}
    records = {r['trial_id']: r for r in doc['records']}
    if len(records) != len(doc['records']) or set(records)-set(tasks):
        raise ValueError('duplicate or unknown recorded trials')
    if any(r.get('input_config_sha256') != tasks[tid]['config_sha256'] for tid, r in records.items()):
        raise ValueError('recorded input configuration differs')
    return records, hashlib.sha256(text.encode()).hexdigest() if text is not None else None


def result_document(plan, records):
    return dict(schema='sensor_preparation_results/1', stage='DEVELOPMENT',
        plan_sha256=config_sha256(plan), expected_trials=len(plan['tasks']),
        complete=len(records) == len(plan['tasks']),
        records=[records[t['trial_id']] for t in plan['tasks'] if t['trial_id'] in records])


def snapshot(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('snapshot output must be a new directory')
    plan = checked_plan(source)
    # The parent atomically replaces results.json; capture one complete version.
    records, digest = read_records(source, plan)
    save_plan(output, plan)
    write_atomic(output/'results.json', result_document(plan, records))
    receipt = dict(schema='sensor_preparation_snapshot/1', source=str(source),
        plan_sha256=config_sha256(plan), source_results_sha256=digest,
        copied_trial_ids=list(records),
        note='Same-host snapshot; traces remain at their original absolute paths.')
    write_atomic(output/'snapshot.json', receipt)
    return receipt


@contextmanager
def exclusive(folder):
    lock = folder/'run.lock'
    with lock.open('x', encoding='utf-8') as file:
        file.write(json.dumps(dict(pid=os.getpid(), stage='merge'))+'\n')
    try:
        yield
    finally:
        lock.unlink()


def merge(target, source):
    target, source = Path(target).resolve(), Path(source).resolve()
    if target == source:
        raise ValueError('merge requires distinct directories')
    with ExitStack() as stack:
        for folder in sorted((target, source)):
            stack.enter_context(exclusive(folder))
        plan = checked_plan(target)
        if checked_plan(source) != plan:
            raise ValueError('different preparation plans cannot be merged')
        primary, primary_hash = read_records(target, plan)
        incoming, incoming_hash = read_records(source, plan)
        shared = set(primary) & set(incoming)
        for tid in shared:
            if primary[tid] != incoming[tid]:
                raise ValueError(f'conflicting duplicate trial: {tid}')
        result = result_document(plan, primary | incoming)
        receipt = dict(schema='sensor_preparation_merge/1', plan_sha256=config_sha256(plan),
            target_before_sha256=primary_hash, source=str(source),
            source_results_sha256=incoming_hash,
            shared_identical_trials=sorted(shared), added_trials=sorted(set(incoming)-set(primary)),
            merged_result_sha256=config_sha256(result),
            source_executor=(json.loads((source/'parallel_executor.json').read_text(encoding='utf-8'))
                             if (source/'parallel_executor.json').exists() else None),
            merge_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        write_atomic(target/'results.json', result)
        write_atomic(target/'merge_receipt.json', receipt)
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='command', required=True)
    split = subs.add_parser('snapshot')
    split.add_argument('--source', type=Path, required=True)
    split.add_argument('--output', type=Path, required=True)
    join = subs.add_parser('merge')
    join.add_argument('--target', type=Path, required=True)
    join.add_argument('--source', type=Path, required=True)
    args = parser.parse_args(argv)
    result = snapshot(args.source, args.output) if args.command == 'snapshot' else merge(args.target, args.source)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
