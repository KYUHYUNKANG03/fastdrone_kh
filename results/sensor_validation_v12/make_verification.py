"""Build the v12 evidence index after the complete, validated 93-task study.

Run under the study's original runtime; it intentionally refuses a later source
tree. It indexes recorded outcomes, not a paper-readiness or reliability claim.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from control.arena import config_sha256
from scripts.sensor_preparation import checked_plan


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build():
    out = Path(__file__).resolve().parent
    prepared = ROOT/'results/prepared_sensor_validation_v11_final'
    plan = checked_plan(prepared)
    raw = json.loads((prepared/'results.json').read_text(encoding='utf-8'))
    data = json.loads((out/'final_analysis/validated/outcomes.json').read_text(encoding='utf-8'))
    diagnostics = json.loads((out/'final_analysis/diagnostics.json').read_text(encoding='utf-8'))
    assert raw['complete'] and len(raw['records']) == len(plan['tasks']) == 93
    assert config_sha256(plan) == raw['plan_sha256'] == data['plan_sha256'] == diagnostics['plan_sha256']
    assert diagnostics['analysis_source_sha256'] == sha(ROOT/'scripts/sensor_startup_diagnostics.py')
    assert {r['trial_id'] for r in data['records']} == {t['trial_id'] for t in plan['tasks']}
    assert len(data['records']) == 93 and not data['execution_counts'].get('pending', 0)
    identities, tests = set(), []
    for name in ('report_tests.xml', 'split_execution_tests.xml', 'startup_analysis_final_tests.xml'):
        path = out/name
        tree = ET.parse(path)
        cases = tree.findall('.//testcase')
        ids = {(c.get('classname'), c.get('name')) for c in cases}
        assert len(ids) == len(cases) and not identities & ids
        identities |= ids
        tests.append(dict(file=name, tests=len(cases), failures=len(tree.findall('.//failure')),
                          errors=len(tree.findall('.//error')), skipped=len(tree.findall('.//skipped')),
                          sha256=sha(path)))
    replays = []
    for name in ('reproduction_before_validation_v12', 'parallel_reproduction_v12/worker_1',
                 'parallel_reproduction_v12/worker_2'):
        path = ROOT/'results'/name/'comparison.json'
        replay = json.loads(path.read_text(encoding='utf-8'))
        replays.append(dict(local_file=path.relative_to(ROOT).as_posix(), sha256=sha(path),
            reproduction_pass=replay['reproduction_pass'], trajectory_bit_identical=replay['trajectory_bit_identical'],
            problems=replay['problems'], reference_trajectory_sha256=replay['reference']['trajectory_sha256'],
            actual_trajectory_sha256=replay['actual']['trajectory_sha256']))
    rows = data['records']
    stages = {}
    for stage in ('integration', 'gnss_startup'):
        members = [r for r in rows if r['stage'] == stage]
        stages[stage] = dict(trials=len(members), execution_status=dict(Counter(r['execution_status'] for r in members)),
            tracking_pass=sum(r.get('tracking_pass') is True for r in members),
            domain_pass=sum(r.get('model_domain_valid') is True for r in members),
            full_pass=sum(r.get('passed') is True for r in members),
            failure_reasons=dict(Counter(reason for r in members for reason in r.get('failure_reasons', []))),
            optimizer_failures=sum(r.get('optimizer_failures', 0) for r in members),
            full_duration=sum(r.get('simulated_seconds') == 12. for r in members))
    payload = dict(schema='sensor_validation_verification/1', stage='DEVELOPMENT',
        plan_sha256=config_sha256(plan), runtime_source_sha256=plan['runtime_source_sha256'],
        source_runtime_example_commit='4d30e025679c56b3acb72edb112d67c706b2cbeb',
        prepared_results_file_sha256=sha(prepared/'results.json'),
        merged_result_canonical_sha256=config_sha256(raw),
        merge_receipt=json.loads((prepared/'merge_receipt.json').read_text(encoding='utf-8')),
        execution_counts=data['execution_counts'], stages=stages,
        diagnostics_unavailable=[r['trial_id'] for r in rows if r.get('diagnostics_unavailable') or not r.get('trace')],
        nominal_repeats=diagnostics['nominal_repeats'], tests=tests,
        distinct_targeted_tests=len(identities), numerical_reproduction_checks=replays,
        unique_planned_tasks=93, additional_reference_replays=3,
        evidence_files_sha256={name: sha(out/name) for name in (
            'final_analysis/diagnostics.json', 'final_analysis/report.md',
            'final_analysis/validated/outcomes.json', 'final_analysis/validated/report.md',
            'final_analysis/startup_rotor.png', 'final_analysis/startup_rotor.svg')},
        verification_builder_sha256=sha(Path(__file__)), final_tuning_started=False, paper_ready=False,
        note='Fixed team gains and engineering sensor assumptions. All failures retained. '
             'Shared controls, quiet repetitions and nominal repeat checks are not independent reliability trials. '
             'Git metadata varies across non-runtime commits; the frozen runtime/configuration checks remain strict. '
             'The separate v13 portability captures are not pooled with this study.')
    with (out/'verification.json').open('x', encoding='utf-8') as file:
        json.dump(payload, file, indent=2, ensure_ascii=False, allow_nan=False)
        file.write('\n')
    print(json.dumps(dict(stages=stages, distinct_targeted_tests=len(identities)), indent=2))


if __name__ == '__main__':
    build()
