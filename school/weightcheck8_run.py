"""weightcheck8 — ablation8의 "IMU 지연 강건성은 인터페이스에서 온다"가 비용 가중치 차이 때문은 아닌지 확인한다.

F13(w_omega 0.25)과 V13(w_omega 1.0)은 각자 튜닝되어 상태 비용 가중치가 다르다. V13-0은 V13 가중치를 쓰므로
F13 -> V13-0 차이에는 인터페이스와 가중치가 섞여 있다. 상태 가중치(w_v, w_omega, terminal)는 두 구조에서
같은 뜻이므로 바꿔 끼워 본다(입력 가중치 r_dev, r_rate는 입력 척도가 달라 그대로 둔다):
  F13-w1     : F13 튜닝값 + w_omega 1.0(V13 값)
  V13-0-w025 : V13-0(A0, 피드백 없음, S0) + w_omega 0.25(F13 값)
  F13-sV     : F13 튜닝값 + 상태 가중치 전부 V13 값(w_v 5, w_omega 1, terminal 10)
조건·시나리오 묶음은 GROUPS(우선순위 순): IMU 5 ms·명목 센서 x V_H 횡돌풍 +10 m/s·0->V_H 가속(rho 1), 시드 1020~1039.
(첫 판 results/weightcheck8은 IMU 5 ms 횡돌풍의 F13-w1·V13-0-w025를 끝냈다 — 이 판은 그 둘을 다시 돌리지 않는다.) 기준 설정·기록·시나리오·시드·
센서 프로파일·시행 함수와 판정은 sensor8/ablation8과 같다. 결과 폴더와 레인 규칙도 ablation8과 같다.

    python weightcheck8_run.py plan     --root <fds> --out <fds>/results/weightcheck8
    python weightcheck8_run.py lanes    --out <fds>/results/weightcheck8 --pc CLOUD --n 2
    python weightcheck8_run.py run-lane --root <fds> --out <fds>/results/weightcheck8 --pc CLOUD --index 0
    python weightcheck8_run.py summary  --out <fds>/results/weightcheck8 --pc CLOUD
"""
import argparse
from copy import deepcopy
import glob
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sensor8_run as s8   # noqa: E402

PLAN_SCHEMA = 'weightcheck8-plan/1'
LANES_SCHEMA = 'weightcheck8-lanes/1'
V13_0 = dict(alloc_mode='A0', alloc_feedback=False, time_align='S0')
VARIANTS = {
    'F13-w1': dict(label='F13', flags=None, cost=dict(w_omega=1.0)),
    'V13-0-w025': dict(label='V13', flags=V13_0, cost=dict(w_omega=0.25)),
    'F13-sV': dict(label='F13', flags=None, cost=dict(w_v=5.0, w_omega=1.0, terminal=10.0)),
}
GUST, ACCEL = 'main_gust_lateral_+10_V_H', 'main_ref_accel_VH_rho1'
IMU5 = 'imu_latency_5ms'
GROUPS = [(IMU5, ACCEL, 'F13-w1'), (IMU5, ACCEL, 'V13-0-w025'),
          ('nominal', GUST, 'F13-w1'), ('nominal', GUST, 'V13-0-w025'),
          ('nominal', ACCEL, 'F13-w1'), ('nominal', ACCEL, 'V13-0-w025'),
          (IMU5, GUST, 'F13-sV')]


def runner_sha256():
    return s8.sha256_bytes(Path(__file__).read_bytes() + Path(s8.__file__).read_bytes())


def setup(root):
    """기준 설정·시나리오·변형별 파생 설정과 overrides. 계획과 실행이 같은 함수를 쓴다."""
    from control.arena import load_config, config_sha256, build_scenarios
    from control.arena_factory import ArenaFactory
    from control.arena_design_check import tuned_overrides
    from control.validation_suite import baseline_params
    from control import main_experiment as me
    problems = s8.check_records(root)
    if problems:
        raise SystemExit('records:\n  - ' + '\n  - '.join(problems))
    base = load_config(root/s8.BASE)
    if config_sha256(base) != s8.BASE_SHA:
        raise SystemExit(f'{s8.BASE}: hash {config_sha256(base)[:12]} is not the tune8 config')
    native = baseline_params()
    spec, pool = s8.scenario_dicts(root, base, native)
    scen = [s for s in pool if s['id'] in (GUST, ACCEL)]
    tuned, used = tuned_overrides(base, root/s8.RUN_DIR, ['V13', 'F13'])
    configs, overrides = {}, {}
    for v, d in VARIANTS.items():
        configs[v] = me._derived(base, scen, d['flags'])
        ov = deepcopy(tuned)
        for key, value in d['cost'].items():
            if key not in ov[d['label']]['cost_weights']:
                raise SystemExit(f'{v}: {key} is not a {d["label"]} cost weight')
            ov[d['label']]['cost_weights'][key] = float(value)
        overrides[v] = ov
    cp = ArenaFactory(base, native).cp
    built = {s.id: s for s in build_scenarios(configs['F13-w1'], cp, native)}
    return dict(base=base, native=native, spec=spec, configs=configs, overrides=overrides, built=built,
                tuned=tuned, config_sha={v: config_sha256(c) for v, c in configs.items()})


def build_plan(root):
    root = s8.use_root(root)
    from control.arena import excluded_from
    from control.arena_sensors import load_sensor_profile
    from control.sensor_binding import resolve_feedback
    from control.validation_suite import source_hashes
    st = setup(root)
    nominal = resolve_feedback(st['base'])
    conditions = []
    for name in dict.fromkeys(c for c, _, _ in GROUPS):
        cond = next(c for c in s8.CONDITIONS if c['name'] == name)
        profile = s8.apply_changes(nominal.profile, cond['changes'], root)
        if cond['changes']:
            profile['name'] = f"{nominal.profile['name']}+{name}"
        profile = load_sensor_profile(profile)
        conditions.append(dict(name=name, note=cond['note'], changes=cond['changes'], profile=profile,
                               profile_sha256=resolve_feedback(st['base'], mode='sensors', profile=profile)
                               .metadata['sensor_profile_sha256']))
    trials = []
    for order, (cname, sid, v) in enumerate(GROUPS):
        label = VARIANTS[v]['label']
        reason = excluded_from(st['configs'][v], label, st['built'][sid])
        if reason:
            raise SystemExit(f'{v} excluded from {sid}: {reason}')
        for seed in s8.SEEDS:
            trials.append(dict(trial_id=f'{cname}/{sid}/{v}/seed_{seed}', condition=cname, order=order,
                               scenario_id=sid, variant=v, controller=label, sensor_seed=seed,
                               est_seconds=s8.EST[sid][label]))
    return dict(schema=PLAN_SCHEMA,
                purpose='weightcheck8: swap state cost weights between F13 and V13-0 (nominal and IMU latency 5 ms, '
                        'V_H lateral gust and 0->V_H accel, seeds 1020-1039) to separate interface from cost-weight effects.',
                groups=[dict(order=i, condition=c, scenario_id=s, variant=v) for i, (c, s, v) in enumerate(GROUPS)],
                base_config=dict(path=s8.BASE, config_sha256=s8.BASE_SHA),
                tuned=dict(run_dir=s8.RUN_DIR, records={k: s8.RECORDS[k] for k in ('V13', 'F13')},
                           values=st['tuned']),
                variants=VARIANTS, cost_weights={v: st['overrides'][v][VARIANTS[v]['label']] for v in VARIANTS},
                derived_config_sha256=st['config_sha'], seeds=s8.SEEDS, trace_seed=s8.TRACE_SEED,
                source_sha256=source_hashes(), runner_sha256=runner_sha256(),
                conditions=conditions, trials=trials)


def plan_path(out):
    return Path(out)/'weightcheck8.plan.json'


def load_plan(out):
    plan = json.loads(plan_path(out).read_text(encoding='utf-8'))
    if plan.get('schema') != PLAN_SCHEMA:
        raise SystemExit('not a weightcheck8 plan')
    return plan, s8.sha256_file(plan_path(out))


def cmd_plan(root, out):
    plan = build_plan(root)
    text = json.dumps(plan, ensure_ascii=False, indent=1) + '\n'
    path = plan_path(out)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_text(encoding='utf-8') != text:
        raise SystemExit(f'{path} exists and differs - use a new output folder')
    s8.write_atomic(path, text)
    for v in VARIANTS:
        print(v, plan['cost_weights'][v])
    print(f"total {len(plan['trials'])} trials, plan sha256 {s8.sha256_file(path)}")


def lanes_path(out, pc):
    return Path(out)/f'{pc}.lanes.json'


def cmd_lanes(out, pc, n):
    plan, plan_sha = load_plan(out)
    loads, lanes = [0.0]*n, [[] for _ in range(n)]
    for order in sorted({t['order'] for t in plan['trials']}):
        group = [t for t in plan['trials'] if t['order'] == order]
        for t in sorted(group, key=lambda t: (-t['est_seconds'], t['trial_id'])):
            k = min(range(n), key=lambda i: (loads[i], i))
            lanes[k].append(t['trial_id'])
            loads[k] += t['est_seconds']
    doc = dict(schema=LANES_SCHEMA, plan_sha256=plan_sha, pc=pc, n=n,
               lanes=[dict(index=i, est_hours=loads[i]/3600.0, trials=lanes[i]) for i in range(n)])
    s8.write_atomic(lanes_path(out, pc), json.dumps(doc, ensure_ascii=False, indent=1) + '\n')
    for lane in doc['lanes']:
        print(f"lane {lane['index']}: {len(lane['trials'])} trials, ~{lane['est_hours']:.1f} h (Windows estimate)")


class Runner:
    def __init__(self, root, plan, plan_sha):
        self.root = s8.use_root(root)
        from control.arena_factory import ArenaFactory
        from control.validation_suite import source_hashes
        self.plan, self.plan_sha = plan, plan_sha
        problems = []
        if source_hashes() != plan['source_sha256']:
            problems.append('control/configs source hashes differ from the plan')
        if runner_sha256() != plan['runner_sha256']:
            problems.append('weightcheck8_run.py or sensor8_run.py differs from the one that made the plan')
        if problems:
            raise SystemExit('refusing to run:\n  - ' + '\n  - '.join(problems))
        st = setup(self.root)
        if st['config_sha'] != plan['derived_config_sha256']:
            raise SystemExit('derived configs differ from the plan')
        for v in VARIANTS:
            if st['overrides'][v][VARIANTS[v]['label']] != plan['cost_weights'][v]:
                raise SystemExit(f'{v}: cost weights differ from the plan')
        self.st = st
        self.factories = {v: ArenaFactory(st['configs'][v], st['native'], overrides=st['overrides'][v])
                          for v in VARIANTS}
        self.conditions = {c['name']: c for c in plan['conditions']}
        self.trials = {t['trial_id']: t for t in plan['trials']}
        self.source = plan['source_sha256']
        self._trim = {}

    def hashes(self, variant):
        label = VARIANTS[variant]['label']
        return dict(plan_sha256=self.plan_sha, base_config_sha256=self.plan['base_config']['config_sha256'],
                    derived_config_sha256=self.plan['derived_config_sha256'][variant],
                    tuned_record_sha256=s8.RECORDS[label], runner_sha256=self.plan['runner_sha256'])

    def run_trial(self, trial):
        import numpy as np
        from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
        from control.validation_suite import run_trial as suite_run
        from control import main_experiment as me
        v = trial['variant']
        label = VARIANTS[v]['label']
        config = self.st['configs'][v]
        s = self.st['built'][trial['scenario_id']]
        cond = self.conditions[trial['condition']]
        if s.id not in self._trim:
            self._trim[s.id] = me.trim_status(self.st['native'], s)
        status, detail = self._trim[s.id]
        row = dict(trial_id=trial['trial_id'], condition=cond['name'], scenario_id=s.id, scenario_type=s.type,
                   controller=label, variant=v, cost_weights=self.plan['cost_weights'][v], case=s.cases[0],
                   window=list(s.window), sensor_seed=trial['sensor_seed'], **self.hashes(v),
                   trim_detail={f'{k:g}': d for k, d in detail.items()},
                   authority_boundary=me.authority_boundary(detail, self.st['spec'], self.st['base']))
        if status:
            row.update(skipped=True, skip_reason='vehicle_limit')
            return row, None
        limits = Acceptance(**config['acceptance'])
        paper = PaperCriteria(**config['paper_criteria'])
        try:
            metrics, result, log = suite_run(self.factories[v], label, s.profile, s.cases[0], limits,
                                             feedback='sensors', sensor_profile=cond['profile'],
                                             sensor_seed=trial['sensor_seed'])
            metrics.update(paper_evaluate(result, s.profile, paper, window=s.window, solve_log=log,
                                          n_max=self.st['native']['n_max']))
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            metrics = dict(passed=False, failure_reasons=['setup_error'], stop_reason=f'{type(exc).__name__}: {exc}')
            result = None
        row.update(skipped=False, skip_reason=None, **metrics)
        row['estimation_error'] = s8.estimation_error(result)
        if result is not None and row.get('sensor_profile_sha256') != cond['profile_sha256']:
            raise SystemExit(f"{trial['trial_id']}: sensor profile differs from the plan")
        return row, result


def cmd_run_lane(root, out, pc, index, only=None):
    import numpy as np
    plan, plan_sha = load_plan(out)
    if only is None:
        lanes = json.loads(lanes_path(out, pc).read_text(encoding='utf-8'))
        if lanes.get('schema') != LANES_SCHEMA or lanes['plan_sha256'] != plan_sha:
            raise SystemExit('lanes file does not match the plan')
        ids = lanes['lanes'][index]['trials']
    else:
        ids = [only]
    runner = Runner(root, plan, plan_sha)
    from control.main_distributed import machine_fingerprint
    from control.validation_suite import json_safe, git_state
    folder = Path(out)/pc/f'lane{index}'/'trials'
    folder.mkdir(parents=True, exist_ok=True)
    traces = Path(out)/pc/'traces'
    traces.mkdir(parents=True, exist_ok=True)
    for stale in glob.glob(str(folder/'*.tmp')) + glob.glob(str(traces/f'*.lane{index}.tmp')):
        os.remove(stale)
    revision, dirty = git_state()
    machine = machine_fingerprint()
    for trial_id in ids:
        trial = runner.trials[trial_id]
        path = folder/(s8.safe_name(trial_id)+'.json')
        if path.exists():
            done = json.loads(path.read_text(encoding='utf-8'))
            want = runner.hashes(trial['variant'])
            if {k: done.get(k) for k in want} != want or done.get('source_sha256') != runner.source:
                raise SystemExit(f'{path}: finished with different hashes - refusing to mix results')
            continue
        started = s8._utc()
        try:
            row, result = runner.run_trial(trial)
        except Exception as exc:
            import traceback
            row = dict(trial_id=trial_id, condition=trial['condition'], scenario_id=trial['scenario_id'],
                       controller=trial['controller'], variant=trial['variant'], sensor_seed=trial['sensor_seed'],
                       **runner.hashes(trial['variant']), skipped=False, skip_reason=None, passed=False,
                       tracking_pass=False, model_domain_valid=False, failure_reasons=['runner_exception'],
                       stop_reason=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc()[-4000:])
            result = None
        if result is not None and trial['sensor_seed'] == s8.TRACE_SEED:
            npz = traces/(s8.safe_name(trial_id)+'.npz')
            tmp = traces/(s8.safe_name(trial_id)+f'.lane{index}.tmp')
            with open(tmp, 'wb') as stream:
                np.savez_compressed(stream, **{k: v for k, v in result.items()
                                               if k not in ('error', 'navigation_trace', 'estimator_diagnostics',
                                                            'sensor_feedback', 'sensor_profile')})
            os.replace(tmp, npz)
        row.update(lane=index, machine=machine, git_revision=revision, git_dirty=dirty,
                   source_sha256=runner.source, started_utc=started, finished_utc=s8._utc())
        s8.write_atomic(path, json.dumps(json_safe(row), ensure_ascii=False, allow_nan=False))
        print(f'[{index}] {trial_id}: {row.get("tracking_pass")}', flush=True)


def done_rows(out, pc):
    rows = {}
    for path in glob.glob(str(Path(out)/pc/'lane*'/'trials'/'*.json')):
        row = json.loads(Path(path).read_text(encoding='utf-8'))
        rows[row['trial_id']] = row
    return rows


def cmd_summary(out, pc):
    plan, plan_sha = load_plan(out)
    rows = done_rows(out, pc)
    lines = [f'# weightcheck8 summary ({pc})', '', f'plan sha256 {plan_sha[:12]}, {len(rows)}/{len(plan["trials"])} done',
             '', '| condition | scenario | variant | pass |', '|---|---|---|---:|']
    for g in plan['groups']:
        got = [rows[t['trial_id']] for t in plan['trials'] if t['order'] == g['order'] and t['trial_id'] in rows]
        cell = f"{sum(bool(r.get('tracking_pass')) for r in got)}/{len(got)}" if got else '-'
        lines.append(f"| {g['condition']} | {g['scenario_id'].replace('main_', '')} | {g['variant']} | {cell} |")
    folder = Path(out)/pc
    folder.mkdir(parents=True, exist_ok=True)
    s8.write_atomic(folder/'summary.md', '\n'.join(lines) + '\n')
    print('\n'.join(lines))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('plan')
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p = sub.add_parser('lanes')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pc', required=True)
    p.add_argument('--n', type=int, required=True)
    p = sub.add_parser('run-lane')
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pc', required=True)
    p.add_argument('--index', type=int, required=True)
    p.add_argument('--only', help='run this one trial id (tests only)')
    p = sub.add_parser('summary')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--pc', required=True)
    a = parser.parse_args(argv)
    if a.cmd == 'plan':
        cmd_plan(a.root, a.out)
    elif a.cmd == 'lanes':
        cmd_lanes(a.out, a.pc, a.n)
    elif a.cmd == 'run-lane':
        cmd_run_lane(a.root, a.out, a.pc, a.index, a.only)
    elif a.cmd == 'summary':
        cmd_summary(a.out, a.pc)
    return 0


if __name__ == '__main__':
    sys.exit(main())
