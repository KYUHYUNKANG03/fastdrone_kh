"""ablation8 — V13 구성요소 제거 실험을 센서 조건(IMU 지연 0·5 ms)에서 다시 돌린다.

sensor8에서 IMU 지연 5 ms의 V_H 돌풍은 V13 20/20, F13 0/20이었다. 그 차이가 NMPC 인터페이스(가상입력,
ω̇=ν)에서 오는지 INDI 구성요소(A1 총추력 보존 배분, C1 배분 결과 피드백, S1 시간 정렬)에서 오는지 가른다.

sensor8과 같은 것: 기준 설정 configs/arena_tune8.json, V13 tune8 기록(sha256 고정), 시나리오 정의와 시드
(1020~1039), 센서 프로파일(nominal, imu_latency_5ms — sensor8의 CONDITIONS 그대로), 시행 함수와 판정.
다른 것: 변형마다 controllers.V13 항목만 바꾼 파생 설정으로 팩토리를 만든다(main8 ladder와 같은 방식,
control.main_experiment._derived). V13-0/1/2도 V13의 튜닝값을 그대로 쓴다 — 변형별로 다시 튜닝하지 않았다.
V13(원본)도 같이 돌린다: 같은 플랫폼에서 네 변형을 비교하고, 다른 플랫폼(PC-63) 결과와의 차이도 잰다.

시행은 우선순위 묶음(order) 순으로 돈다. 레인마다 묶음 하나를 고르게 나눠 받으므로 중간에 끊어도 끝난 묶음은
네 변형 비교가 완결돼 있다. 레인 나누기·실행·재개 규칙은 sensor8_run과 같다.

    python ablation8_run.py plan     --root <fds> --out <fds>/results/ablation8
    python ablation8_run.py lanes    --out <fds>/results/ablation8 --pc CLOUD --n 2
    python ablation8_run.py run-lane --root <fds> --out <fds>/results/ablation8 --pc CLOUD --index 0
    python ablation8_run.py status   --out <fds>/results/ablation8 --pc CLOUD
    python ablation8_run.py summary  --out <fds>/results/ablation8 --pc CLOUD
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

PLAN_SCHEMA = 'ablation8-plan/1'
LANES_SCHEMA = 'ablation8-lanes/1'
LABEL = 'V13'
VARIANTS = {
    'V13-0': dict(alloc_mode='A0', alloc_feedback=False, time_align='S0'),
    'V13-1': dict(alloc_mode='A1', alloc_feedback=False, time_align='S0'),
    'V13-2': dict(alloc_mode='A1', alloc_feedback=True, time_align='S0'),
    'V13': dict(alloc_mode='A1', alloc_feedback=True, time_align='S1'),
}
GUST, ACCEL, BRAKE = 'main_gust_lateral_+10_V_H', 'main_ref_accel_VH_rho1', 'main_ref_brake_VH_rho1'
# 우선순위: IMU 5 ms의 돌풍·가속(핵심 질문) → 명목 돌풍(변형별 기준선) → IMU 5 ms 감속 → 명목 가속·감속
GROUPS = [('imu_latency_5ms', GUST), ('imu_latency_5ms', ACCEL), ('nominal', GUST),
          ('imu_latency_5ms', BRAKE), ('nominal', ACCEL), ('nominal', BRAKE)]


def runner_sha256():
    return s8.sha256_bytes(Path(__file__).read_bytes() + Path(s8.__file__).read_bytes())


def variant_config(me, base, scen, variant):
    return me._derived(base, scen, VARIANTS[variant])


def build_plan(root):
    root = s8.use_root(root)
    from control.arena import load_config, config_sha256, excluded_from, build_scenarios
    from control.arena_factory import ArenaFactory
    from control.arena_sensors import load_sensor_profile
    from control.sensor_binding import resolve_feedback
    from control.validation_suite import baseline_params, source_hashes
    from control import main_experiment as me
    problems = s8.check_records(root)
    if problems:
        raise SystemExit('records:\n  - ' + '\n  - '.join(problems))
    base = load_config(root/s8.BASE)
    if config_sha256(base) != s8.BASE_SHA:
        raise SystemExit(f'{s8.BASE}: hash {config_sha256(base)[:12]} is not the tune8 config')
    native = baseline_params()
    _, pool = s8.scenario_dicts(root, base, native)
    scen = [s for s in pool if s['id'] in (GUST, ACCEL, BRAKE)]
    configs = {v: variant_config(me, base, scen, v) for v in VARIANTS}
    cp = ArenaFactory(base, native).cp
    built = {s.id: s for s in build_scenarios(configs['V13'], cp, native)}
    nominal = resolve_feedback(base)
    s8cond = {c['name']: c for c in s8.CONDITIONS}
    conditions, trials = [], []
    for name in sorted({c for c, _ in GROUPS}, key=[c for c, _ in GROUPS].index):
        cond = s8cond[name]
        profile = s8.apply_changes(nominal.profile, cond['changes'], root)
        if cond['changes']:
            profile['name'] = f"{nominal.profile['name']}+{name}"
        profile = load_sensor_profile(profile)
        conditions.append(dict(name=name, note=cond['note'], changes=cond['changes'], profile=profile,
                               profile_sha256=resolve_feedback(base, mode='sensors', profile=profile)
                               .metadata['sensor_profile_sha256']))
    for order, (cname, sid) in enumerate(GROUPS):
        for variant in VARIANTS:
            reason = excluded_from(configs[variant], LABEL, built[sid])
            if reason:
                raise SystemExit(f'{variant} excluded from {sid}: {reason}')
            for seed in s8.SEEDS:
                trials.append(dict(trial_id=f'{cname}/{sid}/{variant}/seed_{seed}', condition=cname, order=order,
                                   scenario_id=sid, variant=variant, sensor_seed=seed,
                                   est_seconds=s8.EST[sid][LABEL]))
    return dict(schema=PLAN_SCHEMA,
                purpose='ablation8: V13 component ladder (V13-0/1/2/V13, tune8 V13 gains, not re-tuned per variant) '
                        'under nominal sensors and IMU latency 5 ms; paired seeds 1020-1039 as in sensor8.',
                base_config=dict(path=s8.BASE, config_sha256=s8.BASE_SHA),
                template=dict(path=s8.TEMPLATE, sha256=s8.sha256_file(root/s8.TEMPLATE)),
                tuned=dict(run_dir=s8.RUN_DIR, record_sha256=s8.RECORDS[LABEL]),
                variants=VARIANTS,
                derived_config_sha256={v: config_sha256(c) for v, c in configs.items()},
                groups=[dict(order=i, condition=c, scenario_id=s) for i, (c, s) in enumerate(GROUPS)],
                seeds=s8.SEEDS, trace_seed=s8.TRACE_SEED,
                source_sha256=source_hashes(), runner_sha256=runner_sha256(),
                conditions=conditions, trials=trials)


def plan_path(out):
    return Path(out)/'ablation8.plan.json'


def load_plan(out):
    path = plan_path(out)
    plan = json.loads(path.read_text(encoding='utf-8'))
    if plan.get('schema') != PLAN_SCHEMA:
        raise SystemExit(f'{path}: not an ablation8 plan')
    return plan, s8.sha256_file(path)


def cmd_plan(root, out):
    plan = build_plan(root)
    text = json.dumps(plan, ensure_ascii=False, indent=1) + '\n'
    path = plan_path(out)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.read_text(encoding='utf-8') != text:
            raise SystemExit(f'{path} exists and differs from the plan this script makes now - use a new output folder')
        print(f'plan unchanged: {path}')
    else:
        s8.write_atomic(path, text)
        print(f'wrote {path}')
    for g in plan['groups']:
        ts = [t for t in plan['trials'] if t['order'] == g['order']]
        print(f"group {g['order']} {g['condition']} {g['scenario_id']}: {len(ts)} trials, "
              f"{sum(t['est_seconds'] for t in ts)/3600:.1f} worker-hours (Windows estimate)")
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
    path = lanes_path(out, pc)
    if path.exists():
        old = json.loads(path.read_text(encoding='utf-8'))
        if old.get('plan_sha256') != plan_sha or old.get('n') != n or old.get('lanes') != doc['lanes']:
            raise SystemExit(f'{path} exists for another plan or lane count')
        print(f'lanes unchanged: {path}')
    else:
        s8.write_atomic(path, json.dumps(doc, ensure_ascii=False, indent=1) + '\n')
        print(f'wrote {path}')
    for lane in doc['lanes']:
        print(f"lane {lane['index']}: {len(lane['trials'])} trials, ~{lane['est_hours']:.1f} h (Windows estimate)")


def load_lanes(out, pc):
    doc = json.loads(lanes_path(out, pc).read_text(encoding='utf-8'))
    if doc.get('schema') != LANES_SCHEMA:
        raise SystemExit('not an ablation8 lanes file')
    return doc


class Runner:
    def __init__(self, root, plan, plan_sha):
        self.root = s8.use_root(root)
        from control.arena import load_config, config_sha256, build_scenarios
        from control.arena_factory import ArenaFactory
        from control.arena_design_check import tuned_overrides
        from control.validation_suite import baseline_params, source_hashes
        from control import main_experiment as me
        self.plan, self.plan_sha = plan, plan_sha
        problems = s8.check_records(self.root)
        base = load_config(self.root/s8.BASE)
        if config_sha256(base) != plan['base_config']['config_sha256']:
            problems.append('base config hash differs from the plan')
        if source_hashes() != plan['source_sha256']:
            problems.append('control/configs source hashes differ from the plan')
        if runner_sha256() != plan['runner_sha256']:
            problems.append('ablation8_run.py or sensor8_run.py differs from the one that made the plan')
        if problems:
            raise SystemExit('refusing to run:\n  - ' + '\n  - '.join(problems))
        self.base, self.native = base, baseline_params()
        self.spec, pool = s8.scenario_dicts(self.root, base, self.native)
        scen = [s for s in pool if s['id'] in (GUST, ACCEL, BRAKE)]
        self.overrides, self.used = tuned_overrides(base, self.root/s8.RUN_DIR, [LABEL])
        self.configs, self.factories = {}, {}
        for v in VARIANTS:
            config = variant_config(me, base, scen, v)
            if config_sha256(config) != plan['derived_config_sha256'][v]:
                raise SystemExit(f'derived config for {v} differs from the plan')
            self.configs[v] = config
            self.factories[v] = ArenaFactory(config, self.native, overrides=self.overrides)
        cp = ArenaFactory(base, self.native).cp
        self.scenarios = {s.id: s for s in build_scenarios(self.configs['V13'], cp, self.native)}
        self.conditions = {c['name']: c for c in plan['conditions']}
        self.trials = {t['trial_id']: t for t in plan['trials']}
        self.source = plan['source_sha256']
        self._trim = {}

    def hashes(self, variant):
        return dict(plan_sha256=self.plan_sha, base_config_sha256=self.plan['base_config']['config_sha256'],
                    derived_config_sha256=self.plan['derived_config_sha256'][variant],
                    tuned_record_sha256=s8.RECORDS[LABEL], runner_sha256=self.plan['runner_sha256'])

    def run_trial(self, trial):
        import numpy as np
        from control.validation_metrics import Acceptance, PaperCriteria, paper_evaluate
        from control.validation_suite import run_trial as suite_run
        from control import main_experiment as me
        cond = self.conditions[trial['condition']]
        s = self.scenarios[trial['scenario_id']]
        variant = trial['variant']
        config = self.configs[variant]
        if s.id not in self._trim:
            self._trim[s.id] = me.trim_status(self.native, s)
        status, detail = self._trim[s.id]
        row = dict(trial_id=trial['trial_id'], condition=cond['name'], scenario_id=s.id, scenario_type=s.type,
                   controller=LABEL, variant=variant, case=s.cases[0], window=list(s.window),
                   sensor_seed=trial['sensor_seed'], **self.hashes(variant),
                   trim_detail={f'{v:g}': d for v, d in detail.items()},
                   authority_boundary=me.authority_boundary(detail, self.spec, self.base))
        if status:
            row.update(skipped=True, skip_reason='vehicle_limit')
            return row, None
        limits = Acceptance(**config['acceptance'])
        paper = PaperCriteria(**config['paper_criteria'])
        try:
            metrics, result, log = suite_run(self.factories[variant], LABEL, s.profile, s.cases[0], limits,
                                             feedback='sensors', sensor_profile=cond['profile'],
                                             sensor_seed=trial['sensor_seed'])
            metrics.update(paper_evaluate(result, s.profile, paper, window=s.window, solve_log=log,
                                          n_max=self.native['n_max']))
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            metrics = dict(passed=False, failure_reasons=['setup_error'], stop_reason=f'{type(exc).__name__}: {exc}')
            result = None
        row.update(skipped=False, skip_reason=None, **metrics)
        row['estimation_error'] = s8.estimation_error(result)
        if result is not None and row.get('sensor_profile_sha256') != cond['profile_sha256']:
            raise SystemExit(f"{trial['trial_id']}: ran with sensor profile {row.get('sensor_profile_sha256')}, "
                             f"plan says {cond['profile_sha256']}")
        return row, result


def lane_dir(out, pc, index):
    return Path(out)/pc/f'lane{index}'/'trials'


def cmd_run_lane(root, out, pc, index, max_trials=None, only=None):
    import numpy as np
    plan, plan_sha = load_plan(out)
    if only is None:
        lanes = load_lanes(out, pc)
        if lanes['plan_sha256'] != plan_sha:
            raise SystemExit('lanes file was made for another plan')
    runner = Runner(root, plan, plan_sha)
    from control.main_distributed import machine_fingerprint
    from control.validation_suite import json_safe, git_state
    folder = lane_dir(out, pc, index)
    folder.mkdir(parents=True, exist_ok=True)
    traces = Path(out)/pc/'traces'
    traces.mkdir(parents=True, exist_ok=True)
    for stale in glob.glob(str(folder/'*.tmp')) + glob.glob(str(traces/f'*.lane{index}.tmp')):
        os.remove(stale)
        print(f'removed interrupted file {Path(stale).name}', flush=True)
    revision, dirty = git_state()
    machine = machine_fingerprint()
    ids = lanes['lanes'][index]['trials'] if only is None else [only]
    ran = 0
    for trial_id in ids:
        trial = runner.trials[trial_id]
        path = folder/(s8.safe_name(trial_id)+'.json')
        if path.exists():
            done = json.loads(path.read_text(encoding='utf-8'))
            want = runner.hashes(trial['variant'])
            if {k: done.get(k) for k in want} != want or done.get('source_sha256') != runner.source:
                raise SystemExit(f'{path}: finished with different hashes - refusing to mix results')
            continue
        if max_trials is not None and ran >= max_trials:
            break
        started = s8._utc()
        try:
            row, result = runner.run_trial(trial)
        except Exception as exc:
            import traceback
            row = dict(trial_id=trial_id, condition=trial['condition'], scenario_id=trial['scenario_id'],
                       controller=LABEL, variant=trial['variant'], sensor_seed=trial['sensor_seed'],
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
        ran += 1
        verdict = 'SKIP ' + row['skip_reason'] if row['skipped'] else row.get('tracking_pass')
        print(f'[{index}] {trial_id}: {verdict}', flush=True)


def done_rows(out, pc):
    rows = {}
    for path in glob.glob(str(Path(out)/pc/'lane*'/'trials'/'*.json')):
        row = json.loads(Path(path).read_text(encoding='utf-8'))
        rows[row['trial_id']] = row
    return rows


def cmd_status(out, pc):
    plan, _ = load_plan(out)
    rows = done_rows(out, pc)
    for g in plan['groups']:
        ts = [t for t in plan['trials'] if t['order'] == g['order']]
        print(f"group {g['order']} {g['condition']} {g['scenario_id']}: "
              f"{sum(t['trial_id'] in rows for t in ts)}/{len(ts)}")
    print(f"all: {len(rows)}/{len(plan['trials'])}")


def cmd_summary(out, pc):
    plan, plan_sha = load_plan(out)
    rows = done_rows(out, pc)
    lines = [f'# ablation8 summary ({pc})', '', f'plan sha256 {plan_sha[:12]}, {len(rows)}/{len(plan["trials"])} done',
             '', 'Cell: tracking pass / done (of 20).', '',
             '| condition | scenario | ' + ' | '.join(VARIANTS) + ' |', '|---|---|' + '---:|'*len(VARIANTS)]
    for g in plan['groups']:
        cells = []
        for v in VARIANTS:
            ts = [t for t in plan['trials'] if t['order'] == g['order'] and t['variant'] == v]
            got = [rows[t['trial_id']] for t in ts if t['trial_id'] in rows]
            cells.append(f"{sum(bool(r.get('tracking_pass')) for r in got)}/{len(got)}" if got else '-')
        lines.append(f"| {g['condition']} | {g['scenario_id'].replace('main_', '')} | " + ' | '.join(cells) + ' |')
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
    p.add_argument('--max-trials', type=int, help='tests only')
    p.add_argument('--only', help='run this one trial id (tests only)')
    for name in ('status', 'summary'):
        p = sub.add_parser(name)
        p.add_argument('--out', type=Path, required=True)
        p.add_argument('--pc', required=True)
    a = parser.parse_args(argv)
    if a.cmd == 'plan':
        cmd_plan(a.root, a.out)
    elif a.cmd == 'lanes':
        cmd_lanes(a.out, a.pc, a.n)
    elif a.cmd == 'run-lane':
        cmd_run_lane(a.root, a.out, a.pc, a.index, a.max_trials, a.only)
    elif a.cmd == 'status':
        cmd_status(a.out, a.pc)
    elif a.cmd == 'summary':
        cmd_summary(a.out, a.pc)
    return 0


if __name__ == '__main__':
    sys.exit(main())
