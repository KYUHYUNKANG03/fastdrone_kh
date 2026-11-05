"""통합 임무(main_mission_V_H) 궤적을 그림용으로 다시 돌린다.

main8과 같은 것: 기준 설정 configs/arena_tune8.json, tune8 최종 기록, main8 템플릿에서 전개한 시나리오,
기준 센서 프로파일(resolve_feedback(base)), 시드, 시행 함수(validation_suite.run_trial)와 판정.
sensor8_run의 Runner와 같은 방식(me._derived)으로 팩토리를 만든다 — sensor8 nominal은 Windows에서 main8과 비트 동일했다.

    python school/mission_trace.py --root . --out <dir> V13:1020 V13:1024 F13:1020

결과: <dir>/<CONTROLLER>_<SEED>.json(판정·지표)과 .npz(궤적 — 커서 git에는 넣지 않음).
results_summary/build_summary.py --mission-rerun <dir>가 json을 읽어 data/mission_rerun.csv를 만든다.
OS가 다르면(Windows 대 Linux) 로터 포화 구간에서 궤적이 갈릴 수 있다 — results_summary README 4.1절.
"""
import argparse
import json
import sys
import time
from copy import deepcopy
from pathlib import Path

SID = 'main_mission_V_H'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('jobs', nargs='+', help='CONTROLLER:SEED')
    a = ap.parse_args()
    root = Path(a.root).resolve()
    sys.path.insert(0, str(root))
    sys.path.insert(0, str(root/'school'))
    import numpy as np
    import sensor8_run as s8
    from control.arena import load_config, config_sha256, build_scenarios
    from control.arena_factory import ArenaFactory
    from control.arena_design_check import tuned_overrides
    from control.arena_sensors import load_sensor_profile
    from control.sensor_binding import resolve_feedback
    from control.validation_metrics import Acceptance
    from control.validation_suite import baseline_params, run_trial as suite_run, json_safe
    from control import main_experiment as me

    problems = s8.check_records(root)
    if problems:
        raise SystemExit('\n'.join(problems))
    base = load_config(root/s8.BASE)
    assert config_sha256(base) == s8.BASE_SHA, 'base config hash'
    native = baseline_params()
    spec = json.loads((root/s8.TEMPLATE).read_text(encoding='utf-8'))
    spec['controllers'] = list(s8.CONTROLLERS)
    spec['ladder'] = dict(spec['ladder'], variants={})
    pool = {}
    for batch in me.build_batches(spec, base, native):
        for s in batch.config['scenarios']:
            pool[s['id']] = s
    config = me._derived(base, [deepcopy(pool[SID])])
    cp = ArenaFactory(base, native).cp
    scen = {s.id: s for s in build_scenarios(config, cp, native)}[SID]
    overrides, _ = tuned_overrides(base, root/s8.RUN_DIR, list(s8.CONTROLLERS))
    factory = ArenaFactory(config, native, overrides=overrides)
    nominal = resolve_feedback(base)
    profile = load_sensor_profile(deepcopy(nominal.profile))
    limits = Acceptance(**config['acceptance'])
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for job in a.jobs:
        label, seed = job.split(':')
        seed = int(seed)
        t0 = time.time()
        metrics, result, log = suite_run(factory, label, scen.profile, scen.cases[0], limits,
                                         feedback='sensors', sensor_profile=profile, sensor_seed=seed)
        wall = time.time() - t0
        keep = {k: v for k, v in result.items() if k not in ('error', 'navigation_trace')
                and isinstance(v, np.ndarray) and v.dtype != object}
        np.savez_compressed(out/f'{label}_{seed}.npz', n_max=native['n_max'],
                            phases=np.array([[st, en] for _, st, en in scen.profile.get_phase_boundaries()]),
                            **keep)
        metrics = dict(metrics, wall_seconds=wall, error=str(result.get('error')),
                       phase_names=[n for n, _, _ in scen.profile.get_phase_boundaries()],
                       window=list(scen.window), saved=sorted(keep))
        (out/f'{label}_{seed}.json').write_text(json.dumps(json_safe(metrics), ensure_ascii=False, indent=1))
        print(f'{job}: passed={metrics.get("passed")} max_v={metrics.get("max_velocity_error"):.3f} '
              f'sat={metrics.get("motor_saturation_fraction")} T={metrics.get("simulated_seconds")} '
              f'wall={wall:.0f}s reasons={metrics.get("failure_reasons")}', flush=True)


if __name__ == '__main__':
    main()
