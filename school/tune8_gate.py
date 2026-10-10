"""tune8 본 실험 전 관문 — 튜닝이 끝난 제어기가 섭동 없는 순항을 버티는지 본다.

tune7에서는 튜닝된 V13이 V_H(85 m/s) 순항을 못 버틴다는 것을 본 실험(이틀 반)을 다 돌린 뒤에야 알았다.
이 관문은 그 확인을 몇 분으로 당긴다. 튜닝 기록의 최선값으로:

  필수   gate_cruise_VL                  V_L 순항 15 s, 섭동 없음
  필수   gate_cruise_VH                  V_H 순항 15 s, 섭동 없음
  참고   gate_cruise_VH__state_delay_5ms V_H 순항 15 s, 상태 지연 5 ms  (판정에 넣지 않고 기록만)

제어기의 설계 영역 밖 사례는 본 실험과 같은 규칙(arena.excluded_from)으로 뺀다(CPID는 V_H 사례 제외).
판정은 본 실험 합격 기준(Acceptance)의 추종 통과(tracking_pass)다. 추력 모델 영역 판정은 넣지 않는다.

시드는 튜닝 구간(2000~2999) 안에서 튜닝 시나리오가 쓰지 않는 2901~2903이다 — 본시험 시드(1000~1999)를
쓰지 않는다. 이 관문은 게인을 고르지 않는다(통과/실패만 보고한다). 본 실험을 시작할지는 사람이 정한다.

    python school/tune8_gate.py --root <fds8> --controllers V13 CPID
    python school/tune8_gate.py --root <fds8> --controllers V13 --prior      # 초기값으로(점검용)

결과: <run-dir>/<제어기>.gate.json. 전부 통과면 0, 아니면 1로 끝난다.
control/·configs/는 읽기만 한다.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

CONFIG = 'configs/arena_tune8.json'
RUN_DIR = 'results/arena/tuning/tune8'
CONTROLLERS = ('V13', 'M17', 'F13', 'GSLQR', 'CPID')
TIMES = [3.0, 5.0, 7.0]
GATE = (
    # (시나리오, 시드, 필수 여부)
    (dict(id='gate_cruise_VL', type='cruise', speed='V_L', times_s=TIMES, evaluation_start_s=3.0), 2901, True),
    (dict(id='gate_cruise_VH', type='cruise', speed='V_H', times_s=TIMES, evaluation_start_s=3.0), 2902, True),
    (dict(id='gate_cruise_VH__state_delay_5ms', type='cruise', speed='V_H', times_s=TIMES, evaluation_start_s=3.0,
          extra_params=dict(state_delay_s=0.005)), 2903, False),
)


def run_gate(root, label, run_dir=RUN_DIR, prior=False):
    from control.arena import load_config, config_sha256, build_scenarios, validate_config, excluded_from
    from control.arena_factory import ArenaFactory
    from control.validation_metrics import Acceptance
    from control.validation_suite import run_trial
    from models.team_light.control.baseline_v2 import baseline_params
    root = Path(root)
    config = load_config(root/CONFIG)
    lo, hi = config['seeds']['tuning']
    used = set((config['sensor_feedback'].get('tuning_seeds') or {}).values())
    for _, seed, _ in GATE:
        if not lo <= seed <= hi or seed in used:
            raise SystemExit(f'gate seed {seed} must lie in seeds.tuning and be unused by tuning scenarios')
    native = baseline_params()
    overrides, gains = None, dict(source='prior')
    if not prior:
        from control.arena_design_check import tuned_overrides
        overrides, used_records = tuned_overrides(config, root/run_dir, [label])
        gains = dict(source='tuned', **used_records[label])
    derived = deepcopy(config)
    derived['scenarios'] = [deepcopy(s) for s, _, _ in GATE]
    validate_config(derived)
    factory = ArenaFactory(derived, native, overrides=overrides)
    cp = ArenaFactory(config, native).cp
    scenarios = build_scenarios(derived, cp, native)
    limits = Acceptance(**config['acceptance'])
    rows, ok = [], True
    for scenario, (_, seed, required) in zip(scenarios, GATE):
        row = dict(id=scenario.id, sensor_seed=seed, required=required)
        reason = excluded_from(derived, label, scenario)
        if reason:
            row.update(skipped=True, skip_reason=reason)
            rows.append(row)
            print(f'  [{label}] {scenario.id}: skipped ({reason})', flush=True)
            continue
        try:
            metrics, _, _ = run_trial(factory, label, scenario.profile, scenario.cases[0], limits, sensor_seed=seed)
            passed = bool(metrics['tracking_pass'])
            row.update(skipped=False, tracking_pass=passed, stop_reason=metrics['stop_reason'],
                       failure_reasons=[r for r in metrics['failure_reasons'] if r != 'propulsion_model_domain'],
                       rmse_velocity=metrics['rmse_velocity'], rmse_z=metrics['rmse_z'],
                       max_velocity_error=metrics['max_velocity_error'], max_z_error=metrics['max_z_error'],
                       max_omega=metrics['max_omega'],
                       motor_saturation_fraction=metrics.get('motor_saturation_fraction'),
                       simulated_seconds=metrics['simulated_seconds'],
                       trajectory_sha256=metrics['trajectory_sha256'])
        except Exception as exc:            # 관문은 끝까지 돌아 전부 보고한다
            passed = False
            row.update(skipped=False, tracking_pass=False, stop_reason=f'{type(exc).__name__}: {exc}')
        rows.append(row)
        if required and not passed:
            ok = False
        tag = 'PASS' if passed else ('FAIL' if required else 'fail (informational)')
        print(f'  [{label}] {scenario.id}: {tag}  max_omega={row.get("max_omega")}  stop={row.get("stop_reason")}', flush=True)
    return dict(schema='tune8-gate/1', controller=label, gate_pass=ok, gains=gains,
                config=CONFIG, config_sha256=config_sha256(config), rows=rows,
                note='Required rows: nominal cruise at V_L and V_H (tracking_pass). Informational rows are recorded only. '
                     'Seeds 2901-2903 are tuning-range seeds unused by tuning scenarios; no main-test seed is used. '
                     'The gate reports pass/fail and selects nothing.',
                created_utc=datetime.now(timezone.utc).isoformat())


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1], help='fds8 checkout')
    parser.add_argument('--controllers', nargs='+', choices=CONTROLLERS, required=True)
    parser.add_argument('--run-dir', default=RUN_DIR)
    parser.add_argument('--prior', action='store_true', help='초기값으로 돌린다(점검용). 결과 파일 이름에 .prior가 붙는다')
    parser.add_argument('--out-dir', type=Path, help='결과를 쓸 폴더(기본: run-dir)')
    args = parser.parse_args(argv)
    root = args.root.resolve()
    sys.path.insert(0, str(root))
    out_dir = args.out_dir or (root/args.run_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    all_ok = True
    for label in args.controllers:
        print(f'== tune8 gate {label} ({"prior" if args.prior else "tuned"}) ==', flush=True)
        report = run_gate(root, label, args.run_dir, args.prior)
        path = out_dir/f'{label}{".prior" if args.prior else ""}.gate.json'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
        sha = hashlib.sha256(path.read_bytes()).hexdigest()
        print(f'GATE {label}: {"PASS" if report["gate_pass"] else "FAIL"}  ({path}, sha256 {sha})', flush=True)
        all_ok = all_ok and report['gate_pass']
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
