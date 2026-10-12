"""tune8 본 실험 설정(spec)을 PC별 제어기 묶음으로 만든다 — main7_make_spec.py를 tune8에 맞춘 것.

main7과 같은 겹치기 실행이다: 학교 PC마다 자기가 튜닝한 제어기만 맡고, PC끼리 기록을 옮기지 않는다.
설정끼리는 **제어기 목록·기록 해시·사다리 유무만** 다르고 나머지(시나리오, 시드, 기준 설정)는 같다 —
시행 하나는 (시나리오, 제어기, 시드) 단위로 독립이라 나중에 합쳐 쓴다.

main7과 다른 점:
  - 기준 설정 configs/arena_tune8.json, 튜닝 기록 tune8(예산 60).
  - 본 실험 시드 1020~1039(사다리 1020~1029). tune8 개정은 tune7 본 실험(시드 1000~1019) 결과를 본 뒤에
    나왔으므로 그 시드를 다시 쓰지 않는다.
  - 본 실험 전 관문(school/tune8_gate.py)을 통과한 기록만 받는다. 관문 결과가 없거나, 실패했거나,
    지금 기록과 다른 기록으로 돌린 것이면 거부한다.

사다리 묶음은 V13 기록으로 돈다. V13이 없는 설정에서는 사다리 변형을 비운다.
설정 파일은 configs/ 밖(results/main8/)에 쓴다. control/·configs/는 건드리지 않는다.

    python main8_make_spec.py --root <fds8> --controllers V13 CPID --out <fds8>/results/main8/part.spec.json
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

TEMPLATE = 'configs/main_experiment_sensor_candidate_v6.json'
BASE = 'configs/arena_tune8.json'
BASE_SHA = 'f0bb414658361f68e29b838425c0485074c6c32b339c83921162485dc8118767'
TUNE_DIR = 'results/arena/tuning/tune8'          # 튜닝·관문 결과가 쓰인 곳
RUN_DIR = 'results/arena/tuning/tune8_final'     # 본 실험이 읽는 기록 사본
BUDGET = 60
CONTROLLERS = ('V13', 'M17', 'F13', 'GSLQR', 'CPID')
SENSOR_SEEDS = list(range(1020, 1040))           # 비교 묶음 20개
LADDER_SEEDS = list(range(1020, 1030))           # 사다리 10개, 비교 시드의 부분집합


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_gate(root, label, record_sha):
    """관문 결과가 지금 기록으로 돈 PASS인지. 문제 문자열 또는 None."""
    path = root/TUNE_DIR/f'{label}.gate.json'
    if not path.exists():
        return f'{label}: no gate result ({path}) - run school/tune8_gate.py first'
    gate = json.loads(path.read_text(encoding='utf-8'))
    if gate.get('schema') != 'tune8-gate/1':
        return f'{label}: {path} is not a tune8 gate result'
    gains = gate.get('gains') or {}
    if gains.get('source') != 'tuned' or gains.get('record_sha256') != record_sha:
        return f'{label}: gate was run on a different record (or on prior gains) - rerun the gate'
    if gate.get('config_sha256') != BASE_SHA:
        return f'{label}: gate was run on a different config'
    if not gate.get('gate_pass'):
        failed = [r['id'] for r in gate.get('rows', []) if r.get('required') and not r.get('skipped')
                  and not r.get('tracking_pass')]
        return f'{label}: gate FAIL ({", ".join(failed)}) - not starting the main experiment for it'
    return None


def make_spec(labels, root, budget=BUDGET, require_gate=True):
    root = Path(root)
    labels = [l for l in CONTROLLERS if l in labels]
    spec = deepcopy(json.loads((root/TEMPLATE).read_text(encoding='utf-8')))
    spec['purpose'] = (f'tune8 main experiment part ({", ".join(labels)}). Parts differ only in controllers, '
                       'record hashes and the ladder (only the part with V13).')
    spec['base_config'] = dict(path=BASE, config_sha256=BASE_SHA)
    spec['controllers'] = labels
    records = {}
    for label in labels:
        path = root/RUN_DIR/f'{label}.record.json'
        if not path.exists():
            raise SystemExit(f'{path}: missing (copy the final record first)')
        record = json.loads(path.read_text(encoding='utf-8'))
        if record.get('status') != 'complete' or record.get('spent') != budget or record.get('budget') != budget:
            raise SystemExit(f"{label}: record is not complete at {budget} "
                             f"(status {record.get('status')}, {record.get('spent')}/{record.get('budget')})")
        if record.get('config_sha256') != BASE_SHA:
            raise SystemExit(f'{label}: record was tuned on another config ({record.get("config_sha256")})')
        sha = sha256_file(path)
        source = root/TUNE_DIR/f'{label}.record.json'
        if source.exists() and sha256_file(source) != sha:
            raise SystemExit(f'{label}: {path} differs from the tuning output {source}')
        if require_gate:
            problem = check_gate(root, label, sha)
            if problem:
                raise SystemExit(problem)
        records[label] = dict(sha256=sha, tag='tune8', budget=budget)
    spec['tuned'] = dict(run_dir=RUN_DIR, records=records,
                         note=f'Final tune8 records (budget {budget}, configs/arena_tune8.json). '
                              'Each passed the pre-main gate (school/tune8_gate.py) before this spec was written.')
    spec['sensor_seeds'] = SENSOR_SEEDS
    spec['ladder_sensor_seeds'] = LADDER_SEEDS
    spec['sensor_seed_note'] = ('tune8 (2026-10-12): comparison 1020-1039, ladder 1020-1029. Seeds 1000-1019 were '
                                'used by the tune7 main experiment, whose results led to the tune8 revision, so they '
                                'are not reused. Fixed before any tune8 main result.')
    if 'V13' not in labels:
        spec['ladder'] = dict(spec['ladder'], variants={},
                              note='Ladder runs only in the part with V13 (it uses the V13 record).')
    spec['part'] = dict(controllers=labels)
    return spec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--controllers', nargs='+', choices=CONTROLLERS, required=True)
    parser.add_argument('--root', type=Path, required=True, help='fds8 checkout')
    parser.add_argument('--budget', type=int, default=BUDGET, help='60 for the real run; other values only for dry runs')
    parser.add_argument('--no-gate', action='store_true', help='dry runs only - skips the gate check')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    spec = make_spec(args.controllers, args.root.resolve(), args.budget, require_gate=not args.no_gate)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(spec, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    print(f'wrote {args.out} ({", ".join(spec["controllers"])}) sha256 {sha256_file(args.out)}')


if __name__ == '__main__':
    main()
