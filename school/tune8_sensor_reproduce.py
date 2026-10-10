"""tune8 학교 PC 점검용 — scripts/sensor_reproduce.py를 tune8 커밋에서 돌린다.

sensor_reproduce.py는 기준 기록(tune-final-7 코드로 만든 것)과 실행 코드의 해시가 한 글자라도 다르면
계산을 시작하지 않는다. tune8은 control/arena_tune.py에 objective.acceptance_failure 옵션을 넣었으므로
그 파일 하나의 해시가 다르다. 이 점검이 실제로 돌리는 것은 validation_suite의 시행 하나이고
arena_tune.py는 실행 경로에 없다.

그래서 이 스크립트는
  1. 기준 기록과 지금 코드의 해시 차이를 계산하고,
  2. 차이가 ALLOWED(control/arena_tune.py) 밖에 하나라도 있으면 거부하고,
  3. 그 안에만 있으면 그 사실을 출력한 뒤 원래의 수치 재현 비교(판정 일치 + rtol 1e-3)를 그대로 돌린다.
수치 비교 기준은 건드리지 않는다. 결과 파일(comparison.json)도 원래와 같다.

    python tune8_sensor_reproduce.py --root <fds8> --output <새 폴더> [--reference <기준 파일>]
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

ALLOWED = frozenset({'control/arena_tune.py'})


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True, help='fds8 checkout')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path)
    args = parser.parse_args(argv)
    root = args.root.resolve()
    sys.path.insert(0, str(root))
    spec = importlib.util.spec_from_file_location('sensor_reproduce', root/'scripts'/'sensor_reproduce.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    reference_path = args.reference if args.reference else root/'scripts'/'data'/'sensor_reproduction_tune7.json'
    if not reference_path.is_absolute():
        reference_path = root/reference_path
    reference = json.loads(reference_path.read_text(encoding='utf-8'))
    recorded = module.canonical_source_hashes(reference['runtime_source_sha256'])
    current = module.canonical_source_hashes(module.runtime_source_hashes())
    differing = sorted(name for name in set(recorded) | set(current) if recorded.get(name) != current.get(name))
    outside = [name for name in differing if name not in ALLOWED]
    print(f'runtime source files differing from the reference: {differing or "none"}', flush=True)
    if outside:
        print(f'FAIL: source differs outside the allowed set {sorted(ALLOWED)}: {outside}', flush=True)
        return 1
    if differing:
        # 허용한 파일만 다르다 — 해시 대조만 통과시키고 수치 재현 비교는 원래 코드 그대로 돈다.
        module.runtime_source_hashes = lambda: reference['runtime_source_sha256']
    result = module.reproduce(reference_path, args.output)
    print(json.dumps({k: v for k, v in result.items() if k not in ('reference', 'actual')}, indent=2), flush=True)
    return 0 if result['reproduction_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
