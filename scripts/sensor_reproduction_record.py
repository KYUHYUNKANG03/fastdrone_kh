"""Record a sensor DEVELOPMENT reproduction reference (schema sensor_reproduction/1).

The execution path is the one scripts/sensor_reproduce.py replays: the config's
inline sensor profile written to sensors.json, then
control.sensor_campaign.run_campaign for one case, controller and seed. The
reference stores the canonical runtime source hashes, the config hash and the
single trial record. An existing reference is never overwritten; new code gets
a new file name and the old references stay as they are.

python scripts/sensor_reproduction_record.py --config configs/arena_tune7.json \
    --case gust_lateral_p10_VH --controller V13 --seed 3 \
    --run-dir results/reproduction_tune7/projected --output scripts/data/sensor_reproduction_tune7.json
"""
import argparse
import json
import os
from importlib.metadata import version
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control.arena import config_sha256, load_config  # noqa: E402
from control.sensor_binding import runtime_source_hashes  # noqa: E402
from control.sensor_campaign import run_campaign  # noqa: E402
from scripts.sensor_reproduce import canonical_source_hashes  # noqa: E402

THREAD_LIMITS = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')


def environment():
    if any(os.environ.get(name) != '1' for name in THREAD_LIMITS):
        raise SystemExit('set all four numeric thread limits to 1 before recording a reference')
    chip = ''
    if platform.system() == 'Darwin':
        chip = subprocess.run(['sysctl', '-n', 'machdep.cpu.brand_string'], capture_output=True,
                              text=True).stdout.strip()
    return (f"Python {platform.python_version()}; {platform.system()} {platform.release()} "
            f"{platform.machine()} {chip}".rstrip()
            + f"; NumPy {version('numpy')}; SciPy {version('scipy')}; CasADi {version('casadi')}; "
            "all four numeric thread limits 1")


def record(config, case, controller, seed, run_dir, output):
    config, run_dir, output = ROOT/config, ROOT/run_dir, ROOT/output
    if output.exists():
        raise SystemExit(f'{output} exists; references are never overwritten')
    if run_dir.exists() and any(run_dir.iterdir()):
        raise SystemExit('run directory must be new or empty')
    env = environment()
    hashes = canonical_source_hashes(runtime_source_hashes())
    loaded = load_config(config)
    run_dir.mkdir(parents=True, exist_ok=True)
    profile = run_dir/'sensors.json'
    profile.write_text(json.dumps(loaded['sensor_feedback']['profile']), encoding='utf-8')
    trial = run_campaign(config, profile, run_dir/'run', cases=[case], controllers=[controller],
                         seed=seed, timeout_s=900.)
    if len(trial['records']) != 1:
        raise SystemExit(f"expected one trial, got {len(trial['records'])}")
    actual = dict(trial['records'][0])
    actual['run_dir'] = Path(actual['run_dir']).resolve().relative_to(ROOT).as_posix()
    if canonical_source_hashes(runtime_source_hashes()) != hashes:
        raise SystemExit('runtime source changed during the recording run')
    reference = dict(schema='sensor_reproduction/1', stage='DEVELOPMENT',
                     config=config.relative_to(ROOT).as_posix(), config_sha256=config_sha256(loaded),
                     source_screen=(run_dir/'run'/'campaign.json').relative_to(ROOT).as_posix(),
                     environment=env, runtime_source_sha256=hashes, record=actual)
    output.write_text(json.dumps(reference, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps({k: actual.get(k) for k in ('controller', 'scenario_id', 'sensor_seed', 'passed',
                                                  'model_domain_valid', 'trajectory_sha256', 'wall_seconds')}))
    return reference


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--case', required=True)
    p.add_argument('--controller', required=True)
    p.add_argument('--seed', type=int, required=True)
    p.add_argument('--run-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(argv)
    record(a.config, a.case, a.controller, a.seed, a.run_dir, a.output)


if __name__ == '__main__':
    main()
