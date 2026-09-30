"""Audit local DEVELOPMENT prerequisites without launching a simulation.

The paper gate remains closed until the documented research and distributed
execution checks are completed. Static checks do not measure flight quality.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.setup_env import check_config, check_threads, check_versions, read_lock, THREAD_VARIABLES

CONFIG = 'configs/arena_rotor_projected_development_v9.json'
SPEC = 'configs/main_experiment_sensor_candidate_v6.json'


def git_binary():
    # This Mac's /usr/bin/git launches an Xcode license prompt. Use the already
    # installed CLI tool without accepting licenses or changing system settings.
    cli = Path('/Library/Developer/CommandLineTools/usr/bin/git')
    if platform.system() == 'Darwin' and cli.is_file():
        return str(cli)
    return shutil.which('git')


def configure_process():
    """Only this process and its children; no shell/profile/system changes."""
    os.environ.update({key: '1' for key in THREAD_VARIABLES})
    binary = git_binary()
    if binary:
        os.environ['PATH'] = str(Path(binary).parent)+os.pathsep+os.environ.get('PATH', '')


def git_state():
    binary = git_binary()
    if not binary:
        return dict(available=False, reason='git unavailable (portable archive may omit Git)')
    try:
        def read(*args):
            return subprocess.check_output([binary, *args], cwd=ROOT, text=True,
                stderr=subprocess.PIPE, timeout=15).strip()
        return dict(available=True, revision=read('rev-parse', 'HEAD'),
            tracked_dirty=bool(read('status', '--porcelain', '--untracked-files=no')),
            executable=binary)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        return dict(available=False, reason=type(exc).__name__)


def audit(config=CONFIG, spec_path=SPEC):
    checks = []

    def check(name, operation):
        try:
            passed, detail = operation()
            checks.append(dict(name=name, passed=bool(passed), detail=detail))
        except (ValueError, KeyError, TypeError, OSError, ImportError) as exc:
            checks.append(dict(name=name, passed=False, detail=f'{type(exc).__name__}: {exc}'))

    check('dependencies', lambda: check_versions(read_lock(ROOT/'requirements-lock.txt')))
    check('single_thread_environment', check_threads)
    result = dict(schema='sensor_preflight/1', stage='DEVELOPMENT',
        config=str(config), platform=platform.platform(), python=platform.python_version(),
        git=git_state(), checks=checks, simulations_started=0,
        development_ready=False, paper_ready=False)
    # Report absent numerical dependencies instead of crashing on import.
    if not checks[0]['passed']:
        result['paper_blockers'] = ['Local dependencies failed; remaining checks not evaluated.']
        return result
    from control.arena import check_confirmed_facts, config_sha256, load_config
    from control.sensor_binding import resolve_feedback, runtime_source_hashes
    from control.main_experiment import guard_problems, load_spec
    from control.validation_suite import baseline_params
    from scripts.sensor_reproduce import canonical_source_hashes
    check('arena_and_model', lambda: check_config(ROOT/config, None, None))

    def facts():
        problems = check_confirmed_facts(load_config(ROOT/config), baseline_params())
        return not problems, problems
    check('team_arena_invariants', facts)

    hashes = canonical_source_hashes(runtime_source_hashes())
    result['runtime_source_sha256'] = hashes

    def fixture():
        reference = json.loads((ROOT/'scripts/data/sensor_reproduction_v13.json').read_text(encoding='utf-8'))
        problems = []
        if hashes != canonical_source_hashes(reference['runtime_source_sha256']):
            problems.append('runtime differs from active v13 reproduction fixture')
        if config_sha256(load_config(ROOT/reference['config'])) != reference['config_sha256']:
            problems.append('active reference configuration changed')
        return not problems, problems or ['Source compatibility only; no numerical replay executed.']
    check('active_reference_compatible', fixture)

    if (ROOT/'sensor-workspace.json').exists():
        from scripts.sensor_workspace import verify_files
        def archive():
            _, problems = verify_files(ROOT)
            return not problems, problems
        check('portable_package_bytes', archive)
    elif not result['git'].get('available'):
        checks.append(dict(name='source_identity', passed=False,
            detail='Need a Git checkout or verified portable workspace manifest.'))

    blockers = []
    try:
        base = load_config(ROOT/config)
        binding = resolve_feedback(base)
        if binding.mode != 'sensors':
            raise ValueError('sensor preflight requires sensor feedback')
        result['config_sha256'] = config_sha256(base)
        result['feedback'] = binding.metadata
        spec = load_spec(ROOT/spec_path)
        blockers.extend(guard_problems(spec))
        paper_base = load_config(ROOT/spec['base_config']['path'])
        # A tuning seed is expected to differ; policy comparisons exclude it.
        if binding.profile != resolve_feedback(paper_base).profile:
            blockers.append('Paper specification still uses a different sensor/estimator policy.')
    except (ValueError, KeyError, TypeError, OSError) as exc:
        checks.append(dict(name='sensor_binding_and_paper_spec', passed=False, detail=str(exc)))
        blockers.append('Sensor binding/paper spec could not be audited.')
    blockers.extend([
        'The completed 93-task development study does not validate the broader operating envelope; '
        'startup sensitivity, low-RPM excursions and high-speed settling failures need follow-up.',
        'The sensor-inclusive tuning failure policy must declare how model-domain validity and '
        'pre-gust settling relate to the inherited stop_reason/paper_failed penalty.',
        'Rotor stale-feedback policy and combined actuator mismatch/dropout envelope are not validated.',
        'Sensor policy, initialization, fixed-versus-matched estimator comparisons and controller '
        'interface variants are not frozen. Device-specific claims also need hardware specification mapping.',
        'Native Windows numerical reproduction and worker lifecycle checks are not verified.',
        'Runtime source keys now use portable relative paths, but final cross-OS tuning-record '
        'loading still needs a native end-to-end check.',
        'Equal-budget final sensor tuning, design checks and validated record hashes are incomplete.',
        'Held-out sample count, uncertainty reporting and final source/configuration release are not frozen.',
    ])
    result.update(development_ready=all(c['passed'] for c in checks), paper_blockers=blockers,
        not_checked=['closed-loop performance', 'full fairness suite', 'numerical replay',
                     'Windows execution', 'worker lifecycle', 'final tuning readiness'],
        audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=CONFIG)
    parser.add_argument('--spec', default=SPEC)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require', choices=('development', 'paper'), default='development')
    args = parser.parse_args(argv)
    configure_process()
    result = audit(args.config, args.spec)
    # A fresh report path prevents accidentally replacing earlier provenance.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as file:
        json.dump(result, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('runtime_source_sha256',)}, indent=2))
    return 0 if result[args.require+'_ready'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
