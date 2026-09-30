"""Build and check a portable DEVELOPMENT sensor workspace (not a tuning release).

The archive contains allowlisted source/data/documents, never Git credentials,
environments, or arbitrary results. Checksums detect accidental changes; they
are not a signature or a substitute for the final Git/tag and tuning guards.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import platform
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DIRECTORIES = ('control', 'models', 'external', 'configs', 'scripts', 'docs')
EXTENSIONS = {'.py', '.json', '.md', '.txt', '.sh', '.ps1'}
TOP_FILES = ('README.md', 'DISTRIBUTED_RUN.md', 'requirements-lock.txt')
MANIFEST = 'sensor-workspace.json'
CONFIG = 'configs/arena_sensor_candidate_v6.json'


def payload(root):
    root = Path(root)
    files = {name: root/name for name in TOP_FILES if (root/name).is_file()}
    for directory in DIRECTORIES:
        for path in (root/directory).rglob('*'):
            relative = path.relative_to(root)
            if any(part.startswith('.') or part == '__pycache__' for part in relative.parts):
                continue
            if path.is_file() and path.suffix in EXTENSIONS:
                if path.is_symlink() or any(p.is_symlink() for p in path.parents if p != root):
                    raise ValueError(f'symlinks cannot enter a portable workspace: {relative}')
                files[relative.as_posix()] = path
    return dict(sorted(files.items()))


def sha(data):
    return hashlib.sha256(data).hexdigest()


def config_path(name):
    parsed = PurePosixPath(name)
    if (parsed.is_absolute() or '..' in parsed.parts or '\\' in name
            or len(parsed.parts) < 2 or parsed.parts[0] != 'configs' or parsed.suffix != '.json'):
        raise ValueError('workspace config must be a relative configs/*.json path')
    return parsed.as_posix()


def build(root, output, config=CONFIG):
    root, output = Path(root), Path(output)
    config = config_path(config)
    if output.exists():
        raise ValueError('refusing to replace an existing archive; choose a new filename')
    files = {name: path.read_bytes() for name, path in payload(root).items()}
    if config not in files:
        raise ValueError(f'missing candidate configuration: {config}')
    settings = json.loads(files[config])
    digest = sha(json.dumps(settings, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode())
    doc = dict(schema='sensor_workspace/1', stage='DEVELOPMENT',
               config=config, config_sha256=digest,
               files={name: sha(data) for name, data in files.items()},
               notice='Not a tune-final-7 tag, completed tuning bundle, or paper result.')
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in files.items():
            archive.writestr(name, data)
        archive.writestr(MANIFEST, json.dumps(doc, ensure_ascii=False, indent=2)+'\n')
    return dict(archive=str(output), sha256=sha(output.read_bytes()), files=len(files),
                config=config, config_sha256=digest, stage='DEVELOPMENT')


def verify_files(root):
    root = Path(root)
    doc = json.loads((root/MANIFEST).read_text(encoding='utf-8'))
    if doc.get('schema') != 'sensor_workspace/1' or doc.get('stage') != 'DEVELOPMENT':
        raise ValueError('unsupported workspace manifest')
    selected = config_path(doc.get('config', ''))
    if selected not in doc['files']:
        raise ValueError('workspace configuration missing from manifest files')
    problems = []
    actual = payload(root)
    expected = doc['files']
    for name in sorted(set(actual) | set(expected)):
        parsed = PurePosixPath(name)
        if parsed.is_absolute() or '..' in parsed.parts or '\\' in name:
            raise ValueError('invalid path in workspace manifest')
        if name not in expected:
            problems.append(f'unrecorded source/data file: {name}')
        elif name not in actual:
            problems.append(f'missing file: {name}')
        elif sha(actual[name].read_bytes()) != expected[name]:
            problems.append(f'changed file: {name}')
    if selected in actual and not problems:
        settings = json.loads(actual[selected].read_text(encoding='utf-8'))
        digest = sha(json.dumps(settings, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode())
        if digest != doc.get('config_sha256'):
            problems.append('configuration digest differs from manifest')
    return doc, problems


def check(root):
    doc, problems = verify_files(root)
    result = dict(stage='DEVELOPMENT', platform=platform.platform(),
                  python=platform.python_version(), file_problems=problems)
    # Do not import unverified workspace code before the byte checks pass.
    if problems:
        return dict(result, passed=False)
    sys.path.insert(0, str(Path(root).resolve()))
    from scripts.setup_env import read_lock, check_versions, check_threads, check_config
    checks = dict(versions=check_versions(read_lock(Path(root)/'requirements-lock.txt')),
                  threads=check_threads(),
                  config_model=check_config(Path(root)/doc['config'], doc['config_sha256'], None))
    result['checks'] = {key: dict(passed=passed, rows=rows) for key, (passed, rows) in checks.items()}
    result['passed'] = all(passed for passed, _ in checks.values())
    result['not_checked'] = ['Git release/tag', 'Windows orphan workers',
                             'cross-machine numerical reproduction', 'final tuning readiness']
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    pack = sub.add_parser('build')
    pack.add_argument('--output', type=Path, required=True)
    pack.add_argument('--config', default=CONFIG, help='relative arena config to identify and check')
    sub.add_parser('verify')
    sub.add_parser('check')
    a = parser.parse_args(argv)
    if a.command == 'build':
        result = build(ROOT, a.output, a.config)
    elif a.command == 'verify':
        _, problems = verify_files(ROOT)
        result = dict(passed=not problems, problems=problems, stage='DEVELOPMENT')
    else:
        result = check(ROOT)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get('passed', True) else 1


if __name__ == '__main__':
    raise SystemExit(main())
