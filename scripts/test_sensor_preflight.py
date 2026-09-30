import json
import os

import pytest

from scripts import sensor_preflight as preflight, setup_env, verify_arena


def test_missing_dependency_is_failure_even_on_windows(monkeypatch):
    monkeypatch.setattr(setup_env.platform, 'system', lambda: 'Windows')
    monkeypatch.setattr(setup_env.platform, 'machine', lambda: 'AMD64')
    monkeypatch.setattr(setup_env.platform, 'python_version', lambda: '3.13.7')
    def missing(_):
        raise setup_env.PackageNotFoundError()
    monkeypatch.setattr(setup_env, 'version', missing)
    ok, rows = setup_env.check_versions({'casadi': '3.7.2'})
    assert not ok and rows[-1][-1] == 'FAIL'


def test_windows_memory_is_unknown_not_zero(monkeypatch):
    monkeypatch.setattr(verify_arena, 'resource', None)
    assert verify_arena.peak_memory_gb() is None
    assert 'unavailable' in verify_arena.memory_text(None)


def test_audit_fails_cleanly_before_import_on_missing_dependency(monkeypatch):
    monkeypatch.setattr(preflight, 'check_versions', lambda _: (False, ['missing']))
    monkeypatch.setattr(preflight, 'git_state', lambda: {'available': False})
    result = preflight.audit()
    assert not result['development_ready'] and not result['paper_ready']
    assert result['simulations_started'] == 0


def test_static_pass_does_not_open_paper_gate(monkeypatch):
    monkeypatch.setattr(preflight, 'check_versions', lambda _: (True, []))
    monkeypatch.setattr(preflight, 'check_threads', lambda: (True, []))
    monkeypatch.setattr(preflight, 'check_config', lambda *args: (True, []))
    monkeypatch.setattr(preflight, 'git_state', lambda: {'available': True})
    result = preflight.audit()
    assert result['development_ready'] and not result['paper_ready']
    assert any('different sensor/estimator policy' in s for s in result['paper_blockers'])
    assert sum('tuning record sha256 is empty' in s for s in result['paper_blockers']) == 5
    assert 'numerical replay' in result['not_checked']


def test_cli_paper_requirement_fails_and_report_cannot_be_overwritten(tmp_path, monkeypatch):
    monkeypatch.setattr(preflight, 'configure_process', lambda: None)
    monkeypatch.setattr(preflight, 'audit', lambda *a: dict(development_ready=True, paper_ready=False))
    path = tmp_path/'audit.json'
    assert preflight.main(['--output', str(path), '--require', 'paper']) == 1
    assert json.loads(path.read_text())['paper_ready'] is False
    with pytest.raises(FileExistsError):
        preflight.main(['--output', str(path)])


def test_process_environment_sets_threads_and_git_without_touching_profiles(monkeypatch):
    monkeypatch.setattr(preflight, 'git_binary', lambda: '/example/cli/git')
    monkeypatch.setattr(os, 'environ', {'PATH': '/prior/bin'})
    preflight.configure_process()
    assert all(os.environ[k] == '1' for k in setup_env.THREAD_VARIABLES)
    assert os.environ['PATH'] == '/example/cli'+os.pathsep+'/prior/bin'


def test_windows_manifest_keys_match_without_weakening_byte_hashes():
    from scripts.sensor_reproduce import canonical_source_hashes
    assert canonical_source_hashes({'control\\sample.py': 'digest'}) == {'control/sample.py': 'digest'}
    assert canonical_source_hashes({'control\\sample.py': 'changed'}) != {'control/sample.py': 'digest'}
    with pytest.raises(ValueError, match='duplicate'):
        canonical_source_hashes({'control\\sample.py': 'a', 'control/sample.py': 'b'})
