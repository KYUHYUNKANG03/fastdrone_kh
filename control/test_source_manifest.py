import hashlib
from pathlib import PurePosixPath, PureWindowsPath
import pytest
from control.source_manifest import file_hashes


class Source:
    def __init__(self, path, content):
        self.path, self.content = path, content
    def relative_to(self, root):
        return self.path.relative_to(root)
    def read_bytes(self):
        return self.content


def test_native_path_flavours_share_names_without_weakening_byte_checks():
    posix = PurePosixPath('/repo')
    windows = PureWindowsPath('C:/repo')
    names = ['models/team_light/control/model.py', 'control/sensor.py']
    contents = [b'one\n', b'two\n']
    p = file_hashes([Source(posix/n, b) for n, b in zip(names, contents)], posix)
    w = file_hashes([Source(windows/n, b) for n, b in zip(names, contents)], windows)
    assert p == w and list(p) == sorted(names)
    assert p[names[0]] == hashlib.sha256(b'one\n').hexdigest()
    changed = file_hashes([Source(windows/n, b.replace(b'\n', b'\r\n'))
                          for n, b in zip(names, contents)], windows)
    assert changed != p  # Line endings and all other bytes are still pinned.


def test_duplicate_or_outside_root_paths_are_refused():
    root = PureWindowsPath('C:/repo')
    source = Source(root/'control/a.py', b'bytes')
    with pytest.raises(ValueError, match='duplicate source path'):
        file_hashes([source, source], root)
    with pytest.raises(ValueError):
        file_hashes([Source(PureWindowsPath('D:/other/a.py'), b'bytes')], root)


def test_arena_and_tuning_manifests_use_the_shared_builder(monkeypatch):
    from control import sensor_binding, validation_suite
    from control import source_manifest
    calls = []
    def marker(paths, root):
        calls.append(list(paths))
        return {'portable/path.py': 'unchanged-content-hash'}
    monkeypatch.setattr(sensor_binding, 'file_hashes', marker)
    monkeypatch.setattr(source_manifest, 'file_hashes', marker)
    assert sensor_binding.runtime_source_hashes() == validation_suite.source_hashes()
    assert len(calls) == 2 and all(calls)
