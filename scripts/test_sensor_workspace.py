import json
import zipfile
import pytest

from scripts.sensor_workspace import build, verify_files, MANIFEST, CONFIG


def bundle(tmp_path):
    root = tmp_path/'source'
    (root/'configs').mkdir(parents=True)
    (root/CONFIG).write_text('{}')
    (root/'control').mkdir()
    (root/'control'/'hello.py').write_text('print(1)')
    (root/'.git').mkdir()
    (root/'.git'/'config').write_text('private-token')
    (root/'results').mkdir()
    (root/'results'/'private.json').write_text('{}')
    output = tmp_path/'bundle.zip'
    receipt = build(root, output)
    destination = tmp_path/'extracted'
    with zipfile.ZipFile(output) as archive:
        assert not any('.git' in s or s.startswith('results/') for s in archive.namelist())
        archive.extractall(destination)
    return root, destination, output, receipt


def test_portable_bundle_preserves_checksums_and_excludes_private_state(tmp_path):
    _, destination, _, receipt = bundle(tmp_path)
    doc, problems = verify_files(destination)
    assert not problems and receipt['files'] == 2 and doc['stage'] == 'DEVELOPMENT'


@pytest.mark.parametrize('mode', ['change', 'remove', 'add'])
def test_verify_rejects_changed_missing_and_extra_code(tmp_path, mode):
    _, destination, _, _ = bundle(tmp_path)
    code = destination/'control'/'hello.py'
    if mode == 'change':
        code.write_text('print(2)')
    elif mode == 'remove':
        code.unlink()
    else:
        (destination/'control'/'extra.py').write_text('')
    assert len(verify_files(destination)[1]) == 1


def test_refuse_archive_overwrite_and_symlink(tmp_path):
    root, _, output, _ = bundle(tmp_path)
    with pytest.raises(ValueError, match='replace'):
        build(root, output)
    try:
        (root/'control'/'linked.py').symlink_to(root/'control'/'hello.py')
    except OSError:
        pytest.skip('host does not permit creating symlinks')
    with pytest.raises(ValueError, match='symlinks'):
        build(root, tmp_path/'another.zip')


def test_manifest_path_cannot_escape_workspace(tmp_path):
    _, destination, _, _ = bundle(tmp_path)
    path = destination/MANIFEST
    doc = json.loads(path.read_text())
    doc['files']['../secret'] = 'x'
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='invalid path'):
        verify_files(destination)


def test_selected_config_is_the_one_embedded_and_verified(tmp_path):
    root, _, _, _ = bundle(tmp_path)
    (root/'configs'/'new.json').write_text('{"candidate": 9}')
    output = tmp_path/'selected.zip'
    receipt = build(root, output, 'configs/new.json')
    assert receipt['config'] == 'configs/new.json'
    with zipfile.ZipFile(output) as archive:
        destination = tmp_path/'selected'
        archive.extractall(destination)
    doc, problems = verify_files(destination)
    assert not problems and doc['config'] == 'configs/new.json'
    doc['config_sha256'] = 'stale'
    (destination/MANIFEST).write_text(json.dumps(doc))
    assert verify_files(destination)[1] == ['configuration digest differs from manifest']
    with pytest.raises(ValueError, match='relative'):
        build(root, tmp_path/'unsafe.zip', '../secret.json')
