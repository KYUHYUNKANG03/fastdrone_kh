"""Stable relative source names with strict, unchanged content fingerprints."""
import hashlib


def file_hashes(paths, root):
    entries = {}
    for path in paths:
        name = path.relative_to(root).as_posix()
        if name in entries:
            raise ValueError(f'duplicate source path: {name}')
        entries[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(sorted(entries.items()))
