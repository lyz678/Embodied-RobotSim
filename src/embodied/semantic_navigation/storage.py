"""Atomic semantic summaries bound to the geometry used to measure them."""
import gzip
import hashlib
import json
from functools import lru_cache
from pathlib import Path


def geometry_signature(bundle):
    paths = [Path(bundle) / name for name in ('bundle.json', 'map.yaml', 'map.pgm', 'map.pcd')]
    if not all(path.is_file() for path in paths):
        return None
    return _signature(tuple((str(path), path.stat().st_size, path.stat().st_mtime_ns) for path in paths))


@lru_cache(maxsize=8)
def _signature(files):
    digest = hashlib.sha256()
    for filename, _, _ in files:
        digest.update(Path(filename).name.encode())
        with open(filename, 'rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def load_state(path, signature):
    """A summary without matching geometry is not navigation evidence."""
    path = Path(path)
    compressed = Path(str(path) + '.gz')
    if path.exists():
        data = json.loads(path.read_text())
    elif compressed.exists():
        data = json.loads(gzip.decompress(compressed.read_bytes()))
    else:
        return {}
    if not signature or data.get('geometry_signature') != signature or data.get('frame') != 'map':
        return {}
    return data


def save_state(path, data):
    """JSON is the working summary; gzip is an equivalent portable snapshot."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode()
    for target, payload in ((Path(str(path) + '.gz'), gzip.compress(raw, mtime=0)), (path, raw)):
        temp = target.with_name(target.name + '.tmp')
        temp.write_bytes(payload)
        temp.replace(target)
