"""Verify local model bytes, including preprocessing and semantic label configuration."""
import hashlib
from pathlib import Path


def sha(path):
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    h = hashlib.sha256()
    if path.is_file():
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024*1024), b''):
                h.update(chunk)
    else:
        files = sorted(f for f in path.rglob('*') if f.is_file())
        if not files:
            raise ValueError('Empty model directory')
        for f in files:
            h.update(str(f.relative_to(path)).encode())
            with f.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    h.update(chunk)
    return h.hexdigest()


def verify_models(root, manifest):
    root = Path(root).resolve()
    for name in ('floor', 'depth', 'detector'):
        entry = manifest[name]
        assets = entry.get('assets') if name == 'floor' else {entry['path']: entry['sha256']}
        if not assets or name == 'floor' and not {'segformer-b0/config.json', 'segformer-b0/preprocessor_config.json'} <= set(assets):
            raise ValueError('Model manifest needs weights and configuration hashes')
        if name == 'floor' and not any(k.endswith(('.safetensors', '.bin')) and v == entry['sha256'] for k,v in assets.items()):
            raise ValueError('Floor weights hash missing')
        for relative, expected in assets.items():
            path = (root / relative).resolve()
            if not path.is_relative_to(root) or sha(path) != expected:
                raise ValueError(f'Model asset changed: {name}/{relative}')
