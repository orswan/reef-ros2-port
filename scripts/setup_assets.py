#!/usr/bin/env python3
"""Download and verify pinned third-party Gazebo assets for reef_sim.

    scripts/setup_assets.py            # download if missing or invalid, then verify
    scripts/setup_assets.py --verify   # verify only; never touches the network

Assets are listed in src/reef_sim/assets/*.json (source URL, version, license,
archive and per-file SHA-256). They are installed under assets/models/ in the
project (ignored by Git), which persists on the Mac across container
replacement and is added to GZ_SIM_RESOURCE_PATH by scripts/run_x3_scenario.sh.
REEF_ASSETS_DIR overrides that location (used by tests).

Exit status: 0 ok, 1 verification failed, 2 download or extraction failed.
"""
import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_DIR = ROOT / 'src' / 'reef_sim' / 'assets'
MODELS_DIR = Path(os.environ.get('REEF_ASSETS_DIR') or ROOT / 'assets' / 'models').resolve()
STAMP = 'REEF_ASSET.json'
NOTICE = 'ATTRIBUTION.txt'


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def verify(manifest, target):
    """Return a list of problems (empty if the installed copy matches)."""
    if not target.is_dir():
        return [f'{target} does not exist']
    problems = []
    for rel, digest in manifest['files'].items():
        p = target / rel
        if not p.is_file():
            problems.append(f'missing {rel}')
        elif sha256(p) != digest:
            problems.append(f'checksum mismatch {rel}')
    expected = set(manifest['files']) | {STAMP, NOTICE}
    for p in target.rglob('*'):
        if p.is_file() and str(p.relative_to(target)) not in expected:
            problems.append(f'unexpected file {p.relative_to(target)}')
    return problems


def install(manifest, target):
    url = manifest['source_url']
    with tempfile.TemporaryDirectory(dir=MODELS_DIR) as tmp:
        tmp = Path(tmp)
        archive = tmp / 'asset.zip'
        print(f'downloading {url}')
        try:
            with urllib.request.urlopen(url, timeout=60) as r, open(archive, 'wb') as f:
                shutil.copyfileobj(r, f)
        except OSError as e:
            print(f'FAIL download: {e}')
            return 2
        got = sha256(archive)
        if got != manifest['archive_sha256']:
            print(f'FAIL archive checksum {got} != {manifest["archive_sha256"]}')
            return 2
        staged = tmp / 'staged'
        try:
            with zipfile.ZipFile(archive) as z:
                for info in z.infolist():
                    dest = (staged / info.filename).resolve()
                    if not str(dest).startswith(str(staged.resolve())):
                        print(f'FAIL unsafe path in archive: {info.filename}')
                        return 2
                z.extractall(staged)
        except zipfile.BadZipFile as e:
            print(f'FAIL extract: {e}')
            return 2
        stamp = {k: manifest[k] for k in (
            'name', 'owner', 'fuel_version', 'source_url', 'archive_sha256', 'license', 'authors')}
        (staged / STAMP).write_text(json.dumps(stamp, indent=2) + '\n')
        (staged / NOTICE).write_text(manifest['attribution'] + '\n'
                                     + f'License: {manifest["license"]["name"]} '
                                     + f'({manifest["license"]["url"]})\n'
                                     + f'Source: {url}\n')
        problems = verify(manifest, staged)
        if problems:
            print('FAIL extracted files do not match the manifest:', *problems, sep='\n  ')
            return 1
        if target.exists():
            shutil.rmtree(target)
        staged.rename(target)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--verify', action='store_true', help='check installed assets; no download')
    args = ap.parse_args()
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    status = 0
    for mpath in sorted(MANIFEST_DIR.glob('*.json')):
        manifest = json.loads(mpath.read_text())
        target = MODELS_DIR / manifest['install_dir']
        label = f'{manifest["name"]} v{manifest["fuel_version"]} -> {target}'
        problems = verify(manifest, target)
        if not problems:
            print(f'OK   {label} ({len(manifest["files"])} files verified)')
            continue
        if args.verify:
            print(f'FAIL {label}:', *problems[:5], sep='\n  ')
            status = max(status, 1)
            continue
        rc = install(manifest, target)
        if rc:
            status = max(status, rc)
        else:
            print(f'OK   {label} installed and verified')
    if status and args.verify:
        print('Run scripts/setup_assets.py (container terminal) to download and verify.')
    return status


if __name__ == '__main__':
    sys.exit(main())
