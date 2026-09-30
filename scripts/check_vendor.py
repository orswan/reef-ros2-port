#!/usr/bin/env python3
"""Check that vendored upstream files are byte-identical to their pin.

Recomputes Git object IDs (tree for directories, blob for files) from the
working files, so uncommitted edits are detected too, and compares them with
UPSTREAM.json. With --upstream CLONE it also resolves the tag in a clone of
the upstream repository and checks the recorded commit and IDs against it.

Exit status: 0 match, 1 mismatch, 2 invalid input or tool error.
"""
import argparse
import hashlib
import json
import os
import stat
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIR = ROOT / 'src' / 'third_party' / 'rosflight_ros_pkgs'


def git_object_id(kind, data):
    return hashlib.sha1(b'%s %d\0' % (kind, len(data)) + data).hexdigest()


def blob_id(path):
    p = Path(path)
    data = os.readlink(p).encode() if p.is_symlink() else p.read_bytes()
    return git_object_id(b'blob', data)


def tree_id(path):
    """Git tree ID of a directory, as `git write-tree` would record it."""
    entries = []
    for child in Path(path).iterdir():
        mode = child.lstat().st_mode
        if stat.S_ISLNK(mode):
            entries.append((child.name, b'120000', blob_id(child)))
        elif child.is_dir():
            entries.append((child.name + '/', b'40000', tree_id(child)))
        else:
            exe = mode & stat.S_IXUSR
            entries.append((child.name, b'100755' if exe else b'100644', blob_id(child)))
    body = b''
    for name, mode, oid in sorted(entries, key=lambda e: e[0].encode()):
        body += mode + b' ' + name.rstrip('/').encode() + b'\0' + bytes.fromhex(oid)
    return git_object_id(b'tree', body)


def git(clone, *args):
    return subprocess.run(['git', '-C', str(clone), *args], check=True,
                          capture_output=True, text=True).stdout.strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--dir', type=Path, default=DEFAULT_DIR,
                    help='vendored directory containing UPSTREAM.json')
    ap.add_argument('--upstream', type=Path, help='clone of the upstream repository')
    args = ap.parse_args()

    lock_path = args.dir / 'UPSTREAM.json'
    try:
        lock = json.loads(lock_path.read_text())
    except (OSError, ValueError) as e:
        print(f'check_vendor: cannot read {lock_path}: {e}', file=sys.stderr)
        return 2

    failures = 0
    for name, want in lock['vendored'].items():
        path = args.dir / name
        if not path.exists():
            print(f'FAIL {name}: missing')
            failures += 1
            continue
        kind, expected = ('git_tree', want['git_tree']) if 'git_tree' in want \
            else ('git_blob', want['git_blob'])
        actual = tree_id(path) if kind == 'git_tree' else blob_id(path)
        ok = actual == expected
        failures += not ok
        print(f'{"PASS" if ok else "FAIL"} {name}: {kind} {actual}'
              + ('' if ok else f' (expected {expected})'))

    if args.upstream:
        try:
            commit = git(args.upstream, 'rev-parse', lock['tag'] + '^{commit}')
            ok = commit == lock['commit']
            failures += not ok
            print(f'{"PASS" if ok else "FAIL"} upstream tag {lock["tag"]} -> {commit}')
            for name, want in lock['vendored'].items():
                expected = want.get('git_tree') or want.get('git_blob')
                actual = git(args.upstream, 'rev-parse', f'{lock["commit"]}:{name}')
                ok = actual == expected
                failures += not ok
                print(f'{"PASS" if ok else "FAIL"} upstream {lock["commit"][:8]}:{name} -> {actual}')
        except (subprocess.CalledProcessError, OSError) as e:
            print(f'check_vendor: upstream check failed: {e}', file=sys.stderr)
            return 2

    print(f'check_vendor: {"PASS" if failures == 0 else "FAIL"} ({args.dir})')
    return 0 if failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
