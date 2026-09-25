#!/usr/bin/env python3
"""Fail unless the image's pip layer is EXACTLY docker/pip-constraints.txt.

usage: check_pip_lock.py CONSTRAINTS_FILE

Compares every distribution installed under /usr/local (Debian's pip
target; apt's Python lives in /usr/lib and is not pip's business) with the
pinned set, by normalised name and exact version, in both directions: a
missing pin fails, and so does an extra distribution nobody pinned -- the
second is how a new transitive dependency would otherwise slip in.
"""
import importlib.metadata as md
import re
import sys


def norm(name):
    return re.sub(r'[-_.]+', '-', name).lower()


def main():
    want = {}
    for raw in open(sys.argv[1]):
        line = raw.split('#', 1)[0].strip()
        if line:
            name, version = line.split('==')
            want[norm(name)] = version.strip()
    site = '/usr/local/lib/python{}.{}/dist-packages'.format(*sys.version_info)
    have = {norm(d.metadata['Name']): d.version
            for d in md.distributions(path=[site])}
    problems = []
    for name, version in sorted(want.items()):
        if name not in have:
            problems.append(f'pinned but not installed: {name}=={version}')
        elif have[name] != version:
            problems.append(f'version drift: {name} {have[name]} != {version}')
    for name in sorted(set(have) - set(want)):
        problems.append(f'installed but not pinned: {name}=={have[name]}')
    if problems:
        print('pip layer does not match the lock:', *problems, sep='\n  ')
        return 1
    print(f'pip layer matches the lock: {len(want)} distributions under {site}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
