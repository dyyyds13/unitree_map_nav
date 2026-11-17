"""Read-only checks of publishable files; never stages, commits or pushes."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    names = subprocess.check_output(
        ['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=ROOT
    ).decode().split('\0')
    paths = sorted({p for p in names if p and (ROOT/p).is_file()})
    failures = []
    large = []
    total = 0
    # Report locations only; never print matched credential values.
    patterns = [rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
                rb'gh[pousr]_[A-Za-z0-9]{30,}', rb'github_pat_[A-Za-z0-9_]{40,}',
                rb'AKIA[0-9A-Z]{16}']
    for name in paths:
        path = ROOT/name
        size = path.stat().st_size
        total += size
        if size >= 50*1024**2:
            large.append({'path':name, 'MiB':round(size/1024**2, 2)})
        if size >= 100*1024**2:
            failures.append(f'File exceeds 100 MiB: {name}')
        if any(part in {'.venv','__pycache__','output','recordings'} for part in path.relative_to(ROOT).parts):
            failures.append(f'Runtime/generated file in publication set: {name}')
        if path.name.startswith('.env') and path.name != '.env.example' or path.suffix in {'.pem','.key'}:
            failures.append(f'Credential filename in publication set: {name}')
        if size < 2*1024**2 and path.suffix in {'.py','.md','.json','.toml','.txt','.sh','.yml','.yaml'}:
            data = path.read_bytes()
            if any(re.search(pattern,data) for pattern in patterns):
                failures.append(f'Possible credential; inspect locally: {name}')
    checked = 0
    for line in (ROOT/'SHA256SUMS').read_text().splitlines():
        expected,name = line.split(maxsplit=1)
        path = ROOT/name.lstrip('*')
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            failures.append(f'Original resource checksum mismatch: {name}')
        checked += 1
    result = {'files':len(paths), 'total_MiB':round(total/1024**2,2),
              'original_checksums_verified':checked, 'large_files_50MiB_or_more':large,
              'credential_scan':'limited pattern and filename scan, not a full history audit',
              'failures':failures}
    print(json.dumps(result,indent=2,ensure_ascii=False))
    return bool(failures)


if __name__ == '__main__':
    sys.exit(main())
