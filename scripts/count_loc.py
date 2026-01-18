#!/usr/bin/env python3
"""Count lines of code, excluding auto-generated and markdown files."""

import os
import sys
from pathlib import Path
from collections import defaultdict

# Directories to exclude (auto-generated or build artifacts)
EXCLUDE_DIRS = {
    '__pycache__',
    '.git',
    '.venv',
    'venv',
    'env',
    'node_modules',
    'build',
    'dist',
    'egg-info',
    '.eggs',
    'install',
    'log',
    'logs',
    '.mypy_cache',
    '.pytest_cache',
    '.tox',
    '.coverage',
    'htmlcov',
    '.idea',
    '.vscode',
}

# File extensions to exclude
EXCLUDE_EXTENSIONS = {
    '.md',
    '.pyc',
    '.pyo',
    '.so',
    '.o',
    '.a',
    '.dylib',
    '.egg',
    '.whl',
    '.lock',
}

# Files to exclude (auto-generated)
EXCLUDE_FILES = {
    'package-lock.json',
    'yarn.lock',
    'poetry.lock',
    'Pipfile.lock',
}

# Extensions to count
CODE_EXTENSIONS = {
    '.py': 'Python',
    '.sh': 'Shell',
    '.bash': 'Shell',
    '.yml': 'YAML',
    '.yaml': 'YAML',
    '.json': 'JSON',
    '.xml': 'XML',
    '.sdf': 'SDF (Gazebo)',
    '.jinja': 'Jinja Template',
    '.jinja2': 'Jinja Template',
    '.launch.py': 'ROS2 Launch',
    '.cmake': 'CMake',
    '.txt': 'Text/CMake',
    '.toml': 'TOML',
    '.cfg': 'Config',
    '.ini': 'Config',
    '.dockerfile': 'Docker',
}


def should_exclude_dir(dir_name: str) -> bool:
    """Check if directory should be excluded."""
    return dir_name in EXCLUDE_DIRS or dir_name.endswith('.egg-info')


def should_exclude_file(file_path: Path) -> bool:
    """Check if file should be excluded."""
    name = file_path.name
    suffix = file_path.suffix.lower()

    if name in EXCLUDE_FILES:
        return True
    if suffix in EXCLUDE_EXTENSIONS:
        return True
    return False


def get_file_type(file_path: Path) -> str | None:
    """Get the file type for counting."""
    name = file_path.name.lower()
    suffix = file_path.suffix.lower()

    # Special cases
    if name == 'dockerfile':
        return 'Docker'
    if name == 'cmakelists.txt':
        return 'CMake'
    if name.endswith('.launch.py'):
        return 'ROS2 Launch'

    return CODE_EXTENSIONS.get(suffix)


def count_lines(file_path: Path) -> tuple[int, int, int]:
    """Count total, code, and blank lines in a file."""
    total = 0
    blank = 0

    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                total += 1
                if not line.strip():
                    blank += 1
    except Exception:
        return 0, 0, 0

    code = total - blank
    return total, code, blank


def count_loc(root_path: Path, verbose: bool = False) -> dict:
    """Count lines of code in a directory tree."""
    stats = defaultdict(lambda: {'files': 0, 'total': 0, 'code': 0, 'blank': 0})
    file_list = []

    for dirpath, dirnames, filenames in os.walk(root_path):
        # Filter out excluded directories
        dirnames[:] = [d for d in dirnames if not should_exclude_dir(d)]

        for filename in filenames:
            file_path = Path(dirpath) / filename

            if should_exclude_file(file_path):
                continue

            file_type = get_file_type(file_path)
            if file_type is None:
                continue

            total, code, blank = count_lines(file_path)

            stats[file_type]['files'] += 1
            stats[file_type]['total'] += total
            stats[file_type]['code'] += code
            stats[file_type]['blank'] += blank

            if verbose:
                rel_path = file_path.relative_to(root_path)
                file_list.append((file_type, rel_path, total, code))

    return dict(stats), file_list


def print_report(stats: dict, file_list: list = None, verbose: bool = False):
    """Print the LOC report."""
    if not stats:
        print("No code files found.")
        return

    # Print per-file details if verbose
    if verbose and file_list:
        print("\n" + "=" * 70)
        print("FILE DETAILS")
        print("=" * 70)
        print(f"{'Type':<15} {'Lines':>8} {'Code':>8}  File")
        print("-" * 70)
        for file_type, path, total, code in sorted(file_list, key=lambda x: -x[2]):
            print(f"{file_type:<15} {total:>8} {code:>8}  {path}")

    # Summary by type
    print("\n" + "=" * 70)
    print("SUMMARY BY FILE TYPE")
    print("=" * 70)
    print(f"{'Language':<20} {'Files':>8} {'Total':>10} {'Code':>10} {'Blank':>10}")
    print("-" * 70)

    grand_files = 0
    grand_total = 0
    grand_code = 0
    grand_blank = 0

    for lang in sorted(stats.keys(), key=lambda x: -stats[x]['code']):
        s = stats[lang]
        print(f"{lang:<20} {s['files']:>8} {s['total']:>10} {s['code']:>10} {s['blank']:>10}")
        grand_files += s['files']
        grand_total += s['total']
        grand_code += s['code']
        grand_blank += s['blank']

    print("-" * 70)
    print(f"{'TOTAL':<20} {grand_files:>8} {grand_total:>10} {grand_code:>10} {grand_blank:>10}")
    print("=" * 70)


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Count lines of code')
    parser.add_argument('path', nargs='?', default='.', help='Directory to analyze')
    parser.add_argument('-v', '--verbose', action='store_true', help='Show per-file details')
    args = parser.parse_args()

    root = Path(args.path).resolve()

    if not root.exists():
        print(f"Error: Path '{root}' does not exist")
        sys.exit(1)

    print(f"Counting lines of code in: {root}")
    print("Excluding: markdown files, __pycache__, .git, build artifacts, etc.")

    stats, file_list = count_loc(root, verbose=args.verbose)
    print_report(stats, file_list, verbose=args.verbose)


if __name__ == '__main__':
    main()
