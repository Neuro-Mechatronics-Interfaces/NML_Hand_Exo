#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
print_tree.py — Python equivalent of the MATLAB print_tree function.

Usage (examples):
  python print_tree.py
  python print_tree.py "C:\\MyProject"
  python print_tree.py --Start "C:\\MyProject" --MaxDepth 3 --PrintFolderSize true --SortBy FileCount
  python print_tree.py --Start . --Root . --OutputFile tree_output.txt --FolderFilter "*/Data" --FileFilter "*.m"
  python print_tree.py --FolderSizeLim -inf 10 --FolderFileCountLim 10 1e9 --FileExcluder "*.tmp" --FolderExcluder "__pycache__" ".git"

Notes:
- Sizes displayed in human-readable units (B, KB, MB, GB).
- Folder size limits are in GB (same as MATLAB arg).
- Folder filter uses slash-delimited wildcards by level (e.g., '*/Data').
"""

import argparse
import fnmatch
import math
import os
import sys
from typing import Dict, List, Tuple

# ---------- Argument parsing helpers ----------

def str2bool(v: str) -> bool:
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in ("1", "true", "t", "yes", "y", "on"):
        return True
    if s in ("0", "false", "f", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Expected boolean, got '{v}'")

def finite_or_inf(s: str) -> float:
    s = s.strip()
    if s.lower() in ("inf", "+inf", "infinity", "+infinity"):
        return math.inf
    if s.lower() in ("-inf", "-infinity"):
        return -math.inf
    try:
        return float(s)
    except ValueError:
        raise argparse.ArgumentTypeError(f"Expected number or +/-inf, got '{s}'")

# ---------- Human-readable byte formatting ----------

def format_bytes(b: int) -> str:
    if b < 1024:
        return f"{b:d} B"
    kb = b / 1024.0
    if kb < 1024:
        return f"{kb:.1f} KB"
    mb = kb / 1024.0
    if mb < 1024:
        return f"{mb:.1f} MB"
    gb = mb / 1024.0
    return f"{gb:.2f} GB"

# ---------- Size cache ----------

class SizeCache:
    def __init__(self):
        self._cache: Dict[str, Tuple[int, int]] = {}

    def clear(self):
        self._cache.clear()

    def get_folder_size(self, p: str) -> Tuple[int, int]:
        # normalize to native absolute path (no slow Java; mimic MATLAB canonical intent)
        p = os.path.abspath(p)
        if p in self._cache:
            return self._cache[p]

        try:
            entries = list(os.scandir(p))
        except PermissionError:
            self._cache[p] = (0, 0)
            return 0, 0
        except FileNotFoundError:
            self._cache[p] = (0, 0)
            return 0, 0

        sz = 0
        count = 0
        subdirs: List[str] = []
        for e in entries:
            # Skip dot entries like MATLAB version
            if e.name.startswith('.'):
                continue
            try:
                if e.is_file(follow_symlinks=False):
                    try:
                        stat = e.stat(follow_symlinks=False)
                        sz += stat.st_size
                        count += 1
                    except OSError:
                        pass
                elif e.is_dir(follow_symlinks=False):
                    subdirs.append(e.path)
            except OSError:
                continue

        for sub in subdirs:
            ssz, scount = self.get_folder_size(sub)
            sz += ssz
            count += scount

        self._cache[p] = (sz, count)
        return sz, count

# ---------- Folder filter helpers ----------

def split_folder_filter(folder_filter: str) -> List[str]:
    # Replace backslashes with forward slashes and split like MATLAB
    return folder_filter.replace("\\", "/").split("/")

def wildcard_match(name: str, pattern: str) -> bool:
    return fnmatch.fnmatchcase(name, pattern)

# ---------- Core tree writer ----------

def write_tree(path: str,
               fid,
               base_level: int,
               max_levels: float,
               file_filter: str,
               folder_filter_parts: List[str],
               show_size: bool,
               size_limits_bytes: Tuple[float, float],
               count_limits: Tuple[float, float],
               sort_by: str,
               print_files: bool,
               max_files_to_print: float,
               file_excluders: List[str],
               folder_excluders: List[str],
               size_cache: SizeCache,
               progress: bool = False,
               prog_base: float = 0.0,
               prog_width: float = 1.0) -> None:

    # Gather entries, skipping dot-prefixed names like MATLAB version
    try:
        entries = list(os.scandir(path))
    except (FileNotFoundError, PermissionError):
        return
    entries = [e for e in entries if not e.name.startswith('.')]

    dirs = [e for e in entries if e.is_dir(follow_symlinks=False)]
    folder_names = [e.name for e in dirs]

    level = base_level + 1
    if level <= len(folder_filter_parts):
        pattern = folder_filter_parts[level - 1]
    else:
        pattern = "*"

    # Folder include by wildcard at this depth
    keep_mask = [wildcard_match(n, pattern) for n in folder_names]

    # Exclude by FolderExcluder (list of wildcard patterns)
    if folder_excluders:
        excl_mask = []
        for n in folder_names:
            any_match = any(wildcard_match(n, pat) for pat in folder_excluders)
            excl_mask.append(not any_match)
        keep_mask = [k and e for k, e in zip(keep_mask, excl_mask)]

    # Compute sizes & counts for kept dirs
    kept_dirs = [d for d, k in zip(dirs, keep_mask) if k]
    sizes: List[int] = []
    counts: List[int] = []
    n = len(kept_dirs)

    if progress and n > 0 and fid is not sys.stdout:
        print("Please wait, determining folder file-counts and sizes...000%", file=sys.stdout)

    for ii, d in enumerate(kept_dirs, start=1):
        s, c = size_cache.get_folder_size(d.path)
        sizes.append(s)
        counts.append(c)
        if progress and n > 0 and fid is not sys.stdout:
            pct = round(ii * 100 / n)
            # overwrite the 3 digits
            print(f"\b\b\b\b\b{pct:03d}%", end="\n", file=sys.stdout, flush=True)

    # Apply size/count limits
    within_size = [size_limits_bytes[0] <= s <= size_limits_bytes[1] for s in sizes]
    within_count = [count_limits[0] <= c <= count_limits[1] for c in counts]
    keep2 = [a and b for a, b in zip(within_size, within_count)]

    dirs2 = [d for d, k in zip(kept_dirs, keep2) if k]
    folder_sizes = [s for s, k in zip(sizes, keep2) if k]
    folder_counts = [c for c, k in zip(counts, keep2) if k]

    # Sorting
    if sort_by == "FolderSize":
        order = sorted(range(len(dirs2)), key=lambda i: folder_sizes[i], reverse=True)
    elif sort_by == "FileCount":
        order = sorted(range(len(dirs2)), key=lambda i: folder_counts[i], reverse=True)
    else:
        order = list(range(len(dirs2)))

    dirs2 = [dirs2[i] for i in order]
    folder_sizes = [folder_sizes[i] for i in order]
    folder_counts = [folder_counts[i] for i in order]

    # Filter files by file_filter and excluders
    files = []
    for e in entries:
        if e.is_file(follow_symlinks=False):
            if fnmatch.fnmatchcase(e.name, file_filter):
                files.append(e)

    if file_excluders:
        tmp = []
        for e in files:
            if not any(fnmatch.fnmatchcase(e.name, pat) for pat in file_excluders):
                tmp.append(e)
        files = tmp

    n_dirs = len(dirs2)

    # Print subdirectories
    for ii, sub in enumerate(dirs2, start=1):
        size_str = ""
        if show_size:
            sz = folder_sizes[ii - 1]
            size_str = f" [{format_bytes(sz)} ({folder_counts[ii - 1]} Files)]"

        indent = "│   " * (base_level + 1)
        print(f"{indent}├── {sub.name}/{size_str}", file=fid)

        # Calculate new progress range for this subdirectory
        sub_prog_base = prog_base + (ii - 1) / n_dirs * prog_width if n_dirs else prog_base
        sub_prog_width = prog_width / n_dirs if n_dirs else prog_width

        # Recurse if within max_levels
        if base_level + 1 <= max_levels:
            write_tree(
                sub.path, fid, base_level + 1, max_levels, file_filter,
                folder_filter_parts, show_size, size_limits_bytes, count_limits,
                sort_by, print_files, max_files_to_print, file_excluders,
                folder_excluders, size_cache, progress, sub_prog_base, sub_prog_width
            )

        # Print progress after each subdir (mimic MATLAB behavior roughly)
        if progress and fid is not sys.stdout and n_dirs:
            pct = round((sub_prog_base + sub_prog_width) * 100)
            print(f"\b\b\b\b\b{pct:03d}%", end="\n", file=sys.stdout, flush=True)

    # Print files
    if print_files:
        limit = len(files) if math.isinf(max_files_to_print) else int(max_files_to_print)
        for idx, e in enumerate(files[:limit], start=1):
            is_last = (idx == min(len(files), limit))
            connector = "└── " if is_last else "├── "
            indent = "│   " * (base_level + 1)
            print(f"{indent}{connector}{e.name}", file=fid)
    elif n_dirs == 0:
        print(f"{'│   ' * (base_level + 1)}└── [no subfolders]", file=fid)

# ---------- Main ----------

def main():
    parser = argparse.ArgumentParser(
        description="Recursively prints a visual directory tree with file and folder stats (MATLAB print_tree equivalent)."
    )
    # Positional 'root' (optional). Default "" like MATLAB (resolved later from Start/Root).
    parser.add_argument("root", nargs="?", default="", help="Root path (optional). If provided, affects Start/Root logic like MATLAB.")
    # MATLAB-style option names as flags:
    parser.add_argument("--ClearCache", type=str2bool, default=False)
    parser.add_argument("--Start", default=os.getcwd())
    parser.add_argument("--OutputFile", default="")
    parser.add_argument("--Root", default="")
    parser.add_argument("--MaxDepth", type=float, default=math.inf)
    parser.add_argument("--FileFilter", default="*.*")
    parser.add_argument("--FileExcluder", nargs="*", default=[])
    parser.add_argument("--FolderFilter", default="*")
    parser.add_argument("--FolderExcluder", nargs="*", default=[])
    parser.add_argument("--PrintFolderSize", type=str2bool, default=True)
    parser.add_argument("--PrintFiles", type=str2bool, default=True)
    parser.add_argument("--MaxFilesToPrint", type=float, default=math.inf)
    parser.add_argument("--FolderSizeLim", nargs=2, type=finite_or_inf, default=[-math.inf, math.inf],
                        metavar=("MIN_GB", "MAX_GB"))
    parser.add_argument("--FolderFileCountLim", nargs=2, type=finite_or_inf, default=[-math.inf, math.inf],
                        metavar=("MIN", "MAX"))
    parser.add_argument("--SortBy", choices=["FolderSize", "FileCount", "None"], default="FolderSize")

    args = parser.parse_args()

    # Normalize OutputFile path: ensure .txt extension, and default directory behavior like MATLAB
    outfile = args.OutputFile.strip()
    out_dir, out_base = os.path.split(outfile)
    out_stem, _ = os.path.splitext(out_base)
    if outfile:
        outfile = os.path.join(out_dir, f"{out_stem}.txt")

    # Apply MATLAB-like logic relating root/Start/Root/OutputFile
    root = args.root.strip()
    options_root = args.Root.strip()
    startpath = args.Start.strip()

    if root:
        if not options_root:
            options_root = root
        if not out_dir and outfile:
            # put outfile within root if no directory given
            outfile = os.path.join(root, f"{out_stem}.txt")
        if os.path.normcase(os.path.abspath(startpath)) != os.path.normcase(os.path.abspath(root)):
            startpath = root

    # Choose effective root: if options_root set, use it; else use startpath
    effective_root = options_root if options_root else startpath

    # Canonical/real paths (like MATLAB's Java canonical path)
    startpath = os.path.realpath(startpath)
    effective_root = os.path.realpath(effective_root)

    # Validate start within root
    # (Allow equality; require startpath to start with root path + separator or be exactly equal)
    norm_start = os.path.normcase(startpath)
    norm_root = os.path.normcase(effective_root)
    if not (norm_start == norm_root or norm_start.startswith(os.path.join(norm_root, ""))):
        print(f"Error: Path {startpath} is not inside root {effective_root}.", file=sys.stderr)
        sys.exit(1)

    # Open output (stdout or file)
    print_to_file = bool(outfile)
    if print_to_file:
        try:
            os.makedirs(os.path.dirname(outfile), exist_ok=True)
        except Exception:
            pass
        try:
            fid = open(outfile, "w", encoding="utf-8", newline="\n")
        except OSError as e:
            print(f"Failed to open file for writing: {outfile}\n{e}", file=sys.stderr)
            sys.exit(1)
    else:
        # Clear screen like MATLAB when not writing to file
        if os.name == "nt":
            os.system("cls")
        else:
            os.system("clear")
        fid = sys.stdout

    # Header (project folder name)
    project_name = os.path.basename(os.path.normpath(effective_root)) or effective_root
    print(f"{project_name}/", file=fid)

    # Compute relative parts between root and start (for indentation baseline)
    rel = os.path.relpath(startpath, effective_root)
    if rel == ".":
        relative_parts: List[str] = []
    else:
        relative_parts = [p for p in rel.split(os.sep) if p]

    folder_filter_parts = split_folder_filter(args.FolderFilter)
    folder_size_lim_bytes = (args.FolderSizeLim[0] * (1024 ** 3), args.FolderSizeLim[1] * (1024 ** 3))

    size_cache = SizeCache()
    if args.ClearCache:
        size_cache.clear()

    # Optionally announce progress to stdout when writing to file (to mimic MATLAB behavior)
    progress = print_to_file

    # Kick off recursive write
    write_tree(
        startpath,
        fid,
        base_level=len(relative_parts),
        max_levels=args.MaxDepth,
        file_filter=args.FileFilter,
        folder_filter_parts=folder_filter_parts,
        show_size=args.PrintFolderSize,
        size_limits_bytes=folder_size_lim_bytes,
        count_limits=(args.FolderFileCountLim[0], args.FolderFileCountLim[1]),
        sort_by=args.SortBy,
        print_files=args.PrintFiles,
        max_files_to_print=args.MaxFilesToPrint,
        file_excluders=args.FileExcluder,
        folder_excluders=args.FolderExcluder,
        size_cache=size_cache,
        progress=progress
    )

    if print_to_file:
        fid.close()
        print(f"Pretty tree written to {outfile}")

if __name__ == "__main__":
    main()
