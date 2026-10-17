#!/usr/bin/env python3
"""Upload .wpilog and .rwnd files to the team log site with the WPI-Log-Analizer import script.

    python upload_logs.py [--team 8515] FILES_OR_FOLDERS...
    python upload_logs.py --check

Works on Windows, macOS and Linux; stdlib only, plus Node 20+ for the import script.

The site and upload key come from the environment (LOG_SITE_URL, LOG_SITE_KEY); this script never
takes the key as an argument, so it never shows up in a command line or a transcript. The analyzer
repo is found at $LOG_ANALYZER_DIR, else as a sibling of this repo (../WPI-Log-Analizer), else next
to the current folder.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_NAMES = ("WPI-Log-Analizer", "WPI-Log-Analyzer")
DEFAULT_URL = "https://robotlog.spectrum3847.org"


def find_analyzer() -> Path | None:
    env = os.environ.get("LOG_ANALYZER_DIR")
    starts = [Path(env)] if env else []
    here = Path(__file__).resolve()
    for parent in [*here.parents, Path.cwd(), *Path.cwd().parents]:
        starts += [parent / n for n in REPO_NAMES]
    for p in starts:
        if (p / "scripts" / "import.mjs").exists():
            return p
    return None


def upload_key() -> tuple[str, str]:
    """(key, where it came from). The LOG_SITE_KEY variable wins over the file set_upload_key.py saves."""
    if os.environ.get("LOG_SITE_KEY"):
        return os.environ["LOG_SITE_KEY"], "LOG_SITE_KEY variable"
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from set_upload_key import key_file, read_key  # noqa: E402

    return read_key(), str(key_file())


def check() -> tuple[Path | None, bool]:
    analyzer = find_analyzer()
    url = os.environ.get("LOG_SITE_URL") or ""
    key, source = upload_key()
    setter = Path(__file__).resolve().parent / "set_upload_key.py"
    print(f"analyzer repo : {analyzer or 'NOT FOUND (clone github.com/Spectrum3847/WPI-Log-Analizer next to this repo, or set LOG_ANALYZER_DIR)'}")
    print(f"node          : {shutil.which('node') or 'NOT FOUND (install Node 20+)'}")
    print(f"site          : {url or DEFAULT_URL}")
    if key:
        print(f"upload key    : set ({len(key)} chars, from {source})")
    else:
        print("upload key    : NOT SET. A person runs this in their own terminal (hidden prompt; ask a mentor for the key):")
        print(f"                python \"{setter}\"")
    if analyzer and not (analyzer / "node_modules").exists():
        print("note          : the first upload runs `npm install` in the analyzer repo")
    return analyzer, bool(analyzer and key and shutil.which("node"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", help=".wpilog / .rwnd files or folders")
    ap.add_argument("--team", type=int, help="override the team read from the files (e.g. 8515)")
    ap.add_argument("--check", action="store_true", help="only report whether uploads are set up")
    a = ap.parse_args()

    analyzer, ready = check()
    if a.check:
        return 0 if ready else 1
    if not ready:
        return 1
    if not a.paths:
        ap.error("give at least one file or folder")
    if not (analyzer / "node_modules").exists():
        npm = shutil.which("npm")
        if not npm or subprocess.run([npm, "install"], cwd=analyzer).returncode != 0:
            print("npm install failed in the analyzer repo", file=sys.stderr)
            return 1

    env = dict(os.environ)
    env.setdefault("LOG_SITE_URL", DEFAULT_URL)
    env["LOG_SITE_KEY"] = upload_key()[0]  # handed to the child through its environment, never argv
    paths = [str(Path(p).resolve()) for p in a.paths]
    cmd = [shutil.which("node"), "scripts/import.mjs"]
    if a.team:
        cmd += ["--team", str(a.team)]
    return subprocess.run([*cmd, *paths], cwd=analyzer, env=env).returncode


if __name__ == "__main__":
    sys.exit(main())
