#!/usr/bin/env python3
"""Save the log site's upload key for this user on this machine. A person runs this, not an agent.

    python set_upload_key.py           # prompts for the key (hidden), saves it
    python set_upload_key.py --status  # says whether a key is saved, never prints it
    python set_upload_key.py --clear   # removes the saved key

Works on Windows, macOS and Linux; stdlib only.

The key is typed at a hidden prompt and written to a file in your home folder, outside every repo:
    Windows        %APPDATA%\\spectrum-logs\\upload_key
    macOS / Linux  ~/.config/spectrum-logs/upload_key   (mode 600)
upload_logs.py reads it from there (a LOG_SITE_KEY environment variable wins if set). Nothing needs
restarting. The repos are public: never commit the key, paste it into a file in a repo, or type it
into a chat with an agent. Ask a mentor for the key.
"""

from __future__ import annotations

import argparse
import getpass
import os
import sys
from pathlib import Path


def key_file() -> Path:
    if os.name == "nt":
        base = Path(os.environ.get("APPDATA") or Path.home() / "AppData" / "Roaming")
    else:
        base = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    return base / "spectrum-logs" / "upload_key"


def read_key() -> str:
    """The saved key, or "" if none. Used by upload_logs.py."""
    try:
        return key_file().read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def interactive() -> bool:
    """True only when a person can type at this process. On Windows NUL reports isatty() and
    getpass would wait on a console nobody sees, so ask the console itself."""
    if not sys.stdin.isatty():
        return False
    if os.name != "nt":
        return True
    import ctypes
    import msvcrt

    mode = ctypes.c_uint32()
    handle = msvcrt.get_osfhandle(sys.stdin.fileno())
    return bool(ctypes.windll.kernel32.GetConsoleMode(handle, ctypes.byref(mode)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--status", action="store_true", help="report whether a key is saved")
    ap.add_argument("--clear", action="store_true", help="remove the saved key")
    a = ap.parse_args()
    path = key_file()

    if a.status:
        k = read_key()
        print(f"upload key: {'saved' if k else 'not saved'} ({path})")
        if os.environ.get("LOG_SITE_KEY"):
            print("LOG_SITE_KEY environment variable is also set (it takes precedence)")
        return 0 if k or os.environ.get("LOG_SITE_KEY") else 1
    if a.clear:
        path.unlink(missing_ok=True)
        print(f"removed {path}")
        return 0

    if not interactive():
        print("Run this yourself in a terminal; it asks for the key at a hidden prompt.\n"
              f"  python {Path(__file__).name}", file=sys.stderr)
        return 2
    key = getpass.getpass("Upload key (hidden): ").strip()
    if not key:
        print("nothing entered; no change", file=sys.stderr)
        return 1
    if getpass.getpass("Again to confirm: ").strip() != key:
        print("did not match; no change", file=sys.stderr)
        return 1
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.unlink(missing_ok=True)
    with os.fdopen(os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), "w", encoding="utf-8") as fh:
        fh.write(key + "\n")
    tmp.replace(path)
    print(f"saved to {path} ({len(key)} characters). Uploads will use it; the first upload confirms it is right.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
