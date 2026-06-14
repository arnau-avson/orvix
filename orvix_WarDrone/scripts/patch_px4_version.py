"""Patch PX4 px_update_git_header.py for shallow clone compatibility.

The version script crashes with IndexError when nuttx git tags are
unavailable (shallow submodule clone). This patches it to handle
the missing tags gracefully.
"""

import pathlib
import sys

px4_home = sys.argv[1] if len(sys.argv) > 1 else "/root/PX4-Autopilot"
f = pathlib.Path(px4_home) / "src/lib/version/px_update_git_header.py"

if not f.exists():
    print(f"WARNING: {f} not found, skipping patch")
    sys.exit(0)

code = f.read_text()

old = (
    "nuttx_git_tag = re.findall("
    "r'nuttx-[0-9]+\\.[0-9]+\\.[0-9]+', nuttx_git_tags)"
    '[-1].replace("nuttx-", "v")'
)

new = (
    "_nuttx_m = re.findall("
    "r'nuttx-[0-9]+\\.[0-9]+\\.[0-9]+', nuttx_git_tags)\n"
    '    nuttx_git_tag = _nuttx_m[-1].replace("nuttx-", "v") '
    'if _nuttx_m else "v0.0.0"'
)

if old in code:
    code = code.replace(old, new)
    f.write_text(code)
    print("Patched px_update_git_header.py successfully")
else:
    print("WARNING: patch target not found, script may have changed")
