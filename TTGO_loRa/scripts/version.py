

Import("env")

import subprocess

project_dir = env.subst("$PROJECT_DIR")

try:
    git_hash = subprocess.check_output(
        ["git", "rev-parse", "--short=12", "HEAD"],
        cwd=project_dir,
        stderr=subprocess.DEVNULL,
        text=True
    ).strip()
except Exception:
    git_hash = "nogit"

env.Append(CPPDEFINES=[
    ("FW_VERSION", '\\"0.1.0\\"'),
    ("FW_GIT_HASH", '\\"' + git_hash + '\\"')
])

print("Firmware version: 0.1.0")
print("Git commit: " + git_hash)
