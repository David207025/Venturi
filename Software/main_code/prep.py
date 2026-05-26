import os
from SCons.Script import Import

# Explicitly pull the active SCons construction environment into this script scope
Import("env")

# Define the absolute path to your flat toolchain binaries
toolchain_bin = os.path.expanduser("~/embedded_tools/riscv32-esp/bin")

# 1. Inject it directly into PlatformIO's internal SCons build PATH
env.PrependENVPath("PATH", toolchain_bin)

# 2. Force update the active OS environment block for child processes (shell spawns)
os.environ["PATH"] = toolchain_bin + os.path.pathsep + os.environ.get("PATH", "")