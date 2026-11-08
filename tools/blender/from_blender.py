#!/usr/bin/env python3
"""Launch the application configured in config/application.zig.

Set .scene to the saved .blend or AUVDATA1 scene path, then use zig build.
The native plugin exports .blend geometry through the packaged binary exporter.
"""
from pathlib import Path
import subprocess
repo=Path(__file__).resolve().parents[2]
subprocess.run(['zig','build','-Dsimulator=true'],cwd=repo,check=True)
executable=repo/'zig-out/bin/src_3.exe'
if not executable.is_file():executable=repo/'zig-out/bin/src_3'
raise SystemExit(subprocess.run([str(executable),'run'],cwd=repo).returncode)
