"""Closed-loop completion, bounded errors, cleanup, and deterministic repetition."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

executable, plugin, backend, config = map(lambda p: str(Path(p).resolve()), sys.argv[1:5])
env = {k: v for k, v in os.environ.items() if not k.startswith("AUTONOMY_SIM_")}
env.update(AUTONOMY_SIM_BACKEND=backend, AUTONOMY_SIM_REALTIME="0")
if len(sys.argv) > 5:
    env["LD_PRELOAD"] = sys.argv[5] + (":" + env["LD_PRELOAD"] if env.get("LD_PRELOAD") else "")

def outcome(result):
    summaries = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{"steps":')]
    assert len(summaries) == 1, result.stdout[-3000:] + result.stderr[-3000:]
    return summaries[0]

with tempfile.TemporaryDirectory(prefix="sim-zig-mission-") as temporary:
    command = [executable, plugin, "prequalify_robosub", config]
    runs = [subprocess.run(command, env=env, cwd=temporary, capture_output=True, text=True, timeout=120) for _ in range(2)]
    for result in runs:
        assert result.returncode == 0, result.stderr[-5000:]
        assert "completed mission" in result.stderr, result.stderr[-5000:]
        o = outcome(result)
        assert o["status"] == 0 and 720 < o["steps"] < 21600, o
        assert 1.5 <= o["min_depth"] <= o["max_depth"] <= 2.6, o
    assert outcome(runs[0]) == outcome(runs[1]), (outcome(runs[0]), outcome(runs[1]))
    env["AUTONOMY_SIM_MAX_TICKS"] = "720"
    result = subprocess.run(command, env=env, cwd=temporary, capture_output=True, text=True, timeout=120)
    assert result.returncode != 0 and "SimulationTickBudget" in result.stderr, result.stderr[-3000:]
    o = outcome(result)
    assert o["status"] == 1 and o["steps"] == 720, o
    env["AUTONOMY_SIM_BACKEND"] = "/no/such/backend.so"
    result = subprocess.run(command, env=env, cwd=temporary, capture_output=True, text=True, timeout=120)
    assert result.returncode != 0 and "SimulationFailed" in result.stderr, result.stderr[-3000:]
print(f"mission repeated identically: {outcome(runs[0])}")
