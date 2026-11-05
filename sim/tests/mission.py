#!/usr/bin/env python3
"""Require course completion, bounded depth, and the same result from a second run."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

def result_line(stdout):
    for line in reversed(stdout.splitlines()):
        text = line.strip()
        if text.startswith("{") and '"min_depth"' in text:
            return json.loads(text)
    raise AssertionError(stdout[-2000:])

executable, plugin, backend, scenario, controller, simctl = [Path(p).resolve() for p in sys.argv[1:7]]
bounded = "--bounded" in sys.argv[7:]
if not executable.exists():
    print("Build the current controller with zig build to run mission integration tests.")
    raise SystemExit(77)
with tempfile.TemporaryDirectory(prefix="sim-mission-test-") as temporary:
    environment = {key: value for key, value in os.environ.items() if not key.startswith("AUTONOMY_SIM_")}
    environment.update(AUTONOMY_SIM_SCENARIO=str(scenario), AUTONOMY_SIM_BACKEND=str(backend), AUTONOMY_SIM_MAX_TICKS="720" if bounded else "21600")
    command = [str(executable), str(plugin), "prequalify_robosub", str(controller)]
    runs = []
    for _ in range(2):
        result = subprocess.run(command, env=environment, cwd=temporary, capture_output=True, text=True, timeout=120)
        runs.append(result)
    result = runs[0]
    outcome = result_line(result.stdout)
    if bounded:
        assert result.returncode != 0, "expected the bounded course horizon"
        assert "error: SimulationTickBudget" in result.stderr, result.stderr
        assert outcome["steps"] == 720 and outcome["status"] == 1, outcome
    else:
        assert result.returncode == 0, result.stderr[-5000:]
        assert "completed mission" in result.stderr, result.stderr[-5000:]
        assert 720 < outcome["steps"] < 21600 and outcome["status"] == 0, outcome
    assert 1.5 <= outcome["min_depth"] <= outcome["max_depth"] <= 2.6, outcome
    again = result_line(runs[1].stdout)
    assert again["hash"] == outcome["hash"] and again["steps"] == outcome["steps"], (outcome, again)
print(f"{'bounded failure' if bounded else 'course completed'}: {outcome['steps']} ticks, depth {outcome['min_depth']:.3f}–{outcome['max_depth']:.3f} m, second run matched")
