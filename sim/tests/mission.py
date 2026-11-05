#!/usr/bin/env python3
"""Require course completion, bounded depth, and deterministic input replay."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

executable, plugin, backend, scenario, controller, simctl = [Path(p).resolve() for p in sys.argv[1:7]]
bounded = "--bounded" in sys.argv[7:]
if not executable.exists():
    print("Build the current controller with zig build to run mission integration tests.")
    raise SystemExit(77)
with tempfile.TemporaryDirectory(prefix="sim-mission-test-") as temporary:
    trace = Path(temporary) / "mission.jsonl"
    environment = {key: value for key, value in os.environ.items() if not key.startswith("AUTONOMY_SIM_")}
    environment.update(AUTONOMY_SIM_SCENARIO=str(scenario), AUTONOMY_SIM_BACKEND=str(backend), AUTONOMY_SIM_MAX_TICKS="720" if bounded else "21600", AUTONOMY_SIM_TRACE=str(trace))
    result = subprocess.run([str(executable), str(plugin), "prequalify_robosub", str(controller)], env=environment, cwd=temporary, capture_output=True, text=True, timeout=120)
    if bounded:
        assert result.returncode != 0, "expected the bounded course horizon"
        assert "error: SimulationTickBudget" in result.stderr, result.stderr
    else:
        assert result.returncode == 0, result.stderr[-5000:]
        assert "completed mission" in result.stderr, result.stderr[-5000:]
    maximum_depth = 0
    samples = 0
    with trace.open() as source:
        header = json.loads(next(source))
        vehicle = header["scenario"]["camera"]["body"]
        for line in source:
            record = json.loads(line)
            if record["t"] != "sample":
                continue
            samples += 1
            assert all(math.isfinite(v) and abs(v) <= 1 for v in record["input_commands"])
            body = next(b for b in record["bodies"] if b["id"] == vehicle)
            depth = body["pose"]["position"][2]
            assert 1.5 <= depth <= 2.6, f"lost depth control: {depth}"
            maximum_depth = max(maximum_depth, depth)
    assert record["t"] == "end" and record["status"] == int(bounded)
    assert record["step"] == samples - 1
    assert record["step"] == 720 if bounded else 720 < record["step"] < 21600
    replay = subprocess.run([str(simctl), "replay", str(backend), str(trace), str(Path(temporary) / "replay.jsonl")], capture_output=True, text=True, timeout=120)
    assert replay.returncode == 0, replay.stderr
print(f"{'bounded failure' if bounded else 'course completed'}: {record['step']} ticks, max depth {maximum_depth:.3f} m, finite commands and exact input replay passed")
