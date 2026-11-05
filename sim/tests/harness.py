#!/usr/bin/env python3
"""Exercise the command-line harness, worker-count independence and failure reducer."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
simctl, backend = map(lambda p: Path(p).resolve(), sys.argv[1:3])
with tempfile.TemporaryDirectory(prefix="sim-harness-test-") as temporary:
    directory = Path(temporary)
    fixture = {"version": 1, "gravity": [0, 0, 0], "bodies": [{"id": 1, "motion": 1, "mass": 2, "inertia": [1, 1, 1], "pose": {"position": [0, 0, 2]}, "shapes": [{"kind": 0, "size": [.1, .2, .3]}]}]}
    scenario = directory / "scenario.json"
    scenario.write_text(json.dumps(fixture))
    summaries = []
    for workers in [1, 2, 4]:
        output = directory / f"workers-{workers}"
        subprocess.run([sys.executable, str(root / "sim/fuzz.py"), "--simctl", str(simctl), "--backend", str(backend), "--scenario", str(scenario), "--cases", "4", "--workers", str(workers), "--ticks", "12", "--verify-replay", "--output", str(output)], check=True)
        summaries.append(json.loads((output / "summary.json").read_text()))
    assert summaries[0] == summaries[1] == summaries[2], "worker counts changed a trajectory"
    fixture["assertions"] = {"max_position_norm": .01}
    scenario.write_text(json.dumps(fixture))
    output = directory / "failure"
    result = subprocess.run([sys.executable, str(root / "sim/fuzz.py"), "--simctl", str(simctl), "--backend", str(backend), "--scenario", str(scenario), "--cases", "1", "--ticks", "12", "--reduce-failures", "--output", str(output)])
    assert result.returncode == 1, "fuzzer did not detect an invariant failure"
    case = next(output.glob("case-*"))
    reduced = json.loads((case / "reduced-replay.json").read_text())
    assert reduced["ticks"] == 1, "reducer did not find the first failing tick"
    assert "position bound exceeded" in reduced["condition"]
    replay = subprocess.run(reduced["command"], capture_output=True, text=True)
    assert replay.returncode and "position bound exceeded" in replay.stderr, "reduced case did not reproduce"
print("harness and reducer passed")
