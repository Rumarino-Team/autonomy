#!/usr/bin/env python3
"""Exercise simctl replay, worker-count independence and the failure reducer."""
import base64
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("scene_wire", root/"tools/blender/autonomy_sim/wire.py")
wire = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wire)
simctl, backend = map(lambda p: Path(p).resolve(), sys.argv[1:3])

def run(payload, ticks, seed, resolved=False):
    command = [str(simctl), "run", str(backend), "-", str(ticks), str(seed)]
    if resolved:
        command.append("--resolved")
    result = subprocess.run(command, input=payload, capture_output=True)
    line = json.loads(result.stdout)
    return result.returncode, line, result.stderr

with tempfile.TemporaryDirectory(prefix="sim-harness-test-") as temporary:
    directory = Path(temporary)
    fixture = {"version": 1, "gravity": [0, 0, 0], "bodies": [{"id": 1, "motion": 1, "mass": 2, "inertia": [1, 1, 1], "pose": {"position": [0, 0, 2]}, "shapes": [{"kind": 0, "size": [.1, .2, .3]}]}]}
    payload = wire.encode(fixture)
    seeds = [1, 2, 3, 4]
    summaries = []
    for workers in [1, 2, 4]:
        with ThreadPoolExecutor(max_workers=workers) as pool:
            outcomes = list(pool.map(lambda seed: run(payload, 12, seed), seeds))
        assert all(code == 0 and line["status"] == "passed" and line["steps"] == 12 for code, line, _ in outcomes)
        summaries.append([(line["hash"], line["steps"]) for _, line, _ in outcomes])
    assert summaries[0] == summaries[1] == summaries[2], "worker counts changed a trajectory"
    fixture["assertions"] = {"max_position_norm": .01}
    payload = wire.encode(fixture)
    code, line, stderr = run(payload, 12, 1)
    assert code and b"position bound exceeded" in stderr, "seeded run did not hit the position bound"
    case = directory / "failure"
    case.mkdir()
    manifest = {"mode": "physics", "source_scenario": "-", "scene": base64.b64encode(payload).decode("ascii"),
                "seed": 1, "ticks": 12, "backend": str(backend)}
    failure = case / "failure.json"
    failure.write_text(json.dumps(manifest) + "\n")
    reduced_run = subprocess.run([sys.executable, str(root / "sim/reduce.py"), str(failure), "--simctl", str(simctl)], capture_output=True, text=True)
    assert reduced_run.returncode == 0, reduced_run.stderr
    reduced = json.loads((case / "reduced-replay.json").read_text())
    assert reduced["ticks"] == 1, "reducer did not find the first failing tick"
    assert "position bound exceeded" in reduced["condition"]
    replay = subprocess.run(reduced["command"], input=base64.b64decode(reduced["scene"]), capture_output=True)
    assert replay.returncode and b"position bound exceeded" in replay.stderr, "reduced case did not reproduce"
print("harness and reducer passed")
