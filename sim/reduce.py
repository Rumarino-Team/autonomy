#!/usr/bin/env python3
"""Reduce a saved physics failure while preserving the first failing condition."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
from fuzz import mix64, MASK

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("failure", type=Path, help="failure.json written by fuzz.py")
    parser.add_argument("--simctl", type=Path, default=Path("build/portable/sim/simctl"))
    parser.add_argument("--attempts", type=int, default=64)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    manifest = json.loads(args.failure.read_text())
    if manifest["mode"] != "physics":
        parser.error("mission failures replay with the saved seed; this reducer targets physics inputs")
    directory = args.failure.resolve().parent
    with open(directory / "trace.jsonl") as source:
        header = json.loads(next(source))
        items = [json.loads(line) for line in source]
        samples = [item for item in items if item.get("t") == "sample"]
    scenario = header["scenario"]
    seed = header.get("seed", 0)
    ticks = min(manifest["ticks"], max(item["step"] for item in samples) + 1)
    count = len(scenario.get("thrusters", []))
    def command(tick):
        return [(mix64((seed + (1000+i)*0x94D049BB133111EB + (tick//72)*0xD1B54A32D192ED03) & MASK) >> 11) * 2**-53 * .4 - .2 for i in range(count)]
    commands = [command(tick) if header.get("generated_commands", False) else [0.0]*count for tick in range(ticks)]
    for item in samples:
        if item["step"]:
            commands[item["step"]-1] = item["input_commands"]
    scenario["command_sequence"] = commands
    attempts = 0
    signature = None
    with tempfile.TemporaryDirectory(prefix="autonomy-reduce-") as temporary:
        candidate = Path(temporary) / "candidate.json"
        trace = Path(temporary) / "candidate.jsonl"
        def reproduces(document, length):
            nonlocal attempts, signature
            if attempts >= args.attempts:
                return False
            attempts += 1
            candidate.write_text(json.dumps(document))
            try:
                run = subprocess.run([str(args.simctl.resolve()), "run", manifest["backend"], str(candidate), str(trace), str(length), str(seed), "--resolved"], capture_output=True, text=True, timeout=args.timeout)
            except subprocess.TimeoutExpired:
                return False
            reason = next((line for line in reversed(run.stderr.splitlines()) if line.startswith("simctl:")), "")
            if not run.returncode or not reason:
                return False
            if signature is None:
                signature = reason
            return reason == signature
        if not reproduces(scenario, ticks):
            raise SystemExit("Original failure did not reproduce; preserved its artifacts unchanged.")
        lo, hi = 1, ticks
        while lo < hi and attempts < args.attempts:
            middle = (lo + hi) // 2
            if reproduces(scenario, middle):
                hi = middle
            else:
                lo = middle + 1
        ticks = hi
        scenario["command_sequence"] = scenario["command_sequence"][:ticks]
        index = 0
        while index < len(scenario.get("faults", [])) and attempts < args.attempts:
            candidate_document = json.loads(json.dumps(scenario))
            del candidate_document["faults"][index]
            if reproduces(candidate_document, ticks):
                scenario = candidate_document
            else:
                index += 1
        chunk = max(1, ticks // 2)
        while chunk and attempts < args.attempts:
            for start in range(0, ticks, chunk):
                candidate_document = json.loads(json.dumps(scenario))
                candidate_document["command_sequence"][start:start+chunk] = [[0.0]*count for _ in range(min(chunk, ticks-start))]
                if reproduces(candidate_document, ticks):
                    scenario = candidate_document
                if attempts >= args.attempts:
                    break
            chunk //= 2
    output = directory / "reduced.json"
    output.write_text(json.dumps(scenario, separators=(",", ":")) + "\n")
    replay = [str(args.simctl.resolve()), "run", manifest["backend"], str(output), str(directory / "reduced.trace.jsonl"), str(ticks)]
    (directory / "reduced-replay.json").write_text(json.dumps({"ticks": ticks, "attempts": attempts, "condition": signature, "command": replay}, indent=2) + "\n")
    print(json.dumps({"scenario": str(output), "ticks": ticks, "attempts": attempts}))

if __name__ == "__main__":
    main()
