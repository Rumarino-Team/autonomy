#!/usr/bin/env python3
"""Backend-independent seeded physics/mission testing and deterministic replay."""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys

MASK = (1 << 64) - 1

def mix64(x):
    x = (x + 0x9E3779B97F4A7C15) & MASK
    x = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & MASK
    x = ((x ^ (x >> 27)) * 0x94D049BB133111EB) & MASK
    return x ^ (x >> 31)

def file_hash(path):
    h = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def trace_summary(path):
    hashes = hashlib.sha256()
    header = None
    last = None
    with open(path) as source:
        for line in source:
            item = json.loads(line)
            if item["t"] == "header":
                header = item
            elif item["t"] == "sample":
                hashes.update(str(item["hash"]).encode() + b"\n")
                last = item
    if header is None or last is None:
        raise ValueError("trace missing header or samples")
    return {"trajectory": hashes.hexdigest(), "last_tick": last["step"], "final_hash": last["hash"], "backend_version": header["backend_version"]}

def execute(command, environment, timeout, directory):
    try:
        result = subprocess.run(command, env=environment, cwd=directory, capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout, result.stderr
    except OSError as error:
        return 127, "", f"infrastructure execution failed: {error}"
    except subprocess.TimeoutExpired as error:
        def text(value):
            return value.decode(errors="replace") if isinstance(value, bytes) else value or ""
        return 124, text(error.stdout), "infrastructure watchdog expired\n" + text(error.stderr)

def run_case(args, index):
    seed = args.replay_seed if args.replay_seed is not None else mix64((args.seed + index * 0x9E3779B97F4A7C15) & MASK)
    identity = f"case-{index:06d}-{seed}"
    with tempfile.TemporaryDirectory(prefix="autonomy-fuzz-") as temporary:
        directory = Path(temporary)
        trace = directory / "trace.jsonl"
        environment = os.environ.copy()
        # Freeze every simulation option affecting the result, irrespective of the caller's shell.
        for key in list(environment):
            if key.startswith("AUTONOMY_SIM_"):
                del environment[key]
        if args.mode == "physics":
            command = [str(args.simctl), "run", str(args.backend), str(args.scenario), str(trace), str(args.ticks), str(seed)]
        else:
            environment.update(AUTONOMY_SIM_BACKEND=str(args.backend), AUTONOMY_SIM_SCENARIO=str(args.scenario), AUTONOMY_SIM_SEED=str(seed), AUTONOMY_SIM_MAX_TICKS=str(args.ticks), AUTONOMY_SIM_TRACE=str(trace))
            command = [str(args.mission_executable), str(args.plugin), args.mission, str(args.controller)]
        code, stdout, stderr = execute(command, environment, args.timeout, directory)
        summary = {"index": index, "seed": seed, "status": "passed" if code == 0 else "failed", "exit_code": code}
        try:
            summary.update(trace_summary(trace))
        except (OSError, ValueError, KeyError) as error:
            code = code or 1
            stderr += f"\ntrace error: {error}"
            summary.update(status="failed", exit_code=code)
        if not code and args.verify_replay:
            if args.mode == "physics":
                replay_command = [str(args.simctl), "replay", str(args.backend), str(trace), str(directory / "replay.jsonl")]
                replay_env = environment
            else:
                replay_command = command
                replay_env = environment.copy()
                replay_env["AUTONOMY_SIM_TRACE"] = str(directory / "replay.jsonl")
            replay_code, replay_stdout, replay_stderr = execute(replay_command, replay_env, args.timeout, directory)
            try:
                replay_matches = not replay_code and trace_summary(directory / "replay.jsonl")["trajectory"] == summary["trajectory"]
            except (OSError, ValueError, KeyError):
                replay_matches = False
            if not replay_matches:
                summary.update(status="failed", exit_code=replay_code or 1)
                stderr += "\nreplay diverged\n" + replay_stdout + replay_stderr
        if summary["status"] == "failed" or args.keep_passed:
            destination = args.output / identity
            destination.mkdir(parents=True, exist_ok=True)
            for source in directory.iterdir():
                if source.is_file():
                    shutil.copy2(source, destination / source.name)
            source_name = 'source.asim' if args.scenario.suffix=='.asim' else 'source.json'
            shutil.copy2(args.scenario, destination / source_name)
            (destination / "stdout.log").write_text(stdout)
            (destination / "stderr.log").write_text(stderr)
            manifest = {**summary, "mode": args.mode, "backend": str(args.backend), "backend_sha256": file_hash(args.backend), "scenario": str(args.scenario), "scenario_sha256": file_hash(args.scenario), "ticks": args.ticks, "command": command, "environment": {key: value for key, value in environment.items() if key.startswith("AUTONOMY_SIM_")}}
            manifest["platform"] = list(os.uname())
            manifest["source_scenario"] = str(destination / source_name)
            manifest["authored_source"] = getattr(args,'authored_source',str(args.scenario))
            manifest["command"] = [part.replace(str(directory),str(destination)).replace(str(args.scenario),str(destination / source_name)) for part in command]
            manifest["environment"] = {key: value.replace(str(directory),str(destination)).replace(str(args.scenario),str(destination / source_name)) for key,value in manifest["environment"].items()}
            manifest["replay_command"] = [str(args.simctl), "replay", str(args.backend), str(destination / "trace.jsonl"), str(destination / "replay.jsonl")]
            (destination / "failure.json").write_text(json.dumps(manifest, indent=2) + "\n")
            if args.reduce_failures and summary["status"] == "failed" and args.mode == "physics" and trace.exists():
                reduction = subprocess.run([sys.executable, str(Path(__file__).with_name("reduce.py")), str(destination / "failure.json"), "--simctl", str(args.simctl), "--timeout", str(args.timeout)], capture_output=True, text=True)
                (destination / "reduction.log").write_text(reduction.stdout + reduction.stderr)
        return summary

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", type=Path, required=True)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--simctl", type=Path, default=Path("build/portable/sim/simctl"))
    parser.add_argument("--mode", choices=["physics", "mission"], default="physics")
    parser.add_argument("--mission-executable", type=Path, default=Path("zig-out/bin/src_3"))
    parser.add_argument("--plugin", type=Path, default=Path("build/portable/sim/libauv_sim.so"))
    parser.add_argument("--controller", type=Path, default=Path("sim/scenarios/controller.json"))
    parser.add_argument("--mission", choices=["prequalify_robosub", "prequalify_coco"], default="prequalify_robosub")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--replay-seed", type=int)
    parser.add_argument("--cases", type=int, default=32)
    parser.add_argument("--workers", type=int, default=min(32, os.cpu_count() or 1))
    parser.add_argument("--ticks", type=int, default=21600)
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--output", type=Path, default=Path("build/fuzz"))
    parser.add_argument("--verify-replay", action="store_true")
    parser.add_argument("--keep-passed", action="store_true")
    parser.add_argument("--reduce-failures", action="store_true")
    args = parser.parse_args()
    if args.cases <= 0 or args.workers <= 0 or args.ticks <= 0 or args.timeout <= 0 or not 0 <= args.seed <= MASK or (args.replay_seed is not None and not 0 <= args.replay_seed <= MASK):
        parser.error("positive case/worker/tick/timeout values and uint64 seeds required")
    for name in ["backend", "scenario", "simctl", "mission_executable", "plugin", "controller", "output"]:
        setattr(args, name, getattr(args, name).resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    count = 1 if args.replay_seed is not None else args.cases
    with tempfile.TemporaryDirectory(prefix='autonomy-frozen-source-') as temporary:
        if args.scenario.suffix=='.blend':
            args.authored_source=str(args.scenario)
            frozen=Path(temporary)/'scene.asim'
            subprocess.run([str(args.simctl),'snapshot',str(args.scenario),str(frozen)],check=True,timeout=args.timeout)
            args.scenario=frozen
        with concurrent.futures.ThreadPoolExecutor(max_workers=min(count, args.workers)) as executor:
            outcomes = list(executor.map(lambda i: run_case(args, i), range(count)))
    outcomes.sort(key=lambda value: value["index"])
    (args.output / "summary.json").write_text(json.dumps(outcomes, indent=2) + "\n")
    failed = sum(case["status"] != "passed" for case in outcomes)
    print(json.dumps({"cases": count, "failed": failed, "summary": str(args.output / "summary.json")}))
    return int(failed != 0)

if __name__ == "__main__":
    raise SystemExit(main())
