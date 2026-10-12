#!/usr/bin/env python3
"""Restore the engine changes required by the experimental simulator branch."""

import argparse
import json
from pathlib import Path
import subprocess


def git(repo, *args, check=True):
    return subprocess.run(["git", "-C", str(repo), *args], check=check,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def prepare(name):
    root = Path(__file__).resolve().parents[1]
    source = json.loads((root / "vendor_patches/sources.json").read_text())[name]
    repo = root / "vendor" / name
    if not (repo / ".git").exists():
        if name == "stonefish":
            subprocess.run(["git", "-C", str(root), "submodule", "update", "--init",
                            "--", "vendor/stonefish"], check=True)
        else:
            # RTX is an optional fork, outside the upstream submodule.
            if repo.exists() and any(repo.iterdir()):
                raise RuntimeError(f"{repo} already contains files; cannot clone over them")
            subprocess.run(["git", "clone", source["url"], str(repo)], check=True)
            git(repo, "checkout", "--detach", source["commit"])
    patch = root / "vendor_patches" / f"{name}.patch"
    if git(repo, "apply", "--reverse", "--check", str(patch), check=False).returncode == 0:
        print(f"{name}: experimental engine changes already present")
        return
    if git(repo, "rev-parse", "HEAD").stdout.strip() != source["commit"]:
        raise RuntimeError(f"{repo} must use {source['commit']} before applying its patch")
    result = git(repo, "apply", "--check", str(patch), check=False)
    if result.returncode:
        raise RuntimeError(f"Cannot apply {patch.name}; existing edits were preserved:\n{result.stderr}")
    git(repo, "apply", str(patch))
    print(f"{name}: experimental engine changes restored")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("engine", choices=("stonefish", "stonefish_rtx"), default="stonefish", nargs="?")
    prepare(parser.parse_args().engine)
