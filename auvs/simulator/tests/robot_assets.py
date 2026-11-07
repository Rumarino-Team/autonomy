"""Verify bundled saved Blender assets against the compiled preset provenance."""
import hashlib
import json
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
manifest = json.loads((root / "data/robots/manifest.json").read_text())
assert [r["name"] for r in manifest["robots"]] == ["hydrus", "proteus", "bluerov", "girona500"]
source = (root / "robots.cpp").read_text()
for robot in manifest["robots"]:
    asset = root / "data" / robot["asset"]
    digest = hashlib.sha256(asset.read_bytes()).digest()
    assert digest.hex() == robot["asset_sha256"], f"changed saved asset: {asset}"
    pattern = rf"constinit const Robot {robot['name']}\{{.*?\.asset_hash\s*=\s*\{{(.*?)\}}"
    match = re.search(pattern, source, re.S)
    assert match, f"missing compiled definition: {robot['name']}"
    compiled_digest = bytes(int(h, 16) for h in re.findall(r"0x([0-9a-fA-F]{2})", match[1]))
    assert compiled_digest == digest, f"compiled asset hash mismatch: {robot['name']}"
print("All four saved Blender assets match their compiled SHA-256 metadata.")
