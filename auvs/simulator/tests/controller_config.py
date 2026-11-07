"""Check generated calibration and independently verify wrench allocation."""
import json
import os
from pathlib import Path
import subprocess
import sys

program, path = sys.argv[1:3]
env = os.environ.copy()
if len(sys.argv) > 3:
    env["LD_PRELOAD"] = sys.argv[3] + (":" + env["LD_PRELOAD"] if env.get("LD_PRELOAD") else "")
generated = json.loads(subprocess.check_output([program], text=True, env=env))
assert generated == json.loads(Path(path).read_text()), "regenerate Hydrus config with simulator_controller_config"
assert len(generated["tam"]) == 8
# Hydrus nominal geometry/signs. Mount coordinates are checked separately by the
# preserved full source-model fingerprint; this test catches force/moment signs.
rho, diameter, thrust, torque, rpm = 1025, .18, .48, .05, 1000
positions = [( .198,.407,0),(-.198,.407,0),(.198,-.408,0),(-.198,-.408,0),(.211,.169,0),(-.211,.169,0),(.211,-.169,0),(-.211,-.169,0)]
directions = [(-.382683,.92388,0),(.382683,.92388,0),(-.382683,-.92388,0),(.382683,-.92388,0),(0,0,-1),(0,0,-1),(0,0,-1),(0,0,-1)]
com = (-.00011753,-.00019489,-.00638696)
def cross(a,b): return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
maximum = rho*diameter**4*thrust*(rpm/60)**2
columns = []
for p,d in zip(positions,directions):
    f = [x*maximum for x in d]; moment = cross([p[i]-com[i] for i in range(3)],f)
    columns.append(f+[moment[i]-f[i]*diameter*torque/thrust for i in range(3)])
for axis in range(6):
    wrench = [sum(columns[m][i]*generated["tam"][m][axis] for m in range(8)) for i in range(6)]
    assert all(abs(v-(1 if i==axis else 0)) < .002 for i,v in enumerate(wrench)), (axis,wrench)
assert generated["camera_forward"][1] < -.99
print("Hydrus calibration matches current presets and produces six unit wrench axes.")
