"""Compare compiled-Zig controller literals against the native calibration tool."""
import os
from pathlib import Path
import re
import subprocess
import sys
program,path=sys.argv[1:3]
env=os.environ.copy()
if len(sys.argv)>3:env['LD_PRELOAD']=sys.argv[3]+(':'+env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
generated=subprocess.check_output([program],text=True,env=env)
saved=Path(path).read_text()
number=r'[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?'
def values(source,field,next_field):
    section=source.split('.'+field+'=',1)[1].split('.'+next_field+'=',1)[0] if '.'+field+'=' in source else source.split('.'+field+' =',1)[1].split('.'+next_field+' =',1)[0]
    return [float(x) for x in re.findall(number,section)]
g=values(generated,'tam','thruster_output_scale');s=values(saved,'tam','thruster_output_scale')
assert len(g)==len(s)==48
assert all(abs(a-b)<=max(1e-15,abs(a)*1e-12) for a,b in zip(g,s)),(g,s)
rho,diameter,thrust,torque,rpm=1025,.18,.48,.05,1000
positions=[(.198,.407,0),(-.198,.407,0),(.198,-.408,0),(-.198,-.408,0),(.211,.169,0),(-.211,.169,0),(.211,-.169,0),(-.211,-.169,0)]
directions=[(-.382683,.92388,0),(.382683,.92388,0),(-.382683,-.92388,0),(.382683,-.92388,0),(0,0,-1),(0,0,-1),(0,0,-1),(0,0,-1)]
com=(-.00011753,-.00019489,-.00638696)
def cross(a,b):return(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
maximum=rho*diameter**4*thrust*(rpm/60)**2
columns=[]
for p,d in zip(positions,directions):
 f=[x*maximum for x in d];m=cross([p[i]-com[i] for i in range(3)],f);columns.append(f+[m[i]-f[i]*diameter*torque/thrust for i in range(3)])
for axis in range(6):
 wrench=[sum(columns[m][i]*s[m*6+axis] for m in range(8)) for i in range(6)]
 assert all(abs(v-(1 if i==axis else 0))<.002 for i,v in enumerate(wrench)),(axis,wrench)
print('Zig Hydrus calibration unchanged; six analytic wrench axes verified.')
