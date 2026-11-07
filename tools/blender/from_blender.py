#!/usr/bin/env python3
"""Launch a mission directly from a .blend file."""
import argparse
import os
from pathlib import Path
import subprocess

repo=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('blend',type=Path)
parser.add_argument('--scene',default='AutonomySource')
parser.add_argument('--frame',type=int,default=1)
parser.add_argument('--profiles',type=Path,default=repo/'sim/profiles')
parser.add_argument('--backend',choices=['mujoco','stonefish'],default='mujoco')
parser.add_argument('--ticks',type=int,default=21600)
parser.add_argument('--seed',type=int)
parser.add_argument('--controller',type=Path,default=repo/'sim/scenarios/controller.json')
parser.add_argument('--realtime',action='store_true')
parser.add_argument('--blender-link',action='store_true')
parser.add_argument('--port',type=int,default=9760)
args=parser.parse_args()
if not args.blend.is_file() or args.frame<0 or args.ticks<=0 or not 0<args.port<65536:
    parser.error('An existing saved .blend and valid frame/tick/port values are required')
env={k:v for k,v in os.environ.items() if not k.startswith('AUTONOMY_SIM_')}
env.update(AUTONOMY_SIM_SCENARIO=str(args.blend.resolve()),AUTONOMY_SIM_BACKEND=str(repo/f'build/portable/sim/libsim_backend_{args.backend}.so'),
           AUTONOMY_SIM_SOURCE_SCENE=args.scene,AUTONOMY_SIM_SOURCE_FRAME=str(args.frame),
           AUTONOMY_SIM_PROFILES=str(args.profiles.resolve()),
           AUTONOMY_SIM_MAX_TICKS=str(args.ticks),
           AUTONOMY_SIM_REALTIME=str(int(args.realtime)),AUTONOMY_SIM_BLENDER=str(int(args.blender_link)),
           AUTONOMY_SIM_BLENDER_PORT=str(args.port))
if args.seed is not None: env['AUTONOMY_SIM_SEED']=str(args.seed)
raise SystemExit(subprocess.run([str(repo/'zig-out/bin/src_3'),str(repo/'build/portable/sim/libauv_sim.so'),
                                'prequalify_robosub',str(args.controller.resolve())],env=env,cwd=repo).returncode)
