"""Offline evaluated-scene reference helper; unused by native simulation launch."""
import argparse
from pathlib import Path
import sys
import os
import bpy

sys.path.insert(0,str(Path(__file__).parent))
from autonomy_sim import exporter, wire

parser=argparse.ArgumentParser()
parser.add_argument('--scene',required=True)
parser.add_argument('--frame',type=int,required=True)
parser.add_argument('--profiles',type=Path,required=True)
parser.add_argument('--pipe-fd',type=int,required=True)
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
scene=bpy.data.scenes.get(args.scene)
if scene is None: raise ValueError(f'authoring scene not found: {args.scene}')
if bpy.context.window: bpy.context.window.scene=scene
snapshot=exporter.export_scene(scene,args.profiles,args.frame)
with os.fdopen(args.pipe_fd,'wb') as output:
    output.write(wire.encode(snapshot));output.flush()
