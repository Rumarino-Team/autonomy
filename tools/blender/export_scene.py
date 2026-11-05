"""Blender entry point, shared exporter for saved-file command-line runs."""
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
destination=parser.add_mutually_exclusive_group(required=True)
destination.add_argument('--output',type=Path)
destination.add_argument('--pipe-fd',type=int)
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
scene=bpy.data.scenes.get(args.scene)
if scene is None: raise ValueError(f'authoring scene not found: {args.scene}')
if bpy.context.window: bpy.context.window.scene=scene
snapshot=exporter.export_scene(scene,args.profiles,args.frame,args.output)
if args.pipe_fd is not None:
    with os.fdopen(args.pipe_fd,'wb') as output:
        output.write(wire.encode(snapshot));output.flush()
else:
    print('BLENDER_SCENE_EXPORTED',snapshot['source']['content_hash'],len(snapshot['bodies']),args.output)
