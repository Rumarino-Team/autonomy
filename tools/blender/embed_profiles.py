"""Embed calibration in the current .blend; save only with explicit --save."""
import argparse
from pathlib import Path
import sys
import bpy

sys.path.insert(0,str(Path(__file__).parent))
from autonomy_sim import exporter

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scene',default='AutonomySource')
parser.add_argument('--profiles',type=Path,default=Path(__file__).resolve().parents[2]/'sim/profiles')
parser.add_argument('--save',action='store_true')
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
scene=bpy.data.scenes.get(args.scene)
if scene is None: raise ValueError('missing authoring scene '+args.scene)
names=exporter.embed_profiles(scene,args.profiles)
exporter.export_scene(scene,Path('/unavailable-profile-sidecars'))
if args.save:
    if not bpy.data.filepath: raise ValueError('save the .blend file first')
    bpy.ops.wm.save_as_mainfile(filepath=bpy.data.filepath)
print('EMBEDDED_CALIBRATION',names)
