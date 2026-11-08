"""Blender background entry point for the owned AUVDATA1 scene interchange."""
import argparse
from pathlib import Path
import sys
import bpy

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from data_binary import write
from autonomy_sim.exporter import export_snapshot

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--scene')
parser.add_argument('--frame', type=int)
args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
scene = bpy.data.scenes[args.scene] if args.scene else bpy.context.scene
if scene.get('sim_preview'):
    source = bpy.data.scenes.get(scene.get('sim_source_scene', ''))
    if source is None: raise ValueError('saved preview has no authoring scene')
    scene = source
write(args.output, export_snapshot(scene, args.frame), 'scene')
