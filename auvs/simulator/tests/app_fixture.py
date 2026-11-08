"""Isolated Zig sources/configuration for integration tests; never edit user files."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from data_binary import read, decode, encode

class Application:
    def __init__(self, directory, plugin, backend, preload=None, **settings):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        shutil.copytree(ROOT/'src', self.root/'src')
        shutil.copytree(ROOT/'config', self.root/'config')
        shutil.copytree(ROOT/'docs', self.root/'docs')
        for name in ('application.zig','metadata.zig','fuzz_startup.zig','build.zig','build.zig.zon'):
            shutil.copy2(ROOT/name, self.root/name)
        shutil.copytree(ROOT/'include',self.root/'include')
        for name in ('auvs','tools','vendor','arduino'):
            (self.root/name).symlink_to(ROOT/name, target_is_directory=True)
        self.settings = dict(plugin=str(Path(plugin).resolve()), backend=str(Path(backend).resolve()), verbose=False, telemetry=False,
                             reload='.{ .watch = false, .preserve_state = true }')
        self.settings.update(settings)
        self.preload = preload
        self.configure()
        self.build()
    def configure(self, **settings):
        self.settings.update(settings)
        def literal(v):
            if isinstance(v, bool): return str(v).lower()
            if isinstance(v, (int,float)): return repr(v)
            if v.startswith(('.', '@')): return v
            return '"'+v.replace('\\','\\\\').replace('"','\\"')+'"'
        text = 'const Types = @import("../src/Configuration.zig");\npub const config: Types.Application = .{\n'
        if 'controller' not in self.settings: text += '.controller=@import("hydrus.zig").controller,\n'
        text += ''.join('.'+k+'='+literal(v)+',\n' for k,v in self.settings.items())+'};\n'
        (self.root/'config/application.zig').write_text(text)
    def build(self, step=None):
        args = ['zig','build'] + (['-Dexternal-asan=true'] if self.preload else []) + ([step] if step else []) + ['--prefix',str(self.root/'package')]
        p = subprocess.run(args,cwd=self.root,capture_output=True,text=True,timeout=120)
        assert p.returncode == 0, p.stderr[-10000:]
    @property
    def exe(self): return self.root/'package/bin/src_3'
    @property
    def env(self):
        env = os.environ.copy()
        if self.preload: env['LD_PRELOAD'] = self.preload + (':'+env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
        return env
    def run(self, *args, **kwargs):
        return subprocess.run([str(self.exe),*args],cwd=self.root,env=self.env,capture_output=True,text=True,timeout=600 if self.preload else 180,**kwargs)

def outcome(result):
    lines = [line for line in result.stdout.splitlines() if line.startswith('simulation ')]
    assert len(lines)==1, result.stdout[-3000:]+result.stderr[-3000:]
    values = dict(field.split('=') for field in lines[0].split()[1:])
    return {k:float(v) if 'depth' in k else int(v) for k,v in values.items()}
