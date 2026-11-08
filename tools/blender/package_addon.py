#!/usr/bin/env python3
"""Create the Blender Install from Disk ZIP without modifying user preferences."""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

repo = Path(__file__).resolve().parents[2]
output = repo / 'build/portable/autonomy_sim.zip'
output.parent.mkdir(parents=True, exist_ok=True)
with ZipFile(output, 'w', compression=ZIP_DEFLATED) as archive:
    for path in sorted((Path(__file__).parent/'autonomy_sim').glob('*.py')):
        if not path.name.startswith('test_'):
            archive.write(path, 'autonomy_sim/'+path.name)
    archive.write(repo/'tools/data_binary.py', 'autonomy_sim/data_binary.py')
    archive.writestr('autonomy_sim/repo_path.txt', str(repo.resolve()) + '\n')
print(output)
