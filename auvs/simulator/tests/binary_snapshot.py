"""Python -> C++ snapshot agreement, including mesh data and invalid references."""
from pathlib import Path
import subprocess
import sys
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from data_binary import write
identity=dict(position=[0.,0.,0.],orientation=[0.,0.,0.,1.])
scene=dict(version=1,vehicle='hydrus',bodies=[
 dict(id=1,name='hydrus',profile='hydrus',**{'class':0},motion=1,tracked=False,fuzz=False,pose=identity,shapes=[]),
 dict(id=3,name='marker',**{'class':2},motion=0,tracked=True,fuzz=True,pose=dict(position=[1.,2.,3.],orientation=[0.,0.,0.,1.]),shapes=[
 dict(kind=3,collisions=True,pose=identity,vertices=[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],triangles=[[0,1,2],[0,3,1],[0,2,3],[1,3,2]])])])
with tempfile.TemporaryDirectory() as tmp:
 path=Path(tmp)/'scene.bin';write(path,scene,'scene')
 p=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
 assert p.returncode==0,p.stderr
 scene['bodies'][1]['shapes'][0]['triangles'][0][0]=-1
 write(path,scene,'scene')
 p=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
 assert p.returncode!=0 and 'triangle' in p.stderr,p.stderr
 scene['bodies'][1]['shapes'][0]['triangles'][0][0]=0
 scene['bodies'][1]['tracked']=float('nan')
 write(path,scene,'scene')
 p=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
 assert p.returncode!=0 and 'non-finite' in p.stderr,p.stderr
 path.write_bytes(b'{}')
 p=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
 assert p.returncode!=0,p.stderr
print('Python/C++ mesh snapshot agreement and malformed-reference rejection passed.')
