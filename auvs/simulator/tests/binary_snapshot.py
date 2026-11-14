"""Python -> Zig snapshot agreement, including mesh data and invalid references."""
from pathlib import Path
import subprocess
import sys
import tempfile
import copy
import struct
import zlib
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
 # Optional fields and record order are interchange details, not model topology.
 reordered=copy.deepcopy(scene)
 reordered['bodies'][1]['tracked']=True
 reordered=dict(reversed(list(reordered.items())))
 reordered['ignored_extension']={'future':True}
 write(path,reordered,'scene')
 p=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
 assert p.returncode==0,p.stderr
 def reject(mutator, message):
  invalid=copy.deepcopy(reordered);mutator(invalid);write(path,invalid,'scene')
  result=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
  assert result.returncode!=0,(message,result.stdout,result.stderr)
 reject(lambda s:s.update(version=2),'unsupported scene version')
 reject(lambda s:s.update(bodies=[]),'empty scene')
 reject(lambda s:s['bodies'][1].update(id=1),'reserved vehicle ID')
 reject(lambda s:s['bodies'].append(copy.deepcopy(s['bodies'][0])),'duplicate vehicle')
 reject(lambda s:s['bodies'][1].update(motion=1),'dynamic scenery')
 reject(lambda s:s['bodies'][1].update(shapes=[]),'empty scenery')
 reject(lambda s:s['bodies'][1]['pose'].update(orientation=[0.,0.,0.,0.]),'invalid quaternion')
 reject(lambda s:s['bodies'][1]['shapes'][0]['triangles'][0].__setitem__(0,0.5),'fractional index')
 reject(lambda s:s['bodies'][1].update(id=2**32),'oversized ID')
 write(path,reordered,'scene');valid=path.read_bytes()
 for invalid in (valid[:-1],valid+b'x',valid[:-1]+bytes([valid[-1]^1]),valid[:27]):
  path.write_bytes(invalid)
  result=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
  assert result.returncode!=0,'accepted corrupt container'
 for payload in (b'\xff',b'\x06'+struct.pack('<I',0xffffffff)):
  path.write_bytes(b'AUVDATA1'+struct.pack('<IIQI',6,1,len(payload),zlib.crc32(payload))+payload)
  result=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
  assert result.returncode!=0,'accepted invalid dynamic value'
 path.write_bytes(b'{}')
 p=subprocess.run([sys.argv[1],str(path)],capture_output=True,text=True)
 assert p.returncode!=0,p.stderr
print('Python/Zig mesh snapshot agreement and malformed-reference rejection passed.')
