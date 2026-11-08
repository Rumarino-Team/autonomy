"""Golden interchange and hostile-length/checksum checks for Python and Zig."""
from pathlib import Path
import struct
import sys
import zlib
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from data_binary import decode,encode
p=Path(__file__).with_name('binary-golden.bin')
value=dict(x=[1.,2.,3.],optional=9,name='test')
assert decode(p.read_bytes(),kind='metadata')==value
assert encode(value,'metadata')==p.read_bytes()
for b in (b'{}',p.read_bytes()[:-1],p.read_bytes()+b'x',p.read_bytes()[:27],p.read_bytes()[:-1]+b'X'):
    try:decode(b)
    except ValueError:pass
    else:raise AssertionError('accepted malformed binary')
for payload in (b'\xff',b'\x06'+struct.pack('<I',0xffffffff),b'\x07'+struct.pack('<I',0xffffffff)):
    b=b'AUVDATA1'+struct.pack('<IIQI',7,1,len(payload),zlib.crc32(payload))+payload
    try:decode(b)
    except ValueError:pass
    else:raise AssertionError('accepted malformed payload')
print('Python binary golden, limits, versions, checksums and truncation passed.')
