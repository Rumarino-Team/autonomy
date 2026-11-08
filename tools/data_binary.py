"""AUVDATA1 field codec shared by Blender tools, tests and the binary inspector.

Header: magic[8], kind:u32, schema:u32, length:u64, CRC32:u32; little endian.
Payload: tagged values, u32 byte/element counts. No native structure padding.
"""
from pathlib import Path
import struct
import zlib

MAX_BYTES = 64 * 1024 * 1024
MAX_ELEMENTS = 1_000_000
KINDS = dict(state=1, campaign=2, case=3, summary=4, totals=5, scene=6, metadata=7, run=8)

def encode(value, kind, schema=1):
    def text(s):
        b = s.encode('utf-8') if isinstance(s, str) else bytes(s)
        if len(b) > MAX_BYTES: raise ValueError('binary string limit')
        return struct.pack('<I', len(b)) + b
    def item(v, depth=0):
        if depth >= 64: raise ValueError('binary nesting limit')
        if v is None: return b'\0'
        if isinstance(v, bool): return b'\1' + bytes([v])
        if isinstance(v, int): return (b'\2' + struct.pack('<q', v)) if v < 0 else (b'\3' + struct.pack('<Q', v))
        if isinstance(v, float): return b'\4' + struct.pack('<d', v)
        if isinstance(v, (str, bytes, bytearray)): return b'\5' + text(v)
        if isinstance(v, (list, tuple)):
            if len(v) > MAX_ELEMENTS: raise ValueError('binary element limit')
            return b'\6' + struct.pack('<I', len(v)) + b''.join(item(x, depth+1) for x in v)
        if isinstance(v, dict):
            if len(v) > MAX_ELEMENTS: raise ValueError('binary element limit')
            return b'\7' + struct.pack('<I', len(v)) + b''.join(text(k) + item(x, depth+1) for k,x in v.items())
        raise TypeError(type(v))
    payload = item(value)
    if len(payload) > MAX_BYTES: raise ValueError('binary payload limit')
    code = KINDS[kind] if isinstance(kind, str) else kind
    return b'AUVDATA1' + struct.pack('<IIQI', code, schema, len(payload), zlib.crc32(payload)) + payload

def decode(data, *, kind=None, schema=1):
    data = bytes(data)
    if len(data) < 28 or data[:8] != b'AUVDATA1': raise ValueError('unsupported format: expected AUVDATA1 binary; JSON is obsolete')
    code, version, size, checksum = struct.unpack_from('<IIQI', data, 8)
    if code not in KINDS.values(): raise ValueError('unsupported binary kind')
    if kind is not None and code != (KINDS[kind] if isinstance(kind,str) else kind): raise ValueError('wrong binary kind')
    if schema is not None and version != schema: raise ValueError('unsupported binary schema')
    if size > MAX_BYTES or size != len(data)-28: raise ValueError('invalid binary length')
    if zlib.crc32(data[28:]) != checksum: raise ValueError('binary checksum')
    at, nodes = 28, 0
    def raw(n):
        nonlocal at
        if n < 0 or n > len(data)-at: raise ValueError('truncated binary')
        b = data[at:at+n]; at += n; return b
    def number(fmt): return struct.unpack(fmt, raw(struct.calcsize(fmt)))[0]
    def text():
        n = number('<I')
        if n > MAX_BYTES: raise ValueError('binary string limit')
        return raw(n).decode('utf-8', errors='surrogateescape')
    def count():
        n = number('<I')
        if n > MAX_ELEMENTS or n > len(data)-at: raise ValueError('binary element limit')
        return n
    def item(depth=0):
        nonlocal nodes
        nodes += 1
        if depth >= 64 or nodes > MAX_ELEMENTS: raise ValueError('binary nesting/element limit')
        tag = number('<B')
        if tag == 0: return None
        if tag == 1:
            b = number('<B')
            if b > 1: raise ValueError('binary boolean')
            return bool(b)
        if tag == 2: return number('<q')
        if tag == 3: return number('<Q')
        if tag == 4: return number('<d')
        if tag in (5,8): return text()
        if tag == 6: return [item(depth+1) for _ in range(count())]
        if tag == 7:
            result = {}
            for _ in range(count()):
                key = text()
                if key in result: raise ValueError('duplicate binary field')
                result[key] = item(depth+1)
            return result
        raise ValueError('unknown binary tag')
    result = item()
    if at != len(data): raise ValueError('trailing binary data')
    return result

def read(path, **kwargs):
    path = Path(path)
    if path.stat().st_size > MAX_BYTES+28: raise ValueError('binary file too large')
    return decode(path.read_bytes(), **kwargs)

def write(path, value, kind, schema=1):
    path = Path(path)
    data = encode(value, kind, schema)
    temporary = path.with_name(path.name+'.partial')
    temporary.write_bytes(data)
    temporary.replace(path)

if __name__ == '__main__':
    import argparse
    import pprint
    parser = argparse.ArgumentParser(description='Inspect an AUVDATA1 binary file as human-readable text.')
    parser.add_argument('path', type=Path)
    args = parser.parse_args()
    pprint.pprint(read(args.path, schema=None), sort_dicts=False)
