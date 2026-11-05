"""Versioned binary scene transport. Standard MessagePack; no third-party modules."""
import math
import struct

MAGIC = b'ASIMSC01'
HEADER = struct.Struct('<8sQ')
MAX_BYTES = 128 * 1024 * 1024


def pack(value):
    output = bytearray()

    def emit(v, depth=0):
        if depth > 64:
            raise ValueError('scene nesting limit exceeded')
        if v is None: output.append(0xc0)
        elif isinstance(v, bool): output.append(0xc3 if v else 0xc2)
        elif isinstance(v, int):
            output.extend(struct.pack('>BQ' if v >= 0 else '>Bq', 0xcf if v >= 0 else 0xd3, v))
        elif isinstance(v, float):
            if not math.isfinite(v): raise ValueError('non-finite scene value')
            output.extend(struct.pack('>Bd', 0xcb, v))
        elif isinstance(v, str):
            data = v.encode('utf-8')
            output.extend(struct.pack('>BI', 0xdb, len(data))); output.extend(data)
        elif isinstance(v, (list, tuple)):
            output.extend(struct.pack('>BI', 0xdd, len(v)))
            for item in v: emit(item, depth+1)
        elif isinstance(v, dict):
            output.extend(struct.pack('>BI', 0xdf, len(v)))
            for key in sorted(v):
                if not isinstance(key, str): raise ValueError('scene keys must be strings')
                emit(key, depth+1); emit(v[key], depth+1)
        else: raise ValueError('unsupported scene value: '+type(v).__name__)
        if len(output) > MAX_BYTES: raise ValueError('scene exceeds binary size limit')

    emit(value)
    return bytes(output)


def unpack(data):
    """Decode the JSON-compatible subset of MessagePack used by scene/profile data."""
    if not data or len(data) > MAX_BYTES: raise ValueError('invalid binary payload size')
    offset = 0

    def take(n):
        nonlocal offset
        if n < 0 or offset+n > len(data): raise ValueError('truncated binary payload')
        value = data[offset:offset+n]; offset += n
        return value

    def number(fmt): return struct.unpack('>'+fmt, take(struct.calcsize('>'+fmt)))[0]

    def read(depth=0):
        if depth > 64: raise ValueError('scene nesting limit exceeded')
        tag = take(1)[0]
        if tag <= 0x7f: return tag
        if tag >= 0xe0: return tag-256
        if tag in (0xc0, 0xc2, 0xc3): return {0xc0:None, 0xc2:False, 0xc3:True}[tag]
        formats = {0xcc:'B', 0xcd:'H', 0xce:'I', 0xcf:'Q', 0xd0:'b', 0xd1:'h', 0xd2:'i', 0xd3:'q', 0xca:'f', 0xcb:'d'}
        if tag in formats:
            value = number(formats[tag])
            if isinstance(value, float) and not math.isfinite(value): raise ValueError('non-finite binary value')
            return value
        if 0xa0 <= tag <= 0xbf or tag in (0xd9, 0xda, 0xdb):
            count = tag & 31 if tag <= 0xbf else number({0xd9:'B', 0xda:'H', 0xdb:'I'}[tag])
            return take(count).decode('utf-8')
        if 0x90 <= tag <= 0x9f or tag in (0xdc, 0xdd):
            count = tag & 15 if tag <= 0x9f else number('H' if tag == 0xdc else 'I')
            if count > len(data)-offset: raise ValueError('invalid array size')
            return [read(depth+1) for _ in range(count)]
        if 0x80 <= tag <= 0x8f or tag in (0xde, 0xdf):
            count = tag & 15 if tag <= 0x8f else number('H' if tag == 0xde else 'I')
            if count > (len(data)-offset)//2: raise ValueError('invalid map size')
            result = {}
            for _ in range(count):
                key = read(depth+1)
                if not isinstance(key, str) or key in result: raise ValueError('invalid or duplicate map key')
                result[key] = read(depth+1)
            return result
        raise ValueError('unsupported MessagePack value')

    value = read()
    if offset != len(data): raise ValueError('trailing binary payload')
    return value


def encode(scene):
    data = pack(scene)
    return HEADER.pack(MAGIC, len(data))+data


def decode(data):
    if len(data) < HEADER.size: raise ValueError('truncated binary scene header')
    magic, size = HEADER.unpack_from(data)
    if magic != MAGIC or not 0 < size <= MAX_BYTES or len(data) != HEADER.size+size:
        raise ValueError('invalid binary scene header/size')
    return unpack(data[HEADER.size:])
