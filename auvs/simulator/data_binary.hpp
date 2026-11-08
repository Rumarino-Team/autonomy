#ifndef AUTONOMY_DATA_BINARY_HPP
#define AUTONOMY_DATA_BINARY_HPP
// AUVDATA1 interchange, matching src/BinaryData.zig and tools/data_binary.py.
// Values are encoded by field, never by the C++ object's memory representation.
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autonomy::data {
inline constexpr size_t max_bytes = 64 * 1024 * 1024, max_elements = 1000000;

inline void need(bool ok, const char *message) {
    if (!ok)
        throw std::runtime_error(message);
}

struct Value {
    enum Kind {
        Null,
        Bool,
        Number,
        String,
        Array,
        Object
    };

    Kind kind = Null;
    bool boolean = false;
    long double number = 0;
    uint8_t string_tag = 5;
    uint8_t numeric_tag = 4;
    uint64_t bits = 0;
    std::string text;
    std::vector<Value> items;
    std::vector<std::pair<std::string, Value>> fields;
    Value() = default;

    Value(bool x)
        : kind(Bool),
          boolean(x) {
    }

    template <class T>
        requires(std::is_arithmetic_v<T> && !std::is_same_v<T, bool>)
    Value(T x)
        : kind(Number),
          number(x) {
        if constexpr (std::is_integral_v<T>) {
            numeric_tag = std::is_signed_v<T> ? 2 : 3;
            bits = uint64_t(x);
        } else {
            numeric_tag = 4;
            bits = std::bit_cast<uint64_t>(double(x));
        }
    }

    Value(const char *x)
        : kind(String),
          text(x) {
    }

    Value(std::string x)
        : kind(String),
          text(std::move(x)) {
    }

    static Value array(std::initializer_list<Value> xs) {
        Value v;
        v.kind = Array;
        v.items = xs;
        return v;
    }

    static Value object(std::initializer_list<std::pair<std::string, Value>> xs) {
        Value v;
        v.kind = Object;
        v.fields = xs;
        return v;
    }

    const Value *field(const char *name) const {
        for (auto &f : fields)
            if (f.first == name)
                return &f.second;
        return nullptr;
    }
};

inline uint32_t crc32(const uint8_t *p, size_t n) {
    uint32_t c = ~0u;
    for (size_t i = 0; i < n; ++i) {
        c ^= p[i];
        for (unsigned b = 0; b < 8; ++b)
            c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1u)));
    }
    return ~c;
}

struct Reader {
    const std::vector<uint8_t> &bytes;
    size_t at = 0, nodes = 0, allocated = 0;

    void reserve_bytes(size_t count, size_t stride) {
        need(stride && count <= (max_bytes - allocated) / stride, "binary allocation limit");
        allocated += count * stride;
    }

    uint64_t uint(unsigned n) {
        need(n <= bytes.size() - at, "truncated binary data");
        uint64_t v = 0;
        for (unsigned i = 0; i < n; ++i)
            v |= uint64_t(bytes[at++]) << (8 * i);
        return v;
    }

    std::string string() {
        auto n = uint(4);
        need(n <= max_bytes && n <= bytes.size() - at, "invalid binary string length");
        reserve_bytes(size_t(n), 1);
        std::string s(reinterpret_cast<const char *>(bytes.data() + at), size_t(n));
        at += size_t(n);
        return s;
    }

    Value value(unsigned depth = 0) {
        need(depth < 64 && ++nodes <= max_elements, "binary data limit");
        Value v;
        switch (uint(1)) {
        case 0:
            return v;
        case 1: {
            auto b = uint(1);
            need(b <= 1, "invalid binary boolean");
            return Value(b == 1);
        }
        case 2:
            return Value(std::bit_cast<int64_t>(uint(8)));
        case 3:
            return Value(uint(8));
        case 4:
            return Value(std::bit_cast<double>(uint(8)));
        case 5:
            return Value(string());
        case 8: {
            Value e(string());
            e.string_tag = 8;
            return e;
        }
        case 6: {
            v.kind = Value::Array;
            auto n = uint(4);
            need(n <= max_elements && n <= bytes.size() - at, "invalid binary array length");
            reserve_bytes(size_t(n), sizeof(Value));
            v.items.reserve(size_t(n));
            for (uint64_t i = 0; i < n; ++i)
                v.items.push_back(value(depth + 1));
            return v;
        }
        case 7: {
            v.kind = Value::Object;
            auto n = uint(4);
            need(n <= max_elements && n <= (bytes.size() - at) / 5, "invalid binary field count");
            reserve_bytes(size_t(n), sizeof(std::pair<std::string, Value>));
            v.fields.reserve(size_t(n));
            for (uint64_t i = 0; i < n; ++i) {
                auto key = string();
                need(!v.field(key.c_str()), "duplicate binary field");
                v.fields.emplace_back(std::move(key), value(depth + 1));
            }
            return v;
        }
        default:
            throw std::runtime_error("invalid binary tag");
        }
    }
};

inline Value decode(const std::vector<uint8_t> &bytes, uint32_t kind, uint32_t schema = 1) {
    need(bytes.size() >= 28 && bytes.size() <= max_bytes + 28, "invalid binary file length");
    need(
        std::memcmp(bytes.data(), "AUVDATA1", 8) == 0,
        "unsupported format: expected AUVDATA1 binary (JSON is obsolete)"
    );
    Reader r{bytes};
    r.at = 8;
    need(r.uint(4) == kind, "unexpected binary record kind");
    need(r.uint(4) == schema, "unsupported binary schema");
    auto n = r.uint(8), crc = r.uint(4);
    need(n == bytes.size() - r.at, "truncated binary payload");
    need(crc32(bytes.data() + r.at, size_t(n)) == crc, "binary checksum mismatch");
    auto v = r.value();
    need(r.at == bytes.size(), "trailing binary data");
    return v;
}

inline Value read(const char *path, uint32_t kind) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    need(bool(file), "cannot read binary data");
    auto n = file.tellg();
    need(n >= 0 && uint64_t(n) <= max_bytes + 28, "binary file too large");
    std::vector<uint8_t> b(static_cast<size_t>(n));
    file.seekg(0);
    file.read(reinterpret_cast<char *>(b.data()), n);
    need(bool(file), "truncated binary file");
    return decode(b, kind);
}

struct Writer {
    std::vector<uint8_t> bytes;

    void uint(uint64_t v, unsigned n) {
        need(bytes.size() + n <= max_bytes + 28, "binary data limit");
        for (unsigned i = 0; i < n; ++i)
            bytes.push_back(uint8_t(v >> (8 * i)));
    }

    void string(const std::string &s) {
        need(s.size() <= max_bytes, "binary string limit");
        uint(s.size(), 4);
        bytes.insert(bytes.end(), s.begin(), s.end());
    }

    void value(const Value &v) {
        switch (v.kind) {
        case Value::Null:
            uint(0, 1);
            break;
        case Value::Bool:
            uint(1, 1);
            uint(v.boolean, 1);
            break;
        case Value::Number:
            uint(v.numeric_tag, 1);
            uint(v.bits, 8);
            break;
        case Value::String:
            uint(v.string_tag, 1);
            string(v.text);
            break;
        case Value::Array:
            uint(6, 1);
            uint(v.items.size(), 4);
            for (auto &x : v.items)
                value(x);
            break;
        case Value::Object:
            uint(7, 1);
            uint(v.fields.size(), 4);
            for (auto &f : v.fields) {
                string(f.first);
                value(f.second);
            }
            break;
        }
    }
};

inline void write(const char *path, uint32_t kind, const Value &v) {
    Writer payload;
    payload.value(v);
    Writer out;
    for (char c : std::string("AUVDATA1"))
        out.uint(uint8_t(c), 1);
    out.uint(kind, 4);
    out.uint(1, 4);
    out.uint(payload.bytes.size(), 8);
    out.uint(crc32(payload.bytes.data(), payload.bytes.size()), 4);
    std::ofstream file(path, std::ios::binary);
    file.write(reinterpret_cast<const char *>(out.bytes.data()), std::streamsize(out.bytes.size()));
    file.write(
        reinterpret_cast<const char *>(payload.bytes.data()), std::streamsize(payload.bytes.size())
    );
    need(bool(file), "cannot write binary data");
}
} // namespace autonomy::data
#endif
