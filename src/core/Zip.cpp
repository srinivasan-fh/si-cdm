// Zip packages and deflate (Zip.hpp).

#include "core/Zip.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>

namespace sicdm {

namespace {

const std::array<std::uint32_t, 256>& crcTable() {
    static const std::array<std::uint32_t, 256> t = [] {
        std::array<std::uint32_t, 256> r{};
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            r[i] = c;
        }
        return r;
    }();
    return t;
}

bool fail(std::string* why, const std::string& w) {
    if (why) *why = w;
    return false;
}

// --- deflate -------------------------------------------------------------------------------

// Bits least significant first, as deflate packs them.
struct Bits {
    const std::uint8_t* p;
    std::size_t n, pos = 0;
    std::uint64_t buf = 0;
    int count = 0;
    bool over = false; // read past the end

    void fill() {
        while (count <= 56) { // past the end: zeros, flagged once they are used
            buf |= static_cast<std::uint64_t>(pos < n ? p[pos] : 0) << count;
            ++pos;
            count += 8;
        }
    }
    std::uint32_t peek(int k) {
        if (count < k) fill();
        return static_cast<std::uint32_t>(buf & ((std::uint64_t{1} << k) - 1));
    }
    void drop(int k) {
        buf >>= k;
        count -= k;
        if (pos * 8 - static_cast<std::size_t>(count) > n * 8) over = true;
    }
    std::uint32_t take(int k) {
        if (k == 0) return 0;
        const std::uint32_t v = peek(k);
        drop(k);
        return v;
    }
    void toByte() { drop(count % 8); }
};

// A canonical Huffman code: a table on the first kFast bits, the rest decoded a bit at a time.
struct Huffman {
    static constexpr int kFast = 10;
    std::array<std::uint16_t, 16> counts{};
    std::vector<std::uint16_t> symbols;
    std::vector<std::uint16_t> fast; // (length << 12) | symbol, 0: not in the table

    bool build(const std::uint8_t* lengths, int n) {
        counts.fill(0);
        for (int i = 0; i < n; ++i) ++counts[lengths[i]];
        counts[0] = 0;
        int left = 1; // over-subscribed codes can't be decoded
        for (int len = 1; len < 16; ++len) {
            left = left * 2 - counts[static_cast<std::size_t>(len)];
            if (left < 0) return false;
        }
        std::array<std::uint16_t, 16> offs{};
        for (int len = 1; len < 15; ++len) offs[static_cast<std::size_t>(len + 1)] = static_cast<std::uint16_t>(offs[static_cast<std::size_t>(len)] + counts[static_cast<std::size_t>(len)]);
        symbols.assign(static_cast<std::size_t>(n), 0);
        for (int i = 0; i < n; ++i)
            if (lengths[i]) symbols[offs[lengths[i]]++] = static_cast<std::uint16_t>(i);
        // The table: each short code's bits reversed (read least significant first), every
        // completion of its remaining bits.
        fast.assign(std::size_t{1} << kFast, 0);
        int code = 0, index = 0;
        for (int len = 1; len <= kFast; ++len) {
            for (int k = 0; k < counts[static_cast<std::size_t>(len)]; ++k, ++code, ++index) {
                int rev = 0;
                for (int b = 0; b < len; ++b) rev |= ((code >> b) & 1) << (len - 1 - b);
                for (int fill = rev; fill < (1 << kFast); fill += 1 << len)
                    fast[static_cast<std::size_t>(fill)] = static_cast<std::uint16_t>((len << 12) | symbols[static_cast<std::size_t>(index)]);
            }
            code <<= 1;
        }
        return true;
    }

    int decode(Bits& in) const {
        const std::uint32_t look = in.peek(kFast);
        if (const std::uint16_t e = fast[look]) {
            in.drop(e >> 12);
            return e & 0xFFF;
        }
        // A longer code: bit by bit from its start.
        int code = 0, first = 0, index = 0;
        for (int len = 1; len < 16; ++len) {
            code |= static_cast<int>(in.take(1));
            const int count = counts[static_cast<std::size_t>(len)];
            if (code - count < first) return symbols[static_cast<std::size_t>(index + (code - first))];
            index += count;
            first += count;
            first <<= 1;
            code <<= 1;
        }
        return -1;
    }
};

constexpr std::uint16_t kLenBase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
constexpr std::uint8_t kLenExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
constexpr std::uint16_t kDistBase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
constexpr std::uint8_t kDistExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

bool inflateBlock(Bits& in, const Huffman& lit, const Huffman& dist, std::string& out, std::size_t limit, std::string* why) {
    for (;;) {
        const int sym = lit.decode(in);
        if (sym < 0 || in.over) return fail(why, "damaged compressed data (a bad code)");
        if (sym < 256) {
            if (out.size() >= limit) return fail(why, "the data unpacks to more than it should");
            out.push_back(static_cast<char>(sym));
            continue;
        }
        if (sym == 256) return true;
        const int li = sym - 257;
        if (li >= 29) return fail(why, "damaged compressed data (a bad length)");
        const std::size_t len = kLenBase[li] + in.take(kLenExtra[li]);
        const int di = dist.decode(in);
        if (di < 0 || di >= 30) return fail(why, "damaged compressed data (a bad distance)");
        const std::size_t d = kDistBase[di] + in.take(kDistExtra[di]);
        if (d > out.size()) return fail(why, "damaged compressed data (a distance before the start)");
        if (out.size() + len > limit) return fail(why, "the data unpacks to more than it should");
        const std::size_t from = out.size() - d;
        for (std::size_t k = 0; k < len; ++k) out.push_back(out[from + k]); // (may overlap what it writes)
    }
}

std::uint16_t le16(const std::string& s, std::size_t at) {
    return static_cast<std::uint16_t>(static_cast<std::uint8_t>(s[at]) | static_cast<std::uint8_t>(s[at + 1]) << 8);
}
std::uint32_t le32(const std::string& s, std::size_t at) { return le16(s, at) | static_cast<std::uint32_t>(le16(s, at + 2)) << 16; }
std::uint64_t le64(const std::string& s, std::size_t at) { return le32(s, at) | static_cast<std::uint64_t>(le32(s, at + 4)) << 32; }

std::string folded(std::string s) {
    if (!s.empty() && s[0] == '/') s.erase(0, 1);
    for (char& c : s) c = c == '\\' ? '/' : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

} // namespace

std::uint32_t crc32Of(const void* data, std::size_t n, std::uint32_t crc) {
    const auto& t = crcTable();
    const auto* p = static_cast<const std::uint8_t*>(data);
    crc = ~crc;
    for (std::size_t i = 0; i < n; ++i) crc = t[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

bool inflateRaw(const std::uint8_t* data, std::size_t n, std::string& out, std::size_t limit, std::string* why) {
    Bits in{data, n};
    for (;;) {
        const std::uint32_t last = in.take(1), type = in.take(2);
        if (in.over) return fail(why, "compressed data cut short");
        if (type == 0) { // stored
            in.toByte();
            const std::uint32_t len = in.take(16), nlen = in.take(16);
            if (in.over || (len ^ 0xFFFF) != nlen) return fail(why, "damaged compressed data (a stored block)");
            if (out.size() + len > limit) return fail(why, "the data unpacks to more than it should");
            for (std::uint32_t k = 0; k < len; ++k) out.push_back(static_cast<char>(in.take(8)));
            if (in.over) return fail(why, "compressed data cut short");
        } else if (type == 1) { // fixed codes
            static const std::pair<Huffman, Huffman> fixed = [] {
                std::uint8_t l[288], d[30];
                for (int i = 0; i < 288; ++i) l[i] = i < 144 ? 8 : i < 256 ? 9 : i < 280 ? 7 : 8;
                std::fill(d, d + 30, std::uint8_t{5});
                std::pair<Huffman, Huffman> h;
                h.first.build(l, 288);
                h.second.build(d, 30);
                return h;
            }();
            if (!inflateBlock(in, fixed.first, fixed.second, out, limit, why)) return false;
        } else if (type == 2) { // codes of its own
            const int nlit = static_cast<int>(in.take(5)) + 257, ndist = static_cast<int>(in.take(5)) + 1, ncode = static_cast<int>(in.take(4)) + 4;
            if (nlit > 286 || ndist > 30) return fail(why, "damaged compressed data (bad code counts)");
            static const int kOrder[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
            std::uint8_t cl[19] = {};
            for (int i = 0; i < ncode; ++i) cl[kOrder[i]] = static_cast<std::uint8_t>(in.take(3));
            Huffman lens;
            if (!lens.build(cl, 19)) return fail(why, "damaged compressed data (bad code lengths)");
            std::uint8_t ll[286 + 30] = {};
            for (int i = 0; i < nlit + ndist;) {
                const int sym = lens.decode(in);
                if (sym < 0 || in.over) return fail(why, "damaged compressed data (bad code lengths)");
                if (sym < 16) ll[i++] = static_cast<std::uint8_t>(sym);
                else {
                    int rep = 0;
                    std::uint8_t v = 0;
                    if (sym == 16) {
                        if (i == 0) return fail(why, "damaged compressed data (a repeat with nothing before it)");
                        v = ll[i - 1];
                        rep = 3 + static_cast<int>(in.take(2));
                    } else if (sym == 17) rep = 3 + static_cast<int>(in.take(3));
                    else rep = 11 + static_cast<int>(in.take(7));
                    if (i + rep > nlit + ndist) return fail(why, "damaged compressed data (too many code lengths)");
                    while (rep--) ll[i++] = v;
                }
            }
            if (ll[256] == 0) return fail(why, "damaged compressed data (no end-of-block code)");
            Huffman lit, dist;
            if (!lit.build(ll, nlit) || !dist.build(ll + nlit, ndist)) return fail(why, "damaged compressed data (bad codes)");
            if (!inflateBlock(in, lit, dist, out, limit, why)) return false;
        } else {
            return fail(why, "damaged compressed data (an unknown block type)");
        }
        if (last) return true;
    }
}

bool ZipReader::open(std::string bytes, std::string* why) {
    m_bytes = std::move(bytes);
    m_entries.clear();
    const std::string& s = m_bytes;
    if (s.size() < 22) return fail(why, "not a zip package (too short)");
    // The end of the central directory: the last record with its signature (a comment may follow).
    std::size_t eocd = std::string::npos;
    for (std::size_t i = s.size() - 22 + 1; i-- > 0 && s.size() - i <= 22 + 65535;)
        if (le32(s, i) == 0x06054b50u) {
            eocd = i;
            break;
        }
    if (eocd == std::string::npos) return fail(why, "not a zip package (no central directory)");
    std::uint64_t count = le16(s, eocd + 10), cdSize = le32(s, eocd + 12), cdOffset = le32(s, eocd + 16);
    // Zip64: the sizes are in the zip64 record its locator points at.
    if ((count == 0xFFFF || cdOffset == 0xFFFFFFFFu) && eocd >= 20 && le32(s, eocd - 20) == 0x07064b50u) {
        const std::uint64_t z = le64(s, eocd - 20 + 8);
        if (z + 56 <= s.size() && le32(s, static_cast<std::size_t>(z)) == 0x06064b50u) {
            count = le64(s, static_cast<std::size_t>(z) + 32);
            cdSize = le64(s, static_cast<std::size_t>(z) + 40);
            cdOffset = le64(s, static_cast<std::size_t>(z) + 48);
        }
    }
    if (cdOffset + cdSize > s.size()) return fail(why, "damaged zip package (the directory is outside the file)");
    std::size_t at = static_cast<std::size_t>(cdOffset);
    for (std::uint64_t k = 0; k < count; ++k) {
        if (at + 46 > s.size() || le32(s, at) != 0x02014b50u) return fail(why, "damaged zip package (a bad directory entry)");
        Entry e;
        e.method = le16(s, at + 10);
        e.crc = le32(s, at + 16);
        e.packed = le32(s, at + 20);
        e.size = le32(s, at + 24);
        const std::size_t nameLen = le16(s, at + 28), extraLen = le16(s, at + 30), commentLen = le16(s, at + 32);
        e.offset = le32(s, at + 42);
        if (at + 46 + nameLen + extraLen > s.size()) return fail(why, "damaged zip package (a bad directory entry)");
        e.name = s.substr(at + 46, nameLen);
        // Zip64 sizes and offset, in that order, for the fields that are full.
        for (std::size_t x = at + 46 + nameLen; x + 4 <= at + 46 + nameLen + extraLen;) {
            const std::uint16_t id = le16(s, x), len = le16(s, x + 2);
            if (id == 0x0001) {
                std::size_t y = x + 4;
                if (e.size == 0xFFFFFFFFu && y + 8 <= x + 4 + len) e.size = le64(s, y), y += 8;
                if (e.packed == 0xFFFFFFFFu && y + 8 <= x + 4 + len) e.packed = le64(s, y), y += 8;
                if (e.offset == 0xFFFFFFFFu && y + 8 <= x + 4 + len) e.offset = le64(s, y);
            }
            x += 4 + len;
        }
        m_entries.push_back(std::move(e));
        at += 46 + nameLen + extraLen + commentLen;
    }
    return true;
}

const ZipReader::Entry* ZipReader::find(const std::string& name) const {
    const std::string want = folded(name);
    for (const Entry& e : m_entries)
        if (folded(e.name) == want) return &e;
    return nullptr;
}

std::optional<std::string> ZipReader::read(const std::string& name, std::string* why) const {
    const Entry* e = find(name);
    if (!e) {
        fail(why, "no " + name + " in the package");
        return std::nullopt;
    }
    return read(*e, why);
}

std::optional<std::string> ZipReader::read(const Entry& e, std::string* why) const {
    const std::string& s = m_bytes;
    if (e.offset + 30 > s.size() || le32(s, static_cast<std::size_t>(e.offset)) != 0x04034b50u) {
        fail(why, "damaged zip package (" + e.name + " isn't where the directory says)");
        return std::nullopt;
    }
    const std::size_t start = static_cast<std::size_t>(e.offset) + 30 + le16(s, static_cast<std::size_t>(e.offset) + 26) + le16(s, static_cast<std::size_t>(e.offset) + 28);
    if (start + e.packed > s.size()) {
        fail(why, "damaged zip package (" + e.name + " is cut short)");
        return std::nullopt;
    }
    std::string out;
    const auto* p = reinterpret_cast<const std::uint8_t*>(s.data()) + start;
    if (e.method == 0) out.assign(reinterpret_cast<const char*>(p), static_cast<std::size_t>(e.packed));
    else if (e.method == 8) {
        constexpr std::uint64_t kMax = std::uint64_t{2} << 30; // 2 GB unpacked at most
        if (e.size > kMax) {
            fail(why, e.name + " is too large to unpack");
            return std::nullopt;
        }
        out.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(e.size, e.packed * 1032 + 4096))); // (deflate packs at most ~1032:1)
        std::string w;
        if (!inflateRaw(p, static_cast<std::size_t>(e.packed), out, static_cast<std::size_t>(e.size), &w)) {
            fail(why, e.name + ": " + w);
            return std::nullopt;
        }
    } else {
        fail(why, e.name + " is packed a way this reader doesn't know (method " + std::to_string(e.method) + ")");
        return std::nullopt;
    }
    if (out.size() != e.size || crc32Of(out.data(), out.size()) != e.crc) {
        fail(why, e.name + " is damaged (its checksum doesn't match)");
        return std::nullopt;
    }
    return out;
}

} // namespace sicdm
