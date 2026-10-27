#include "core/ImageWrite.hpp"

#include "core/FileIO.hpp"

#include <algorithm>
#include <array>

namespace sicdm {

namespace {

std::uint32_t crc32(const std::uint8_t* data, std::size_t n, std::uint32_t crc = 0) {
    static const std::array<std::uint32_t, 256> table = [] {
        std::array<std::uint32_t, 256> t{};
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[i] = c;
        }
        return t;
    }();
    crc = ~crc;
    for (std::size_t i = 0; i < n; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

void put32(std::string& s, std::uint32_t v) {
    s += static_cast<char>(v >> 24), s += static_cast<char>(v >> 16), s += static_cast<char>(v >> 8), s += static_cast<char>(v);
}

void chunk(std::string& out, const char type[4], const std::string& data) {
    put32(out, static_cast<std::uint32_t>(data.size()));
    std::string body(type, 4);
    body += data;
    out += body;
    put32(out, crc32(reinterpret_cast<const std::uint8_t*>(body.data()), body.size()));
}

// Bits out, least significant first (deflate's order).
struct BitWriter {
    std::string out;
    std::uint32_t acc = 0;
    int n = 0;
    void put(std::uint32_t bits, int count) {
        acc |= bits << n;
        n += count;
        while (n >= 8) {
            out += static_cast<char>(acc & 0xFF);
            acc >>= 8;
            n -= 8;
        }
    }
    // A Huffman code: sent most significant bit first.
    void code(std::uint32_t c, int len) {
        std::uint32_t r = 0;
        for (int i = 0; i < len; ++i) r |= ((c >> i) & 1u) << (len - 1 - i);
        put(r, len);
    }
    void flush() {
        if (n > 0) out += static_cast<char>(acc & 0xFF);
        acc = 0, n = 0;
    }
};

// A literal or length symbol's fixed code.
void literal(BitWriter& w, int sym) {
    if (sym < 144) w.code(0x30u + static_cast<std::uint32_t>(sym), 8);
    else if (sym < 256) w.code(0x190u + static_cast<std::uint32_t>(sym - 144), 9);
    else if (sym < 280) w.code(static_cast<std::uint32_t>(sym - 256), 7);
    else w.code(0xC0u + static_cast<std::uint32_t>(sym - 280), 8);
}

constexpr int kLenBase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
constexpr int kLenExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
constexpr int kDistBase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
constexpr int kDistExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

void match(BitWriter& w, int len, int dist) {
    int li = 28;
    while (kLenBase[li] > len) --li;
    literal(w, 257 + li);
    if (kLenExtra[li]) w.put(static_cast<std::uint32_t>(len - kLenBase[li]), kLenExtra[li]);
    int di = 29;
    while (kDistBase[di] > dist) --di;
    w.code(static_cast<std::uint32_t>(di), 5);
    if (kDistExtra[di]) w.put(static_cast<std::uint32_t>(dist - kDistBase[di]), kDistExtra[di]);
}

std::uint32_t adler32(const std::string& data) {
    std::uint32_t a = 1, b = 0;
    for (unsigned char c : data) {
        a = (a + c) % 65521;
        b = (b + a) % 65521;
    }
    return (b << 16) | a;
}

std::string pngFile(int width, int height, int colourType, int bitDepth, const std::string& zdata) {
    std::string out = "\x89PNG\r\n\x1a\n";
    std::string ihdr;
    put32(ihdr, static_cast<std::uint32_t>(width));
    put32(ihdr, static_cast<std::uint32_t>(height));
    ihdr += static_cast<char>(bitDepth);
    ihdr += static_cast<char>(colourType);
    ihdr += '\0', ihdr += '\0', ihdr += '\0';
    chunk(out, "IHDR", ihdr);
    chunk(out, "IDAT", zdata);
    chunk(out, "IEND", "");
    return out;
}

} // namespace

std::string zlibCompress(const std::string& data) {
    BitWriter w;
    w.out += static_cast<char>(0x78), w.out += static_cast<char>(0x01);
    w.put(1, 1); // the last block
    w.put(1, 2); // fixed Huffman codes
    // Greedy LZ77: the longest match among the latest few places the next three bytes were seen.
    constexpr int kWindow = 32768, kHashBits = 15, kChain = 24;
    std::vector<int> head(1u << kHashBits, -1), prev(kWindow, -1);
    const auto* d = reinterpret_cast<const unsigned char*>(data.data());
    const int n = static_cast<int>(data.size());
    auto hashAt = [&](int i) { return ((static_cast<unsigned>(d[i]) << 10) ^ (static_cast<unsigned>(d[i + 1]) << 5) ^ d[i + 2]) & ((1u << kHashBits) - 1); };
    auto insert = [&](int i) {
        if (i + 2 >= n) return;
        const unsigned h = hashAt(i);
        prev[i & (kWindow - 1)] = head[h];
        head[h] = i;
    };
    int i = 0;
    while (i < n) {
        int bestLen = 0, bestDist = 0;
        if (i + 2 < n) {
            int cand = head[hashAt(i)];
            for (int k = 0; k < kChain && cand >= 0 && i - cand < kWindow; ++k) {
                int l = 0;
                const int most = std::min(258, n - i);
                while (l < most && d[cand + l] == d[i + l]) ++l;
                if (l > bestLen) bestLen = l, bestDist = i - cand;
                if (l == most) break;
                const int next = prev[cand & (kWindow - 1)];
                if (next >= cand) break; // overwritten by a newer place
                cand = next;
            }
        }
        if (bestLen >= 3) {
            match(w, bestLen, bestDist);
            for (int k = 0; k < bestLen; ++k) insert(i + k);
            i += bestLen;
        } else {
            literal(w, d[i]);
            insert(i);
            ++i;
        }
    }
    literal(w, 256); // end of block
    w.flush();
    put32(w.out, adler32(data));
    return w.out;
}

std::string encodePngCompressed(int width, int height, int channels, const std::vector<std::uint8_t>& raw) {
    const std::size_t bpp = static_cast<std::size_t>(channels), rowBytes = static_cast<std::size_t>(width) * bpp;
    // Filter 1 (each byte less the one a pixel to its left): smooth shading packs well.
    std::string filtered;
    filtered.reserve((rowBytes + 1) * static_cast<std::size_t>(height));
    for (int y = 0; y < height; ++y) {
        filtered += '\1';
        const std::uint8_t* row = raw.data() + static_cast<std::size_t>(y) * rowBytes;
        for (std::size_t x = 0; x < rowBytes; ++x) filtered += static_cast<char>(static_cast<std::uint8_t>(row[x] - (x >= bpp ? row[x - bpp] : 0)));
    }
    return pngFile(width, height, channels == 3 ? 2 : 0, 8, zlibCompress(filtered));
}

std::string encodePng(int width, int height, int channels, int bitDepth, const std::vector<std::uint8_t>& raw) {
    const std::size_t rowBytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(channels) * static_cast<std::size_t>(bitDepth / 8);
    // Filter byte 0 (none) before each row.
    std::string filtered;
    filtered.reserve((rowBytes + 1) * static_cast<std::size_t>(height));
    for (int y = 0; y < height; ++y) {
        filtered += '\0';
        filtered.append(reinterpret_cast<const char*>(raw.data()) + static_cast<std::size_t>(y) * rowBytes, rowBytes);
    }
    // zlib stream of stored blocks (at most 65535 bytes each), then the Adler-32.
    std::string z;
    z += static_cast<char>(0x78), z += static_cast<char>(0x01);
    std::size_t pos = 0;
    do {
        const std::size_t n = std::min<std::size_t>(65535, filtered.size() - pos);
        const bool last = pos + n == filtered.size();
        z += static_cast<char>(last ? 1 : 0);
        z += static_cast<char>(n & 0xFF), z += static_cast<char>(n >> 8);
        z += static_cast<char>(~n & 0xFF), z += static_cast<char>((~n >> 8) & 0xFF);
        z.append(filtered, pos, n);
        pos += n;
    } while (pos < filtered.size());
    put32(z, adler32(filtered));
    return pngFile(width, height, channels == 3 ? 2 : 0, bitDepth, z); // colour type: RGB or grey
}

bool writePng8(const std::string& path, int width, int height, int channels, const std::vector<std::uint8_t>& pixels, std::string* error) {
    return writeFile(path, encodePng(width, height, channels, 8, pixels), error);
}

bool writePng16(const std::string& path, int width, int height, int channels, const std::vector<std::uint16_t>& pixels, std::string* error) {
    std::vector<std::uint8_t> raw;
    raw.reserve(pixels.size() * 2);
    for (std::uint16_t v : pixels) {
        raw.push_back(static_cast<std::uint8_t>(v >> 8));
        raw.push_back(static_cast<std::uint8_t>(v & 0xFF));
    }
    return writeFile(path, encodePng(width, height, channels, 16, raw), error);
}

} // namespace sicdm
