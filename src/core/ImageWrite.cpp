#include "core/ImageWrite.hpp"

#include "core/FileIO.hpp"

#include <algorithm>
#include <array>
#include <cmath>

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


std::string pngChunk(const char type[4], const std::string& data) {
    std::string out;
    chunk(out, type, data);
    return out;
}

// --- JPEG (baseline, ITU T.81) ----------------------------------------------------------

namespace {

// Zigzag position -> the coefficient's place in the 8 x 8 block (row major).
constexpr std::uint8_t kZigzag[64] = {0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,  12, 19, 26, 33, 40, 48,
                                      41, 34, 27, 20, 13, 6,  7,  14, 21, 28, 35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23,
                                      30, 37, 44, 51, 58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};

// The example quantisation tables of the standard (Annex K), row major.
constexpr std::uint8_t kLumaQ[64] = {16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,  14, 13, 16, 24, 40,  57,
                                     69, 56, 14, 17, 22,  29,  51,  87,  80, 62, 18, 22, 37,  56,  68,  109, 103, 77, 24, 35, 55,  64,
                                     81, 104, 113, 92, 49, 64, 78,  87,  103, 121, 120, 101, 72, 92, 95,  98,  112, 100, 103, 99};
constexpr std::uint8_t kChromaQ[64] = {17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99, 24, 26, 56, 99, 99, 99,
                                       99, 99, 47, 66, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
                                       99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};

// The standard's Huffman tables: how many codes of each length (1..16), then the symbols.
constexpr std::uint8_t kDcLumaBits[16] = {0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
constexpr std::uint8_t kDcChromaBits[16] = {0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
constexpr std::uint8_t kDcValues[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
constexpr std::uint8_t kAcLumaBits[16] = {0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d};
constexpr std::uint8_t kAcLumaValues[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81,
    0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18,
    0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
    0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5,
    0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};
constexpr std::uint8_t kAcChromaBits[16] = {0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77};
constexpr std::uint8_t kAcChromaValues[162] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08,
    0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25,
    0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
    0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
    0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4,
    0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

// Codes for a table's symbols (canonical Huffman: codes of each length counted up in turn).
struct HuffCode {
    std::uint16_t code[256]{};
    std::uint8_t size[256]{};
};

HuffCode huffCodes(const std::uint8_t bits[16], const std::uint8_t* values) {
    HuffCode h;
    std::uint16_t code = 0;
    std::size_t k = 0;
    for (int len = 1; len <= 16; ++len) {
        for (int i = 0; i < bits[len - 1]; ++i, ++k) {
            h.code[values[k]] = code++;
            h.size[values[k]] = static_cast<std::uint8_t>(len);
        }
        code = static_cast<std::uint16_t>(code << 1);
    }
    return h;
}

// Entropy-coded bits, most significant first, with a 0 stuffed after every 0xFF byte.
struct JpegBits {
    std::string& out;
    std::uint32_t acc = 0;
    int n = 0;
    void put(std::uint32_t bits, int count) {
        for (int i = count - 1; i >= 0; --i) {
            acc = (acc << 1) | ((bits >> i) & 1u);
            if (++n == 8) {
                out += static_cast<char>(acc);
                if ((acc & 0xFF) == 0xFF) out += '\0';
                acc = 0, n = 0;
            }
        }
    }
    void flush() { // pad with ones
        while (n) put(1, 1);
    }
};

int magnitudeBits(int v) {
    v = v < 0 ? -v : v;
    int s = 0;
    while (v) ++s, v >>= 1;
    return s;
}

} // namespace

std::string encodeJpeg(int width, int height, const std::vector<std::uint8_t>& rgb, int quality) {
    if (width <= 0 || height <= 0 || rgb.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3) return {};
    quality = std::clamp(quality, 1, 100);
    const int scale = quality < 50 ? 5000 / quality : 200 - 2 * quality;
    std::uint8_t qt[2][64]; // row major
    for (int i = 0; i < 64; ++i) {
        qt[0][i] = static_cast<std::uint8_t>(std::clamp((kLumaQ[i] * scale + 50) / 100, 1, 255));
        qt[1][i] = static_cast<std::uint8_t>(std::clamp((kChromaQ[i] * scale + 50) / 100, 1, 255));
    }
    static const HuffCode dcCode[2] = {huffCodes(kDcLumaBits, kDcValues), huffCodes(kDcChromaBits, kDcValues)};
    static const HuffCode acCode[2] = {huffCodes(kAcLumaBits, kAcLumaValues), huffCodes(kAcChromaBits, kAcChromaValues)};
    // The DCT's basis: c[u][x] = C(u)/2 cos((2x + 1) u pi / 16).
    static const auto basis = [] {
        std::array<std::array<double, 8>, 8> c{};
        for (int u = 0; u < 8; ++u)
            for (int x = 0; x < 8; ++x) c[u][x] = (u == 0 ? std::sqrt(0.5) : 1.0) * 0.5 * std::cos((2 * x + 1) * u * 3.14159265358979323846 / 16);
        return c;
    }();
    std::string out;
    auto u16 = [&](int v) { out += static_cast<char>((v >> 8) & 0xFF), out += static_cast<char>(v & 0xFF); };
    auto marker = [&](int m) { out += '\xFF', out += static_cast<char>(m); };
    marker(0xD8); // SOI
    marker(0xE0); // APP0: JFIF 1.1, no density
    u16(16);
    out.append("JFIF\0", 5);
    out += '\x01', out += '\x01', out += '\0';
    u16(1), u16(1);
    out += '\0', out += '\0';
    for (int t = 0; t < 2; ++t) { // DQT, in zigzag order
        marker(0xDB);
        u16(67);
        out += static_cast<char>(t);
        for (int k = 0; k < 64; ++k) out += static_cast<char>(qt[t][kZigzag[k]]);
    }
    marker(0xC0); // SOF0: 8 bits, Y at 2 x 2, Cb and Cr at 1 x 1
    u16(17);
    out += '\x08';
    u16(height), u16(width);
    out += '\x03';
    out += '\x01', out += '\x22', out += '\x00';
    out += '\x02', out += '\x11', out += '\x01';
    out += '\x03', out += '\x11', out += '\x01';
    auto dht = [&](int cls, int id, const std::uint8_t bits[16], const std::uint8_t* values) {
        int count = 0;
        for (int i = 0; i < 16; ++i) count += bits[i];
        marker(0xC4);
        u16(19 + count);
        out += static_cast<char>(cls << 4 | id);
        out.append(reinterpret_cast<const char*>(bits), 16);
        out.append(reinterpret_cast<const char*>(values), static_cast<std::size_t>(count));
    };
    dht(0, 0, kDcLumaBits, kDcValues);
    dht(1, 0, kAcLumaBits, kAcLumaValues);
    dht(0, 1, kDcChromaBits, kDcValues);
    dht(1, 1, kAcChromaBits, kAcChromaValues);
    marker(0xDA); // SOS
    u16(12);
    out += '\x03';
    out += '\x01', out += '\x00';
    out += '\x02', out += '\x11';
    out += '\x03', out += '\x11';
    out += '\x00', out += '\x3F', out += '\x00';

    JpegBits bits{out};
    int lastDc[3] = {0, 0, 0};
    auto pixel = [&](int x, int y) { // clamped at the edges
        x = std::clamp(x, 0, width - 1), y = std::clamp(y, 0, height - 1);
        return &rgb[(static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)) * 3];
    };
    auto block = [&](const double in[64], int comp) {
        const int t = comp == 0 ? 0 : 1;
        double tmp[64], f[64];
        for (int y = 0; y < 8; ++y) // rows
            for (int u = 0; u < 8; ++u) {
                double s = 0;
                for (int x = 0; x < 8; ++x) s += basis[static_cast<std::size_t>(u)][static_cast<std::size_t>(x)] * in[y * 8 + x];
                tmp[y * 8 + u] = s;
            }
        for (int u = 0; u < 8; ++u) // columns
            for (int v = 0; v < 8; ++v) {
                double s = 0;
                for (int y = 0; y < 8; ++y) s += basis[static_cast<std::size_t>(v)][static_cast<std::size_t>(y)] * tmp[y * 8 + u];
                f[v * 8 + u] = s;
            }
        int q[64];
        for (int k = 0; k < 64; ++k) q[k] = static_cast<int>(std::lround(f[kZigzag[k]] / qt[t][kZigzag[k]]));
        // DC: the change from the last block of this component.
        const int diff = q[0] - lastDc[comp];
        lastDc[comp] = q[0];
        const int ds = magnitudeBits(diff);
        bits.put(dcCode[t].code[ds], dcCode[t].size[ds]);
        if (ds) bits.put(static_cast<std::uint32_t>(diff < 0 ? diff + (1 << ds) - 1 : diff), ds);
        // AC: runs of zeros and the next value; 16 zeros (0xF0); end of block (0x00).
        int run = 0;
        for (int k = 1; k < 64; ++k) {
            if (q[k] == 0) {
                ++run;
                continue;
            }
            while (run > 15) bits.put(acCode[t].code[0xF0], acCode[t].size[0xF0]), run -= 16;
            const int s = std::min(magnitudeBits(q[k]), 10);
            const int v = std::clamp(q[k], -1023, 1023);
            const int sym = run << 4 | s;
            bits.put(acCode[t].code[sym], acCode[t].size[sym]);
            bits.put(static_cast<std::uint32_t>(v < 0 ? v + (1 << s) - 1 : v), s);
            run = 0;
        }
        if (run) bits.put(acCode[t].code[0x00], acCode[t].size[0x00]);
    };
    double yb[4][64], cb[64], cr[64];
    for (int my = 0; my < height; my += 16)
        for (int mx = 0; mx < width; mx += 16) {
            for (int y = 0; y < 16; ++y)
                for (int x = 0; x < 16; ++x) {
                    const std::uint8_t* p = pixel(mx + x, my + y);
                    const double r = p[0], g = p[1], b = p[2];
                    yb[(y / 8) * 2 + x / 8][(y % 8) * 8 + x % 8] = 0.299 * r + 0.587 * g + 0.114 * b - 128;
                }
            for (int y = 0; y < 8; ++y)
                for (int x = 0; x < 8; ++x) { // the colour, averaged over 2 x 2
                    double sb = 0, sr = 0;
                    for (int k = 0; k < 4; ++k) {
                        const std::uint8_t* p = pixel(mx + 2 * x + (k & 1), my + 2 * y + (k >> 1));
                        const double r = p[0], g = p[1], b = p[2];
                        sb += -0.168736 * r - 0.331264 * g + 0.5 * b;
                        sr += 0.5 * r - 0.418688 * g - 0.081312 * b;
                    }
                    cb[y * 8 + x] = sb / 4, cr[y * 8 + x] = sr / 4; // (+128, then -128 for the DCT)
                }
            for (int k = 0; k < 4; ++k) block(yb[k], 0);
            block(cb, 1);
            block(cr, 2);
        }
    bits.flush();
    marker(0xD9); // EOI
    return out;
}

} // namespace sicdm
