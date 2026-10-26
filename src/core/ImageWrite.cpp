#include "core/ImageWrite.hpp"

#include "core/FileIO.hpp"

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

} // namespace

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
    std::uint32_t a = 1, b = 0;
    for (unsigned char c : filtered) {
        a = (a + c) % 65521;
        b = (b + a) % 65521;
    }
    put32(z, (b << 16) | a);

    std::string out = "\x89PNG\r\n\x1a\n";
    std::string ihdr;
    put32(ihdr, static_cast<std::uint32_t>(width));
    put32(ihdr, static_cast<std::uint32_t>(height));
    ihdr += static_cast<char>(bitDepth);
    ihdr += static_cast<char>(channels == 3 ? 2 : 0); // colour type: RGB or grey
    ihdr += '\0', ihdr += '\0', ihdr += '\0';
    chunk(out, "IHDR", ihdr);
    chunk(out, "IDAT", z);
    chunk(out, "IEND", "");
    return out;
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
