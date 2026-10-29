#pragma once

// Reading zip packages (3MF, zipped AMF): the central directory, stored and deflated
// members, CRC-checked. Deflate (RFC 1951) is decoded here; nothing else is needed.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

// CRC-32 (IEEE 802.3, as zip and PNG use it) of the bytes, continuing from `crc`.
std::uint32_t crc32Of(const void* data, std::size_t n, std::uint32_t crc = 0);

// Raw deflate data (no zlib or gzip header) decoded into `out`, which must not grow past
// `limit` bytes. False with why when the data is damaged or too large.
bool inflateRaw(const std::uint8_t* data, std::size_t n, std::string& out, std::size_t limit, std::string* why = nullptr);

class ZipReader {
public:
    // The package's members from its central directory. False with why when it isn't a zip.
    bool open(std::string bytes, std::string* why = nullptr);

    struct Entry {
        std::string name; // as stored ("3D/3dmodel.model")
        std::uint16_t method = 0;
        std::uint32_t crc = 0;
        std::uint64_t packed = 0, size = 0, offset = 0; // offset: of its local header
    };
    const std::vector<Entry>& entries() const noexcept { return m_entries; }

    // A member's bytes by its name (a leading '/' and letter case ignored). Nothing (with
    // why) when there is no such member or it can't be read.
    std::optional<std::string> read(const std::string& name, std::string* why = nullptr) const;
    std::optional<std::string> read(const Entry& e, std::string* why = nullptr) const;
    const Entry* find(const std::string& name) const;

private:
    std::string m_bytes;
    std::vector<Entry> m_entries;
};

// Whether bytes start like a zip package.
inline bool looksLikeZip(const std::string& bytes) { return bytes.size() >= 4 && bytes[0] == 'P' && bytes[1] == 'K' && bytes[2] == 3 && bytes[3] == 4; }

} // namespace sicdm
