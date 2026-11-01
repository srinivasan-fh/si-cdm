#pragma once

// SHA-256 (FIPS 180-4), for checking a downloaded release against the feed's digest.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace sicdm {

std::string sha256Hex(std::string_view data);       // 64 lower-case hex digits
std::optional<std::string> sha256File(const std::string& path); // nullopt: can't read it

} // namespace sicdm
