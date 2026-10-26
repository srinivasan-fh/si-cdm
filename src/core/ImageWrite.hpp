#pragma once

// PNG files from pixels: greyscale or RGB, 8 or 16 bits per channel. The image data is
// stored uncompressed (deflate's stored blocks), so files are large but exact and quick.

#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

// `pixels`: rows top to bottom, `channels` (1 grey, 3 RGB) samples per pixel; 8-bit
// samples in `pixels8`, or 16-bit ones in `pixels16` (big-endian in the file).
bool writePng8(const std::string& path, int width, int height, int channels, const std::vector<std::uint8_t>& pixels8, std::string* error = nullptr);
bool writePng16(const std::string& path, int width, int height, int channels, const std::vector<std::uint16_t>& pixels16, std::string* error = nullptr);

// The bytes of such a file (for tests and in-memory use).
std::string encodePng(int width, int height, int channels, int bitDepth, const std::vector<std::uint8_t>& rawBigEndianSamples);

} // namespace sicdm
