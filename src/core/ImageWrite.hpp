#pragma once

// PNG files from pixels: greyscale or RGB, 8 or 16 bits per channel. The image data is
// stored uncompressed (deflate's stored blocks), so files are large but exact and quick;
// encodePngCompressed() packs them (LZ77 with deflate's fixed Huffman codes) for images
// that travel, such as the pictures in a report.

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
std::string encodePngCompressed(int width, int height, int channels, const std::vector<std::uint8_t>& pixels8);

// A zlib stream (RFC 1950) of the data, deflated (RFC 1951) with fixed Huffman codes.
std::string zlibCompress(const std::string& data);

// One PNG chunk: its length, type, data and CRC (for writers that add their own chunks).
std::string pngChunk(const char type[4], const std::string& data);

// A baseline JPEG (JFIF) of RGB pixels (rows top to bottom): YCbCr with the colour halved
// both ways (4:2:0), the standard quantisation tables scaled by `quality` (1..100) and the
// standard Huffman tables, written in every file.
std::string encodeJpeg(int width, int height, const std::vector<std::uint8_t>& rgb, int quality = 90);

} // namespace sicdm
