#pragma once

#include <cstddef>

// The fonts compiled into the core library (see cmake/EmbedFonts.cmake, resources/fonts):
// the UI draws with them and TextOutline reads their glyph outlines.
namespace sicdm::fonts {

struct Blob {
    const unsigned char* data;
    std::size_t size;
};

extern const Blob Inter_Regular;
extern const Blob Inter_SemiBold;
extern const Blob JetBrainsMono_Regular;

} // namespace sicdm::fonts
