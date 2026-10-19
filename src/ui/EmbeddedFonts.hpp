#pragma once

#include <cstddef>

// UI fonts and app icons compiled into the app (see cmake/EmbedFonts.cmake,
// resources/fonts, resources/icon).
namespace sicdm::fonts {

struct Blob {
    const unsigned char* data;
    std::size_t size;
};

extern const Blob Inter_Regular;
extern const Blob Inter_SemiBold;
extern const Blob JetBrainsMono_Regular;

} // namespace sicdm::fonts

// App icon PNGs (rendered by tools/make_icon.py).
namespace sicdm::icons {

using fonts::Blob;
extern const Blob sicdm_256;
extern const Blob sicdm_64;
extern const Blob sicdm_32;

} // namespace sicdm::icons
