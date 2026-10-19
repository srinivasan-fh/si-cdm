#pragma once

#include <cstddef>

// UI fonts compiled into the app (see cmake/EmbedFonts.cmake, resources/fonts).
namespace sicdm::fonts {

struct Blob {
    const unsigned char* data;
    std::size_t size;
};

extern const Blob Inter_Regular;
extern const Blob Inter_SemiBold;
extern const Blob JetBrainsMono_Regular;

} // namespace sicdm::fonts
