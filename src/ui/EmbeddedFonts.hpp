#pragma once

#include "core/FontData.hpp"

// UI fonts (core/FontData.hpp) and app icons compiled into the app (see
// cmake/EmbedFonts.cmake, resources/fonts, resources/icon).

// App icon PNGs (rendered by tools/make_icon.py).
namespace sicdm::icons {

using fonts::Blob;
extern const Blob sicdm_256;
extern const Blob sicdm_64;
extern const Blob sicdm_32;

} // namespace sicdm::icons
